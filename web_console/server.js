import express from 'express';
import cors from 'cors';
import { spawn, execFile } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';
import { discoverCamera } from './cameraDiscovery.js';
import { automaticMapName, assertMappingDuration, inspectMapPair, inspectMapQuality, qualityOptions, saveMap } from './mapStorage.js';
import { installNavigationRoutes } from './navigationRoutes.js';
import { terminateProcess } from './managedProcess.js';
import { createRunManifest } from './runManifest.js';
import { installVisionRoutes } from './visionRoutes.js';
import { installCalibrationRoutes } from './calibrationRoutes.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const repoRoot = path.resolve(__dirname, '..');
const rosDistro = process.env.ROS_DISTRO || 'lyrical';

// 建图会话证据采集：现场 8 段碎片化会话事后无法对账，根因是建图生命周期完全不产证据。
// 这里接 scripts/run_manifest.py（与导航共用一套 manifest/summary 规范），全程 best-effort。
const mappingRun = createRunManifest({
    scriptPath: path.join(repoRoot, 'scripts', 'run_manifest.py'),
    cwd: repoRoot,
    runsDir: process.env.ROBOT_RUNS_DIR || ''
});

// 诊断包用的有限长度日志环：只保留后端自身消息，不含环境变量或用户文件。
const consoleRing = [];
for (const level of ['log', 'warn', 'error']) {
    const original = console[level].bind(console);
    console[level] = (...args) => {
        try {
            consoleRing.push({ at: Date.now(), level,
                line: args.map(a => (a instanceof Error ? a.stack || a.message : String(a))).join(' ').slice(0, 600) });
            if (consoleRing.length > 400) consoleRing.shift();
        } catch {}
        original(...args);
    };
}

const app = express();
const port = Number(process.env.ROBOT_API_PORT || 3000);

app.use(cors());
app.use(express.json());

// 记录所有后端的子进程
let activeProcesses = {};
let mappingStopping = false;
let mappingStarting = false;
let mappingStartedAt = 0;
let mappingMode = 'manual';
let lastSavedMap = null;
// 建图运行代次：进程 exit 回调与 abort 共用 finalizeMapping，
// 只有代次匹配且未结束时才允许清状态，防止上一轮进程的迟到 exit 清掉新一轮。
let mappingGeneration = 0;
let abortInFlight = null;
// 质量门槛快照：每次操作只取一次，保存/恢复/列表/导航启动必须用同一份。
function mapQualityGate() {
    try {
        return qualityOptions(loadConfig().save_map);
    } catch (error) {
        // 配置写错时回落严格默认，绝不允许把门禁放宽。
        console.error('[map-quality] 配置非法，回落 MAP_QUALITY:', error.message);
        return qualityOptions({});
    }
}
function validateSavedMap(saved) {
    if (!saved || typeof saved.path !== 'string' || !saved.path) return null;
    try { return inspectMapPair(saved.path, mapQualityGate()); }
    catch { return null; }
}
function rememberMap(saved) {
    // saveMap 已通过 inspectMapPair；这里再防一次，空图不能写成“最近保存”。
    if (!validateSavedMap(saved)) return;
    lastSavedMap = { ...saved, savedAt: Date.now() };
    const dest = path.join(loadConfig().maps_dir, '.console-last-saved.json');
    fs.writeFileSync(dest + '.tmp', JSON.stringify(lastSavedMap));
    fs.renameSync(dest + '.tmp', dest);
}
function restoredMap() {
    if (lastSavedMap && validateSavedMap(lastSavedMap)) return lastSavedMap;
    lastSavedMap = null;
    try {
        const saved = JSON.parse(fs.readFileSync(path.join(loadConfig().maps_dir, '.console-last-saved.json'), 'utf8'));
        // 旧版本可能保存了空白/损坏地图；不能把它当作可导航的“最近保存”。
        if (validateSavedMap(saved)) return saved;
    } catch {}
    return null;
}
const taskRunning = (config, name) => (config.tasks[name] || []).some(task => !!activeProcesses[task.id]);

// 加载配置文件
function loadConfig() {
    try {
        const configPath = path.join(__dirname, 'ros_config.json');
        const data = fs.readFileSync(configPath, 'utf8');
        const config = JSON.parse(data);
        // Resolve checked-in relative paths from this app, independent of cwd/user.
        config.maps_dir = path.resolve(__dirname, process.env.ROBOT_MAPS_DIR || config.maps_dir || '../maps');
        if (config.save_map) {
            config.save_map.save_dir = path.resolve(__dirname, process.env.ROBOT_MAPS_DIR || config.save_map.save_dir || config.maps_dir);
        }
        return config;
    } catch (err) {
        console.error('无法读取配置文件:', err);
        return { tasks: { mapping: [] } };
    }
}

function shellQuote(value) {
    return `'${String(value).replace(/'/g, `'\\''`)}'`;
}

function resolveWorkspaceSetup() {
    const explicitSetup = process.env.ROBOT_WORKSPACE_SETUP?.trim();
    if (explicitSetup) {
        return explicitSetup;
    }

    const explicitWorkspace = process.env.ROBOT_WORKSPACE?.trim();
    const candidates = [
        explicitWorkspace ? path.join(explicitWorkspace, 'install/setup.bash') : null,
        path.resolve(__dirname, '../robot_ws/install/setup.bash')
    ].filter(Boolean);

    return candidates.find((candidate) => fs.existsSync(candidate)) || '';
}

function buildShellCommand(command, args = []) {
    const safeArgs = (args || []).map((arg) => shellQuote(arg));
    return [command, ...safeArgs].join(' ');
}

function buildRosCommand(command, args = []) {
    const steps = [`source ${shellQuote(`/opt/ros/${rosDistro}/setup.bash`)}`];
    const workspaceSetup = resolveWorkspaceSetup();

    if (workspaceSetup) {
        steps.push(`source ${shellQuote(workspaceSetup)}`);
    }

    steps.push(buildShellCommand(command, args));
    return steps.join(' && ');
}

async function checkMappingInput(action = '建图', auto = false) {
    const script = path.resolve(__dirname, '../scripts/mapping_preflight.py');
    await new Promise((resolve, reject) => {
        const proc = spawn('bash', ['-c', buildRosCommand('python3', [script, ...(auto ? ['--auto'] : [])])], {
            detached: true, stdio: ['ignore', 'pipe', 'pipe']
        });
        let output = '';
        const collect = chunk => { output = (output + chunk.toString()).slice(-4000); };
        proc.stdout.on('data', collect);
        proc.stderr.on('data', collect);
        const timer = setTimeout(() => {
            try { process.kill(-proc.pid, 'SIGKILL'); } catch (error) {
                if (error.code !== 'ESRCH') console.error(error.message);
            }
        }, 8000);
        proc.once('error', error => { clearTimeout(timer); reject(error); });
        proc.once('close', code => {
            clearTimeout(timer);
            if (code === 0) return resolve();
            let missing = '实时雷达、里程计或 TF';
            try { missing = JSON.parse(output.trim().split('\n').at(-1)).missing.join('、'); } catch {}
            reject(new Error(`不能开始${action}：缺少${missing}。请检查小车电源、Wi-Fi 和 micro-ROS Agent。`));
        });
    });
}

// 摄像头地址：配置优先；为空时用固件 UDP 33333 自动发现，并做短缓存。
const cameraDiscoveryOptions = {
    timeoutMs: Number(process.env.ROBOT_CAMERA_DISCOVERY_TIMEOUT_MS || 2000),
    port: Number(process.env.ROBOT_CAMERA_DISCOVERY_PORT || 33333),
    broadcast: process.env.ROBOT_CAMERA_DISCOVERY_BROADCAST || '255.255.255.255'
};
let cameraDiscoveryCache = { url: '', at: 0, pending: null };

async function resolveCameraUrl(config) {
    const configured = (config.camera?.stream_url || '').trim();
    if (configured) {
        return configured;
    }

    const now = Date.now();
    if (now - cameraDiscoveryCache.at < 15000) {
        return cameraDiscoveryCache.url;
    }
    if (cameraDiscoveryCache.pending) {
        return cameraDiscoveryCache.pending;
    }

    cameraDiscoveryCache.pending = discoverCamera(cameraDiscoveryOptions)
        .then((device) => {
            cameraDiscoveryCache.url = device?.stream_url || '';
            cameraDiscoveryCache.at = Date.now();
            if (device) {
                console.log(`[camera] discovered ${device.name || '<unnamed>'} at ${device.stream_url}`);
            } else {
                console.log('[camera] discovery found no device');
            }
            return cameraDiscoveryCache.url;
        })
        .catch((err) => {
            console.error('[camera] discovery failed:', err.message);
            cameraDiscoveryCache.url = '';
            cameraDiscoveryCache.at = Date.now();
            return '';
        })
        .finally(() => {
            cameraDiscoveryCache.pending = null;
        });

    return cameraDiscoveryCache.pending;
}

// 运行时配置（前端可读取的可配置项）
// 浏览器侧事件落盘：页面把关键状态变化（连接、锁定、遥控、错误）上报，
// 与后端事件、网络采样一起构成完整的离线分析素材。
app.post('/api/client_log', (req, res) => {
    const logDir = process.env.ROBOT_CONSOLE_LOG_DIR;
    if (!logDir) return res.json({ success: true, stored: false });
    const rows = Array.isArray(req.body) ? req.body.slice(0, 50) : [req.body];
    const file = path.join(logDir, 'frontend-events.jsonl');
    try {
        const stamp = Date.now();
        const lines = rows.filter(row => row && typeof row === 'object').slice(0, 50)
            .map(row => JSON.stringify({ at: stamp / 1000, ...row })).join('\n') + '\n';
        fs.appendFileSync(file, lines);
        // 单文件上限 5 MB，超出后截断保留最近内容，避免长期运行撑爆磁盘。
        if (fs.statSync(file).size > 5 * 1024 * 1024) {
            const tail = fs.readFileSync(file, 'utf8').split('\n').slice(-20000).join('\n');
            fs.writeFileSync(file, tail);
        }
        res.json({ success: true, stored: true });
    } catch (error) {
        res.status(500).json({ success: false, error: error.message });
    }
});

app.get('/api/runtime_config', async (req, res) => {
    const config = loadConfig();
    const camera = { ...(config.camera || {}) };
    camera.stream_url = process.env.ROBOT_SIMULATION === '1' ? '' : await resolveCameraUrl(config);
    res.json({
        topics: config.topics || {},
        camera,
        simulation: process.env.ROBOT_SIMULATION === '1'
    });
});

// 手动触发一次摄像头发现（前端“自动发现”按钮）。
const vision = installVisionRoutes({ app, repoRoot });

app.post('/api/camera/discover', async (req, res) => {
    if (process.env.ROBOT_SIMULATION === '1') return res.json({ success: false, stream_url: '', error: '仿真环境不发现实车摄像头' });
    cameraDiscoveryCache = { url: '', at: 0, pending: null };
    const url = await resolveCameraUrl(loadConfig());
    res.json({ success: Boolean(url), stream_url: url });
});

const navigation = installNavigationRoutes({
    app, root: __dirname, loadConfig, buildRosCommand, activeProcesses,
    mappingBusy: () => mappingStarting || mappingStopping || taskRunning(loadConfig(), 'mapping')
});
const calibration = installCalibrationRoutes({ app, repoRoot, buildRosCommand,
    simulation: process.env.ROBOT_SIMULATION === '1', apiPort: port });

// ---- 建图停止/中止的公共设施（P0-B）----
// 有界超时包装：超时返回 null 且不抛错，避免停止路径被单个慢操作拖死。
const withTimeout = (promise, ms) => Promise.race([
    promise.catch(() => null),
    new Promise(resolve => { const timer = setTimeout(() => resolve(null), ms); timer.unref?.(); })
]);

// best-effort 零速兜底：优先用已启动的控制桥（节点热、时延低），失败再退化为一次性 CLI。
// 只作为停止路径的安全兜底，不能当作"已验证停车"的证据。
function publishZeroVelocity(sessionId) {
    const attempts = [];
    if (typeof sessionId === 'string' && sessionId.trim()) {
        attempts.push(navigation.velocity({ linear: 0, angular: 0, sessionId: sessionId.trim() })
            .then(() => 'bridge', () => Promise.reject(new Error('bridge 零速被拒绝'))));
    }
    attempts.push(new Promise((resolve, reject) => {
        const proc = spawn('bash', ['-c', buildRosCommand('ros2', ['topic', 'pub', '--once', '-w', '1',
            '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'])],
            { detached: true, stdio: 'ignore' });
        const timer = setTimeout(() => {
            try { process.kill(-proc.pid, 'SIGKILL'); } catch (error) { if (error.code !== 'ESRCH') console.error(error.message); }
            reject(new Error('零速命令超时'));
        }, 2500);
        timer.unref?.();
        proc.once('exit', code => {
            clearTimeout(timer);
            if (code === 0) resolve('cli');
            else reject(new Error(`零速命令退出码 ${code}`));
        });
        proc.once('error', error => { clearTimeout(timer); reject(error); });
    }));
    return Promise.any(attempts)
        .then(channel => { console.log(`[mapping] 已补发零速（${channel}）`); return channel; })
        .catch(reason => {
            console.error('[mapping] 零速补发失败:', (reason?.errors || []).map(item => item.message).join('/') || reason?.message);
            return null;
        });
}

// 终止目标任务的进程树：单个任务失败不阻断其它任务，失败项如实上报（不做无限重试）。
async function terminateTaskProcesses(tasks) {
    const result = { stopped: 0, failed: [] };
    for (const task of tasks) {
        const proc = activeProcesses[task.id];
        if (!proc) continue;
        try {
            await terminateProcess(proc);
        } catch (error) {
            result.failed.push({ id: task.id, error: error.message });
            continue; // 保留 activeProcesses 记录：终止失败不得伪装 idle
        }
        if (activeProcesses[task.id] === proc) activeProcesses[task.id] = null;
        result.stopped += 1;
    }
    return result;
}

// 幂等收敛建图状态：代次不匹配（上一轮迟到回调）时不做任何事。
function finalizeMapping(generation) {
    if (generation !== mappingGeneration) return false;
    mappingStartedAt = 0;
    mappingMode = 'manual';
    return true;
}

// 当前控制权是否已由本会话持有（用于"只回滚本次新增租约"）。
async function controlOwnerIs(sessionId) {
    if (typeof sessionId !== 'string' || !sessionId.trim()) return false;
    try {
        const status = await navigation.status();
        return status.controlOwner === sessionId.trim();
    } catch { return false; }
}

// 预检/保存闸门失败时归还控制权：只回滚本请求新取得的租约。
async function releaseNewClaim(sessionId, ownedBefore) {
    if (ownedBefore) return;
    try { await navigation.release(sessionId); }
    catch (error) { console.error('[control] 释放控制权失败:', error.message); }
}

// 停止建图但不保存（P0-B）：跳过 saveMap，先终止进程组，再清状态。
// 保存失败/质量不合格时这条路径必须可用，重复调用幂等。
async function abortMapping(req, res) {
    const config = loadConfig();
    if (mappingStarting) return res.status(409).json({ success: false, error: '建图正在启动，请稍后重试' });
    if (abortInFlight) return res.status(409).json({ success: false, error: '建图正在停止，请等待完成' });
    if (!taskRunning(config, 'mapping')) {
        finalizeMapping(mappingGeneration);
        return res.json({ success: true, aborted: true, alreadyStopped: true, savedMap: null,
            message: '建图未在运行（未保存）' });
    }
    const sessionId = req.body?.sessionId;
    const generation = mappingGeneration;
    const startedAt = Date.now();
    mappingStopping = true;
    // 立即发一次零速（不阻塞），保证"停止→/cmd_vel=0"在 1s 内可见。
    const immediateZero = publishZeroVelocity(sessionId);
    abortInFlight = terminateTaskProcesses((config.tasks.mapping || []));
    try {
        const result = await abortInFlight;
        if (result.failed.length) {
            // 终止失败：保留运行/故障状态，绝不报 idle 或成功。
            console.error('[mapping] abort 有进程未退出:', result.failed);
            return res.status(500).json({ success: false, aborted: false, stopped: false, savedMap: null,
                error: `停止建图失败：${result.failed.map(item => `${item.id}: ${item.error}`).join('；')}`,
                elapsedMs: Date.now() - startedAt });
        }
        finalizeMapping(generation);
        // 最后一个非零速度生产者已退出后再补一次零速；带超时，不阻塞响应。
        const finalZero = await withTimeout(publishZeroVelocity(sessionId), 1200);
        const immediate = finalZero ? null : await withTimeout(immediateZero, 200);
        await mappingRun.finish('aborted', '停止建图但不保存');
        return res.json({ success: true, aborted: true, savedMap: null, stoppedProcesses: result.stopped,
            zeroVelocity: finalZero || immediate || null, elapsedMs: Date.now() - startedAt,
            message: '建图已中止（未保存）' });
    } catch (error) {
        console.error('[mapping] abort 失败:', error);
        return res.status(500).json({ success: false, aborted: false, savedMap: null,
            error: `停止建图失败：${error.message}` });
    } finally {
        abortInFlight = null;
        mappingStopping = false;
    }
}

// 通用启动任务接口
app.post('/api/run_task/:taskName', async (req, res) => {
    const taskName = req.params.taskName;
    console.log(`收到启动任务请求: ${taskName}`);

    const config = loadConfig();
    const tasks = config.tasks[taskName] || [];
    // 建图模式：缺省/旧调用一律 manual；只有显式 mode=auto 才追加 --auto。
    const requestedMode = taskName === 'mapping' && req.body?.mode === 'auto' ? 'auto' : 'manual';
    if (taskName === 'mapping' && (mappingStarting || mappingStopping || navigation.busy() || taskRunning(config, 'navigation'))) {
        return res.status(409).json({ success: false, error: '请先停止导航；保存地图期间也不能重新开始建图' });
    }

    if (tasks.length === 0) {
        return res.status(404).json({ success: false, error: `任务类别 ${taskName} 未定义或为空` });
    }

    let claimedHere = false;
    let ownedBefore = false;
    if (taskName === 'mapping' && !taskRunning(config, 'mapping')) {
        // 预检只读，排在取控制权之前：预检失败不再留下无人释放的 claim（P0-B）。
        mappingStarting = true;
        try {
            await checkMappingInput(requestedMode === 'auto' ? '自动建图' : '建图', requestedMode === 'auto');
        } catch (error) {
            return res.status(409).json({ success: false, error: error.message });
        } finally { mappingStarting = false; }
        // 建图属于变更类任务：仅控制页面可启动（无所有者时该操作即取得控制权）。
        ownedBefore = await controlOwnerIs(req.body?.sessionId);
        try {
            await navigation.claim(req.body?.sessionId);
            claimedHere = true;
        } catch (error) {
            return res.status(409).json({ success: false, error: error.message });
        }
    }
    let generation = mappingGeneration;
    if (taskName === 'mapping') {
        mappingGeneration += 1;
        generation = mappingGeneration;
    }
    try {
        tasks.forEach((task) => {
            const { id, command, args, description } = task;
            const taskArgs = Array.isArray(args) ? [...args] : [];
            if (taskName === 'mapping' && requestedMode === 'auto') taskArgs.push('--auto');

            if (!activeProcesses[id]) {
                console.log(`[${id}] 启动: ${description} (${command} ${taskArgs.join(' ')})`);
                const fullCommand = buildRosCommand(command, taskArgs);

                const proc = spawn('bash', ['-c', fullCommand], {
                    detached: true,
                    stdio: 'inherit'
                });
                proc.__mappingGeneration = generation;

                proc.on('error', (err) => {
                    console.error(`[${id}] 启动错误:`, err.message);
                    if (activeProcesses[id] === proc) {
                        activeProcesses[id] = null;
                        if (taskName === 'mapping') finalizeMapping(proc.__mappingGeneration);
                    }
                });

                proc.on('exit', (code) => {
                    console.log(`[${id}] 已退出 (代码: ${code})`);
                    if (activeProcesses[id] === proc) {
                        activeProcesses[id] = null;
                        if (taskName === 'mapping') {
                            finalizeMapping(proc.__mappingGeneration);
                            mappingRun.finish('aborted', `建图进程自行退出（代码 ${code}）`)
                                .catch(() => {});
                        }
                    }
                });

                activeProcesses[id] = proc;
                if (taskName === 'mapping') {
                    mappingMode = requestedMode;
                    mappingStartedAt = Date.now();
                }
            }
        });

        if (taskName === 'mapping') {
            // 记录这次建图的生效参数与起止；不阻塞响应，失败也只记日志。
            mappingRun.start('mapping', {
                mode: requestedMode,
                mapsDir: loadConfig().maps_dir,
                sessionId: String(req.body?.sessionId || '').slice(0, 128)
            }).catch(error => console.error('[mapping] 运行清单创建失败:', error.message));
        }
        res.json({ success: true, message: `任务 [${taskName}] 已按配置启动` });
    } catch (error) {
        console.error(`[${taskName}] 启动失败:`, error);
        if (taskName === 'mapping') {
            finalizeMapping(generation);
            if (claimedHere) await releaseNewClaim(req.body?.sessionId, ownedBefore);
        }
        res.status(500).json({ success: false, error: error.message });
    }
});

// 通用停止任务接口。"保存并停止"是默认行为；?save=false 走不保存的中止路径（P0-B）。
app.post('/api/stop_task/:taskName', async (req, res) => {
    const taskName = req.params.taskName;
    const saveRequested = !(req.query.save === 'false' || req.query.save === '0' || req.body?.save === false);
    console.log(`收到停止任务请求: ${taskName}${saveRequested ? '' : '（不保存）'}`);
    if (taskName === 'mapping' && !saveRequested) return abortMapping(req, res);

    const config = loadConfig();
    const tasks = config.tasks[taskName] || [];

    if (taskName === 'mapping' && abortInFlight) {
        return res.status(409).json({ success: false, error: '建图正在停止，请等待完成' });
    }
    if (taskName === 'mapping' && (mappingStarting || mappingStopping)) {
        return res.status(409).json({ success: false, error: '正在保存并停止建图，请等待完成' });
    }
    if (taskName === 'mapping') mappingStopping = true;
    try {
        let savedMap = null;
        let saveError = null;
        if (taskName === 'mapping' && taskRunning(config, 'mapping')) {
            // 保存是附带动作，停止是硬要求：保存失败只记录，不再阻断 terminateProcess
            // （此前 saveMap 抛错会跳过 terminateProcess，导致建图进程残留）。
            try {
                assertMappingDuration(mappingStartedAt, config.save_map?.min_mapping_ms);
                savedMap = await saveMap(config, automaticMapName(), buildRosCommand, { mappingStartedAt });
                rememberMap(savedMap);
            } catch (error) {
                saveError = error;
                console.error('[mapping] 保存失败，继续停止建图:', error.message);
            }
        }
        const generation = mappingGeneration;
        const result = await terminateTaskProcesses(tasks);
        if (result.failed.length) {
            console.error(`[${taskName}] 部分进程未退出:`, result.failed);
            return res.status(500).json({ success: false, savedMap: null, stopped: false,
                error: `停止失败：${result.failed.map(item => `${item.id}: ${item.error}`).join('；')}` });
        }
        if (taskName === 'mapping') {
            finalizeMapping(generation);
            // 保存成功=saved，保存失败=失败但已停止，未保存=用户主动停止。
            const status = saveError ? 'failed' : (savedMap ? 'success' : 'aborted');
            await mappingRun.finish(status, saveError ? `保存失败：${saveError.message}` : '保存并停止');
        }
        if (saveError) {
            // 停止已完成但保存失败：明确区分，前端不得据此认为建图仍在运行。
            return res.status(saveError.statusCode || 500).json({ success: false, savedMap: null,
                stopped: true, saveFailed: true,
                error: `建图已停止，但保存失败：${saveError.message}` });
        }
        res.json({ success: true, savedMap, stopped: true,
            message: savedMap ? `地图已保存：${savedMap.name}，建图已停止` : `任务 [${taskName}] 已停止` });
    } catch (error) {
        console.error(`[${taskName}] 停止失败:`, error);
        res.status(error.statusCode || 500).json({ success: false, error: error.message });
    } finally {
        if (taskName === 'mapping') mappingStopping = false;
    }
});

// 状态查询接口 (针对特定任务类别)
app.get('/api/task_status/:taskName', (req, res) => {
    const taskName = req.params.taskName;
    const config = loadConfig();
    const tasks = config.tasks[taskName] || [];

    const isRunning = tasks.some(task => !!activeProcesses[task.id]);
    res.json({ isRunning, ...(taskName === 'mapping' ? { saving: mappingStopping, mode: mappingMode,
        runId: mappingRun.runId, lastSavedMap: restoredMap() } : {}) });
});

// 兼容旧的接口 (映射到 mapping 任务)
app.post('/api/start_mapping', (req, res) => res.redirect(307, '/api/run_task/mapping'));
app.post('/api/stop_mapping', (req, res) => res.redirect(307, '/api/stop_task/mapping'));
app.get('/api/mapping_status', (req, res) => res.redirect(301, '/api/task_status/mapping'));

// 停止建图但不保存（P0-B）：跳过 saveMap，先终止进程组再清状态。
app.post('/api/abort_mapping', abortMapping);
app.post('/api/mapping/abort', abortMapping);

// 地图质量评估缓存：按 yaml/pgm 的 mtime+size 与阈值快照做键，轮询时避免重复解析 PGM。
const mapQualityCache = new Map();
function mapQualityOf(prefix, gate) {
    const signature = (() => {
        try {
            const yaml = fs.statSync(prefix + '.yaml');
            const pgm = fs.statSync(prefix + '.pgm');
            return `${yaml.mtimeMs}:${yaml.size}:${pgm.mtimeMs}:${pgm.size}:${gate.minOccupiedCells}:${gate.maxUnknownRatio}:${gate.expectedResolution}`;
        } catch { return null; }
    })();
    if (signature) {
        const hit = mapQualityCache.get(prefix);
        if (hit && hit.signature === signature) return hit.result;
    }
    const result = inspectMapQuality(prefix, gate);
    if (signature) {
        mapQualityCache.set(prefix, { signature, result });
        if (mapQualityCache.size > 200) mapQualityCache.delete(mapQualityCache.keys().next().value);
    }
    return result;
}

// 获取地图列表：每张图附带质量结论（navigable）与原因，前端据此禁用差图（P0-A）。
// 阈值与实际评估使用同一份快照，避免"报的门槛"和"判的门槛"不一致。
app.get('/api/list_maps', (req, res) => {
    const config = loadConfig();
    const mapsDir = config.maps_dir || config.save_map?.save_dir || '/tmp';
    const gate = mapQualityGate();
    const thresholds = { minOccupiedCells: gate.minOccupiedCells, maxUnknownRatio: gate.maxUnknownRatio,
        expectedResolution: gate.expectedResolution };
    try {
        if (!fs.existsSync(mapsDir)) {
            return res.json({ maps: [], thresholds });
        }
        const files = fs.readdirSync(mapsDir)
            .filter(f => f.endsWith('.yaml'))
            .map(f => {
                const prefix = path.join(mapsDir, f.replace('.yaml', ''));
                const evaluated = mapQualityOf(prefix, gate);
                return {
                    name: f.replace('.yaml', ''),
                    modifiedAt: fs.statSync(path.join(mapsDir, f)).mtimeMs,
                    path: prefix,
                    // ok=可读，navigable=通过质量门禁；准入只看 navigable。
                    readable: evaluated.ok,
                    navigable: evaluated.navigable,
                    quality: evaluated.quality,
                    qualityError: evaluated.error
                };
            })
            .sort((a, b) => b.modifiedAt - a.modifiedAt || a.name.localeCompare(b.name));
        res.json({ maps: files, thresholds });
    } catch (err) {
        console.error('[list_maps] 读取地图目录失败:', err);
        res.status(500).json({ maps: [], thresholds, error: err.message });
    }
});

// 保存地图接口：保存闸门（时长/质量）失败时必须归还本次新增的控制权（P0-B）。
app.post('/api/save_map', async (req, res) => {
    const sessionId = req.body?.sessionId;
    let claimedHere = false;
    let ownedBefore = false;
    try {
        if (!taskRunning(loadConfig(), 'mapping') || mappingStopping) throw new Error('请在实时建图运行时保存地图');
        ownedBefore = await controlOwnerIs(sessionId);
        await navigation.claim(sessionId);
        claimedHere = true;
        assertMappingDuration(mappingStartedAt, loadConfig().save_map?.min_mapping_ms);
        const savedMap = await saveMap(loadConfig(), req.body?.mapName, buildRosCommand,
            { mappingStartedAt });
        rememberMap(savedMap);
        res.json({ success: true, ...savedMap, message: `地图已保存：${savedMap.name}` });
    } catch (error) {
        if (claimedHere) await releaseNewClaim(sessionId, ownedBefore);
        res.status(error.statusCode || 500).json({ success: false, error: error.message });
    }
});

// ---- 诊断包导出（P2-3）----
// 只包含：版本标识、经过筛选的运行参数、节点/进程状态、任务事件和有限长度日志。
// 明确排除：密钥、环境变量全集、无关用户文件。
const DIAGNOSTIC_ENV_WHITELIST = ['ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
    'ROS_AUTOMATIC_DISCOVERY_RANGE', 'FASTDDS_BUILTIN_TRANSPORTS', 'ROBOT_SIMULATION',
    'ROBOT_API_PORT', 'ROBOT_MAPS_DIR', 'ROBOT_CONSOLE_LOG_DIR', 'ROBOT_WORKSPACE',
    'VITE_ROBOT_API_PORT', 'VITE_ROBOT_WS_PORT'];

function gitInfo() {
    return new Promise(resolve => {
        execFile('git', ['rev-parse', 'HEAD'], { cwd: repoRoot, timeout: 5000 }, (error, stdout) => {
            if (error) return resolve({ available: false });
            execFile('git', ['rev-parse', '--abbrev-ref', 'HEAD'], { cwd: repoRoot, timeout: 5000 }, (e2, branch) => {
                execFile('git', ['status', '--porcelain', '--', 'web_console', 'scripts'],
                    { cwd: repoRoot, timeout: 8000, maxBuffer: 1024 * 1024 }, (e3, dirty) => {
                        resolve({ available: true, commit: stdout.trim(), branch: branch.trim(),
                            dirtyPaths: e3 ? null : dirty.trim().split('\n').filter(Boolean).slice(0, 50) });
                    });
            });
        });
    });
}

function tailFile(file, bytes) {
    try {
        const fd = fs.openSync(file, 'r');
        try {
            const size = fs.fstatSync(fd).size;
            const start = Math.max(0, size - bytes);
            const buffer = Buffer.alloc(Math.min(size, bytes));
            fs.readSync(fd, buffer, 0, buffer.length, start);
            return { file: path.basename(file), bytes: size, truncated: start > 0, tail: buffer.toString('utf8') };
        } finally { fs.closeSync(fd); }
    } catch (error) {
        return { file: path.basename(file), error: error.message };
    }
}

app.get('/api/diagnostics/bundle', async (_req, res) => {
    const config = loadConfig();
    const bundle = {
        schema: 'leap-diagnostics/1',
        generatedAt: new Date().toISOString(),
        versions: {
            git: await gitInfo(),
            console: (() => { try { const p = JSON.parse(fs.readFileSync(path.join(__dirname, 'package.json'), 'utf8')); return { name: p.name, version: p.version }; } catch { return null; } })(),
            node: process.version,
            rosDistro,
            platform: `${os.type()} ${os.release()}`,
            frontendBuild: (() => { try { return fs.statSync(path.join(__dirname, 'dist', 'index.html')).mtime.toISOString(); } catch { return null; } })()
        },
        runtimeParameters: {
            apiPort: port,
            mapsDir: config.maps_dir,
            workspaceSetup: resolveWorkspaceSetup(),
            simulation: process.env.ROBOT_SIMULATION === '1',
            cameraConfigured: Boolean((config.camera?.stream_url || '').trim()),
            // 白名单环境变量；绝不导出完整 process.env。
            env: Object.fromEntries(DIAGNOSTIC_ENV_WHITELIST.filter(key => process.env[key] !== undefined)
                .map(key => [key, process.env[key]]))
        },
        configuredTasks: Object.fromEntries(Object.entries(config.tasks || {}).map(([name, list]) =>
            [name, (list || []).map(task => ({ id: task.id, description: task.description, command: task.command, args: task.args }))])),
        navigation: null,
        calibration: calibration.status(),
        mapping: {
            isRunning: taskRunning(config, 'mapping'),
            saving: mappingStopping,
            lastSavedMap: (() => { const m = restoredMap(); return m ? { name: m.name, savedAt: m.savedAt } : null; })()
        },
        maps: (() => {
            try {
                return fs.readdirSync(config.maps_dir).filter(f => f.endsWith('.yaml'))
                    .map(f => ({ name: f.replace(/\.yaml$/, ''), modifiedAt: fs.statSync(path.join(config.maps_dir, f)).mtimeMs }));
            } catch { return []; }
        })(),
        backendLog: consoleRing.slice(-200),
        consoleLogs: []
    };
    try { bundle.navigation = await navigation.diagnostics(); }
    catch (error) { bundle.navigation = { error: error.message }; }
    const logDir = process.env.ROBOT_CONSOLE_LOG_DIR;
    if (logDir && fs.existsSync(logDir)) {
        let budget = 120000;
        for (const name of fs.readdirSync(logDir).filter(f => f.endsWith('.log')).sort().slice(0, 8)) {
            if (budget <= 0) break;
            const entry = tailFile(path.join(logDir, name), Math.min(20000, budget));
            budget -= (entry.tail || '').length;
            bundle.consoleLogs.push(entry);
        }
    }
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    res.setHeader('Content-Disposition', `attachment; filename="leap-diagnostics-${stamp}.json"`);
    res.type('application/json').send(JSON.stringify(bundle, null, 2));
});

// 停止所有活跃进程的函数
async function stopAllProcesses() {
    await Promise.all(Object.values(activeProcesses).filter(Boolean).map(terminateProcess));
}

let shuttingDown = false;
async function shutdown() {
    if (shuttingDown) return;
    shuttingDown = true;
    try {
        await calibration.close();
        await vision.close();
        await navigation.close();
        await stopAllProcesses();
        process.exit(0);
    } catch (error) {
        console.error('清理 ROS 进程失败:', error.message);
        process.exit(1);
    }
}
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

app.listen(port, () => {
    console.log(`ROS 2 运行服务后端已启动并监听在 http://localhost:${port}`);
});
