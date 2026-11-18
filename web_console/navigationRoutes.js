import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawn } from 'node:child_process';
import { NavigationBridge } from './navigationBridge.js';
import { terminateProcess } from './managedProcess.js';
import { inspectMapPair, parseMapYaml, qualityOptions } from './mapStorage.js';

export function installNavigationRoutes({ app, root, loadConfig, buildRosCommand, activeProcesses, mappingBusy }) {
    const bridge = new NavigationBridge(buildRosCommand('python3', [path.join(root, '../scripts/navigation_bridge.py')]));
    let starting = false;
    let stopping = false;
    let loadedMap = '';
    let processError = '';
    let configuration = null;
    const fileIdentity = filename => {
        try {
            const stat = fs.statSync(filename);
            if (!stat.isFile() || stat.size > 2*1024*1024) return { path: filename, sha256: null, error: '参数文件无效或超过 2 MiB' };
            return { path: fs.realpathSync(filename), sha256: crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex') };
        }
        catch { return { path: filename, sha256: null }; }
    };
    const tasks = () => loadConfig().tasks.navigation || [];
    const running = () => tasks().some(task => !!activeProcesses[task.id]);
    const wrap = handler => async (req, res) => {
        try { await handler(req, res); }
        catch (error) { res.status(409).json({ success: false, error: error.message }); }
    };

    // ---- 请求标识幂等（P1-2 乱序/重复请求）----
    // 携带 requestId 的变更类请求在 30 秒窗口内重放同一结果，而不是重复执行；
    // 并发重复请求共享同一个执行中的 Promise。心跳与速度不走此表
    // （心跳无副作用，速度由递增 sequence 去重）。
    const idempotency = new Map();
    async function dedupe(body, endpoint, factory) {
        const requestId = body?.requestId;
        if (typeof requestId !== 'string' || !requestId.trim() || requestId.length > 128) return factory();
        const key = `${endpoint}:${requestId.trim()}`;
        const now = Date.now();
        for (const [old, entry] of idempotency) if (now - entry.at > 30000) idempotency.delete(old);
        const hit = idempotency.get(key);
        if (hit) {
            if (hit.settled) {
                if (hit.error) throw hit.error;
                return hit.value;
            }
            return hit.promise;
        }
        const entry = { at: now, settled: false };
        entry.promise = (async () => factory())();
        entry.promise.catch(() => {});
        idempotency.set(key, entry);
        if (idempotency.size > 500) idempotency.delete(idempotency.keys().next().value);
        try {
            entry.value = await entry.promise;
            entry.settled = true;
            return entry.value;
        } catch (error) {
            entry.error = error;
            entry.settled = true;
            throw error;
        }
    }

    app.get('/api/navigation_status', wrap(async (_req, res) => {
        const status = await bridge.request('status');
        const lostOwner = running() && !status.isRunning;
        res.json({ ...status, configuration, isRunning: starting || running(), mapPath: loadedMap || status.mapPath,
            ...(processError ? { phase: 'failed', ready: false, error: processError } : {}),
            ...(lostOwner ? { phase: 'failed', ready: false, error: 'ROS 控制服务已重启，请停止并重新启动导航' } : {}) });
    }));
    app.get('/api/task_status/navigation', (_req, res) => res.redirect(307, '/api/navigation_status'));
    app.post('/api/run_task/navigation', (_req, res) => res.status(400).json({ success: false, error: '请从导航面板选择地图启动' }));

    // 任务事件（P2-2）：增量拉取，seq 单调递增，错误事件不会被轮询擦除。
    app.get('/api/navigation/events', wrap(async (req, res) => {
        const since = Number(req.query.since);
        res.json(await bridge.request('events', { since: Number.isFinite(since) && since > 0 ? Math.floor(since) : 0 }));
    }));

    // 保存级质量门禁：空白/损坏/低占用地图不能启动导航。
    // 门槛与 save_map / /api/list_maps 共用 qualityOptions，禁止这里再走 300/95 默认值。
    // Node 自带测试进程使用 1x1 假地图；仅在显式 ROBOT_MAP_QUALITY=strict 或生产进程执行。
    const mapQualityEnabled = () => process.env.ROBOT_MAP_QUALITY === 'strict' ||
        !process.env.NODE_TEST_CONTEXT;
    const navigationMapGate = () => {
        try { return qualityOptions(loadConfig().save_map); }
        catch (error) {
            // 配置写错时回落严格默认，绝不放宽门禁。
            console.error('[navigation] 地图质量配置非法，回落 MAP_QUALITY:', error.message);
            return qualityOptions({});
        }
    };
    const validateNavigationMap = mapPath => {
        const gate = navigationMapGate();
        try {
            return inspectMapPair(mapPath, gate);
        } catch (error) {
            let origin = '<无法读取>';
            try {
                origin = JSON.stringify(parseMapYaml(fs.readFileSync(mapPath + '.yaml', 'utf8')).origin);
            } catch { /* 结构损坏时保留可读错误 */ }
            throw new Error(`地图质量校验失败：${error.message}；origin=${origin}；` +
                `门槛：占用≥${gate.minOccupiedCells} 且未知≤${(gate.maxUnknownRatio * 100).toFixed(1)}%`);
        }
    };

    const startNavigation = req => async () => {
        if (starting || stopping || mappingBusy()) throw new Error('正在切换任务，请先保存并停止建图');
        const config = loadConfig();
        const mapPath = typeof req.body.mapPath === 'string' ? path.resolve(req.body.mapPath) : '';
        if (!mapPath || path.dirname(mapPath) !== fs.realpathSync(config.maps_dir) || !fs.existsSync(mapPath + '.yaml')) {
            throw new Error('请选择地图目录中的有效地图');
        }
        if (mapQualityEnabled()) validateNavigationMap(mapPath);
        if (running()) {
            if (!(await bridge.request('status')).isRunning) throw new Error('ROS 控制服务已重启，请先停止导航');
            if (loadedMap !== mapPath) throw new Error('请先停止当前导航再切换地图');
            return { success: true, message: '导航已启动，请查看就绪状态' };
        }
        if (!tasks().length) throw new Error('未配置 navigation 任务');
        starting = true;
        try {
            const health = await bridge.request('status');
            // status while stopped reports only the live sensor requirements.
            if (health.missing.length) throw new Error('不能开始导航：缺少' + health.missing.join('、'));
            // begin 校验控制会话：无所有者时视为明确启用并授予；他人持有则拒绝。
            await bridge.request('begin', { mapPath, sessionId: req.body?.sessionId, requestId: req.body?.requestId });
            loadedMap = mapPath;
            configuration = { capturedAt: new Date().toISOString(), mapYaml: fileIdentity(mapPath+'.yaml'),
                parameterFiles: tasks().map(task => {
                    const explicit = [...(task.args || [])].reverse().find(arg => arg.startsWith('params_file:='));
                    return fileIdentity(explicit ? path.resolve(explicit.slice('params_file:='.length))
                        : path.join(root, '../robot_ws/install/ldc_navigation/share/ldc_navigation/param/ldc_nav2.yaml'));
                }) };
            processError = '';
            for (const task of tasks()) {
                const proc = spawn('bash', ['-c', buildRosCommand(task.command, [...(task.args || []), `map:=${mapPath}.yaml`, 'cmd_vel_output:=cmd_vel_auto', 'bond_timeout:=10.0', 'managed_by_console:=true'])],
                    { detached: true, stdio: 'inherit' });
                activeProcesses[task.id] = proc;
                const exited = () => {
                    if (activeProcesses[task.id] !== proc) return;
                    activeProcesses[task.id] = null;
                    if (!stopping) processError = '导航节点已退出，请检查日志并重新启动';
                    bridge.request('stop').catch(error => console.error(error.message));
                };
                proc.on('exit', exited);
                proc.on('error', exited);
            }
            return { success: true, message: '导航正在启动，等待节点激活及初始定位' };
        } finally { starting = false; }
    };
    app.post('/api/start_navigation', wrap(async (req, res) => {
        res.json(await dedupe(req.body, 'start_navigation', startNavigation(req)));
    }));

    const stopNavigation = () => async () => {
        if (starting || stopping) throw new Error('任务正在切换，请稍后重试');
        stopping = true;
        try {
            // 停止属于清理类操作：任意页面可用，先取消并确认再结束进程。
            await bridge.request('stop');
            for (const task of tasks()) {
                const proc = activeProcesses[task.id];
                if (!proc) continue;
                await terminateProcess(proc);
                activeProcesses[task.id] = null;
            }
            loadedMap = '';
            processError = '';
            return { success: true };
        } finally { stopping = false; }
    };
    const stop = wrap(async (req, res) => {
        res.json(await dedupe(req.body, 'stop_navigation', stopNavigation()));
    });
    app.post('/api/stop_navigation', stop);
    app.post('/api/stop_task/navigation', stop);

    // 变更类运动操作：统一携带 sessionId 与 requestId；重复 requestId 幂等重放。
    const dedupedOperations = { goals: 'goals', cancel: 'cancel', initial_pose: 'initial_pose', estop: 'estop' };
    for (const [endpoint, op] of Object.entries(dedupedOperations)) {
        app.post(`/api/navigation/${endpoint}`, wrap(async (req, res) => {
            res.json(await dedupe(req.body, endpoint, () => bridge.request(op, req.body)));
        }));
    }
    // 高频操作不走幂等表：心跳仅续期（归属校验在桥接层），速度由 sequence 去重。
    const plainOperations = { heartbeat: 'heartbeat', velocity: 'velocity' };
    for (const [endpoint, op] of Object.entries(plainOperations)) {
        app.post(`/api/navigation/${endpoint}`, wrap(async (req, res) => res.json(await bridge.request(op, req.body))));
    }

    // 控制权归属（P1-3）：claim 只在空闲或本页面持有时成功；
    // takeover 先取消并确认停止再生成新租约；release 由持有者释放。
    // reset_estop 是人工急停的显式复位，不能由 goals 隐式完成（P1 安全门控）。
    const controlOperations = { claim: 'claim', takeover: 'takeover', release: 'release', reset_estop: 'reset_estop' };
    for (const [endpoint, op] of Object.entries(controlOperations)) {
        app.post(`/api/control/${endpoint}`, wrap(async (req, res) => {
            res.json(await dedupe(req.body, endpoint, () => bridge.request(op, req.body)));
        }));
    }

    app.post('/api/control/manual_enable', wrap(async (req, res) => {
        res.json(await dedupe(req.body, 'manual_enable', async () => {
            // 先取消导航并等待终态确认，再生成新的速度会话（桥接层校验会话归属）。
            await bridge.request('cancel', { sessionId: req.body?.sessionId, requestId: req.body?.requestId });
            return bridge.request('manual_enable', req.body);
        }));
    }));

    return {
        close: async () => {
            if (bridge.process) {
                try { await bridge.request('estop'); } catch (error) { console.error(error.message); }
                const child = bridge.process;
                bridge.close();
                await terminateProcess(child);
            }
        },
        busy: () => starting || stopping,
        // 建图/保存等变更类任务在执行前取得控制权（无所有者时授予，他人持有则拒绝）。
        claim: async (sessionId) => {
            if (typeof sessionId !== 'string' || !sessionId.trim() || sessionId.length > 128) {
                throw new Error('缺少控制会话标识，请刷新页面后重试');
            }
            return bridge.request('claim', { sessionId: sessionId.trim() });
        },
        // 归还控制权：只有持有者可以释放；调用方负责保证"只回滚本次新增的租约"。
        release: async (sessionId) => {
            if (typeof sessionId !== 'string' || !sessionId.trim() || sessionId.length > 128) {
                throw new Error('缺少控制会话标识，请刷新页面后重试');
            }
            return bridge.request('release', { sessionId: sessionId.trim() });
        },
        // 只读状态：用于判断"当前控制权是否已由本会话持有"。
        status: () => bridge.request('status'),
        // 直接速度指令（停止路径的零速兜底）；会话归属校验在桥接层完成。
        velocity: (payload = {}) => bridge.request('velocity', payload),
        diagnostics: async () => {
            const report = { bridgeAlive: !!bridge.process, bridgePid: bridge.process?.pid ?? null,
                loadedMap, configuration, processError, starting, stopping,
                taskProcesses: tasks().map(task => ({ id: task.id, pid: activeProcesses[task.id]?.pid ?? null })) };
            try { report.status = await bridge.request('status'); } catch (error) { report.statusError = error.message; }
            try { report.events = (await bridge.request('events', { since: 0 })).events; } catch (error) { report.eventsError = error.message; }
            return report;
        }
    };
}
