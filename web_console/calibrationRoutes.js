import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawn } from 'node:child_process';
import { terminateProcess } from './managedProcess.js';

const ID = /^[a-zA-Z0-9_-]{1,80}$/;
function checkedChild(root, id) {
    if (typeof id !== 'string' || !ID.test(id)) throw new Error('无效会话或采集标识');
    const target = path.join(root, id);
    if (fs.existsSync(target) && fs.lstatSync(target).isSymbolicLink()) throw new Error('不允许符号链接');
    return target;
}
function bytesIn(folder) {
    let total = 0;
    if (!fs.existsSync(folder)) return total;
    for (const entry of fs.readdirSync(folder, { withFileTypes: true })) {
        if (entry.isSymbolicLink()) throw new Error('数据目录中存在符号链接，请人工检查');
        const file = path.join(folder, entry.name);
        total += entry.isDirectory() ? bytesIn(file) : fs.statSync(file).size;
    }
    return total;
}

export function installCalibrationRoutes({ app, repoRoot, buildRosCommand, simulation = false, apiPort = 3000,
    dataRoot = process.env.LDC_CALIBRATION_DIR || path.join(os.homedir(), '.local/share/ldc', simulation ? 'simulation/calibration' : 'calibration') }) {
    fs.mkdirSync(dataRoot, { recursive: true, mode: 0o700 });
    if (fs.lstatSync(dataRoot).isSymbolicLink()) throw new Error('标定根目录不能是符号链接');
    const root = fs.realpathSync(dataRoot);
    const helper = path.join(repoRoot, 'scripts/calibration/workspace.py');
    const diagnostics = path.join(repoRoot, 'scripts/diagnostics/cli.py');
    const helpers = new Set();
    const ownedJobs = new Set();
    let job = null;
    let starting = false;
    let stopping = null;
    let closing = false;
    let stopSequence = 0;
    function execute(script, args, input = {}, timeout = 30000, ros = false) {
        if (closing) return Promise.reject(new Error('服务正在退出'));
        return new Promise((resolve, reject) => {
            const proc = ros ? spawn('bash', ['-c', buildRosCommand('/usr/bin/python3', [script, ...args])], { detached: true, stdio: ['pipe', 'pipe', 'pipe'] })
                : spawn('/usr/bin/python3', [script, ...args], { detached: true, stdio: ['pipe', 'pipe', 'pipe'] });
            helpers.add(proc);
            let stdout = '', stderr = '', failure = '';
            const timer = setTimeout(() => { failure = '操作超时，原始数据已保留'; terminateProcess(proc).catch(() => {}); }, timeout);
            proc.stdout.on('data', chunk => {
                stdout += chunk;
                if (stdout.length > 2*1024*1024) { failure = '结果过大'; terminateProcess(proc).catch(() => {}); }
            });
            proc.stderr.on('data', chunk => { stderr = (stderr+chunk).slice(-12000); });
            proc.on('error', error => { failure = error.message; });
            proc.on('close', code => {
                clearTimeout(timer); helpers.delete(proc);
                if (failure || code !== 0) return reject(new Error(failure || stderr.trim() || `操作退出码 ${code}`));
                try { resolve(JSON.parse(stdout)); } catch { reject(new Error('工具未返回有效报告')); }
            });
            proc.stdin.on('error', () => {});
            proc.stdin.end(JSON.stringify(input));
        });
    }
    const workspace = (action, body = {}, timeout) => execute(helper, ['--root', root, '--action', action], body, timeout);
    const wrap = handler => async (req, res) => {
        try { await handler(req, res); }
        catch (error) { res.status(409).json({ success: false, error: error.message }); }
    };
    app.use('/api/calibration', (req, res, next) => {
        if (req.method !== 'GET' && req.headers.origin) {
            const normalize = host => ['localhost', '127.0.0.1', '[::1]'].includes(host) ? 'loopback' : host;
            try {
                if (normalize(new URL(req.headers.origin).hostname) !== normalize(req.hostname)) throw new Error();
            } catch { return res.status(403).json({ success: false, error: '拒绝来自其他站点的标定操作' }); }
        }
        next();
    });
    const jobStatus = () => job ? { kind: job.kind, sessionId: job.sessionId, captureId: job.captureId,
        running: job.proc.exitCode === null && job.proc.signalCode === null, exitCode: job.proc.exitCode,
        error: job.error, stoppedByUser: !!job.stoppedByUser, output: job.output.slice(-1500) } : null;
    app.get('/api/calibration/sessions', wrap(async (_req, res) => res.json({ sessions: await workspace('list'), simulation, job: jobStatus() })));
    app.post('/api/calibration/sessions', wrap(async (req, res) => res.json(await workspace('create', { ...req.body, simulation }))));
    app.get('/api/calibration/sessions/:id', wrap(async (req, res) => {
        checkedChild(root, req.params.id);
        const data = await workspace('get', { sessionId: req.params.id });
        for (const capture of data.captures) {
            if (capture.state === 'recording' && !(jobStatus()?.running && job.captureId === capture.id)) capture.state = 'interrupted';
        }
        res.json({ ...data, job: jobStatus() });
    }));
    for (const action of ['dimensions', 'measurement', 'analyze', 'camera', 'observations']) {
        app.post(`/api/calibration/sessions/:id/${action}`, wrap(async (req, res) => {
            checkedChild(root, req.params.id);
            if (jobStatus()?.running && job.sessionId === req.params.id && ['analyze', 'camera'].includes(action)) throw new Error('请结束本次采集后生成报告');
            res.json(await workspace(action, { ...req.body, sessionId: req.params.id }, action === 'camera' ? 120000 : 30000));
        }));
    }
    function capturePath(sessionId, captureId) {
        const session = checkedChild(root, sessionId);
        const captures = checkedChild(session, 'captures');
        return checkedChild(captures, captureId);
    }
    async function startJob(kind, sessionId, prepare) {
        if (closing) throw new Error('服务正在退出');
        if (starting || stopping || ownedJobs.size) throw new Error('已有采集或回放正在准备、运行或停止');
        starting = true;
        const beforeStop = stopSequence;
        try {
            const { captureId, args } = await prepare();
            if (closing || beforeStop !== stopSequence) throw new Error(`${kind === 'record' ? '采集' : '回放'}启动已取消`);
            if (stopping || ownedJobs.size) throw new Error('已有采集或回放正在运行或停止');
            const proc = spawn('bash', ['-c', buildRosCommand('/usr/bin/python3', [diagnostics, ...args])],
                { detached: true, stdio: ['ignore', 'pipe', 'pipe'] });
            const entry = { kind, sessionId, captureId, proc, output: '', error: '', log: '' };
            ownedJobs.add(entry);
            job = entry;
            proc.stdout.on('data', chunk => { entry.output = (entry.output+chunk).slice(-12000); });
            proc.stderr.on('data', chunk => { entry.log = (entry.log+chunk).slice(-4000); });
            proc.on('error', error => { entry.error = error.message; });
            proc.on('close', code => {
                ownedJobs.delete(entry);
                if (code !== 0 && !entry.error && !entry.stoppedByUser) entry.error = entry.log || `工具退出码 ${code}`;
            });
            return jobStatus();
        } finally { starting = false; }
    }
    function stopJobs() {
        ++stopSequence; // Invalidate preparation before waiting for process cleanup.
        if (stopping) return stopping;
        stopping = Promise.all([...ownedJobs].map(async entry => {
            entry.stoppedByUser = true;
            await terminateProcess(entry.proc);
            ownedJobs.delete(entry);
        })).finally(() => { stopping = null; });
        return stopping;
    }
    app.post('/api/calibration/sessions/:id/captures', wrap(async (req, res) => {
        res.json(await startJob('record', req.params.id, async () => {
            await workspace('get', { sessionId: req.params.id });
            const seconds = Number(req.body?.seconds ?? 60), maxMb = Number(req.body?.maxMb ?? 128), cameraEvery = Number(req.body?.cameraEvery ?? 0);
            if (!(seconds > 0 && seconds <= 600 && Number.isInteger(maxMb) && maxMb >= 1 && maxMb <= 512 && cameraEvery >= 0 && cameraEvery <= 60)) throw new Error('采集参数超出范围');
            const quota = 2*1024**3;
            if (bytesIn(root)+maxMb*1024**2 > quota) throw new Error('标定资料将超过 2 GiB，请人工归档；不会自动删除旧数据');
            const disk = fs.statfsSync(root);
            if (disk.bavail*disk.bsize < maxMb*1024**2+256*1024**2) throw new Error('磁盘剩余空间不足');
            const captureId = crypto.randomUUID();
            const folder = capturePath(req.params.id, captureId);
            return { captureId, args: ['record', simulation ? '--sim' : '--real', '--output', folder,
                '--seconds', String(seconds), '--max-mb', String(maxMb), '--camera-every', String(cameraEvery), '--api-port', String(apiPort)] };
        }));
    }));
    app.post('/api/calibration/stop', wrap(async (_req, res) => {
        await stopJobs();
        res.json({ success: true, job: jobStatus() });
    }));
    for (const action of ['report', 'summarize', 'replay']) {
        app.post(`/api/calibration/sessions/:id/captures/:capture/${action}`, wrap(async (req, res) => {
            const folder = capturePath(req.params.id, req.params.capture);
            if (!fs.existsSync(path.join(folder, 'recording.sqlite3'))) throw new Error('采集文件不存在');
            if (action === 'replay') {
                res.json(await startJob('replay', req.params.id, () => ({ captureId: req.params.capture, args: ['replay', folder] })));
            } else {
                if (starting || stopping || ownedJobs.size) throw new Error('请先结束当前采集或回放');
                res.json(await execute(diagnostics, [action, folder], {}, 30000, true));
            }
        }));
    }
    app.get('/api/calibration/sessions/:id/board', wrap(async (req, res) => {
        const session = checkedChild(root, req.params.id);
        res.download(path.join(session, 'checkerboard_9x6_20mm_A4.svg'));
    }));
    return { status: () => ({ root, simulation, starting, stopping: !!stopping, job: jobStatus() }), close: async () => {
        closing = true;
        await Promise.all([stopJobs(), ...[...helpers].map(terminateProcess)]);
    } };
}
