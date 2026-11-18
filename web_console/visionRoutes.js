import { spawn } from 'node:child_process';
import http from 'node:http';
import path from 'node:path';
import { terminateProcess } from './managedProcess.js';

export function validateVisionOptions(body = {}, simulation = false) {
    const device = body.device ?? '0';
    const filter = body.filter ?? 'all';
    const url = String(body.url || '').trim();
    if (!['0', 'cpu'].includes(device) || !['all', 'traffic'].includes(filter)) throw new Error('device/filter 参数无效');
    if (url) {
        const parsed = new URL(url);
        if (!['http:', 'https:'].includes(parsed.protocol) || parsed.username || parsed.password) throw new Error('需要无凭据的 HTTP(S) 摄像头 URL');
        if (simulation && !['127.0.0.1', 'localhost', '[::1]'].includes(parsed.hostname)) throw new Error('仿真仅允许本地摄像头源');
    } else if (simulation) throw new Error('仿真必须指定本地摄像头 URL');
    return { device, filter, url };
}

export function installVisionRoutes({ app, repoRoot, simulation = process.env.ROBOT_SIMULATION === '1', gatewayPort }) {
    const port = gatewayPort ?? (simulation ? 18321 : 18320);
    const base = `http://127.0.0.1:${port}`;
    let proc = null, operation = null, error = '', starting = false, closing = false;
    const previews = new Set();
    // 预览帧计数：raw / annotated 各自独立，前端据此做彼此独立的看门狗
    // （此前两路共用一个 frame.id，raw 卡住时会被 annotated 的帧掩盖）。
    const previewFrames = { raw: 0, annotated: 0 };
    const frameCarry = { raw: '', annotated: '' };
    function countFrames(kind, chunk) {
        // multipart 边界 "--frame" 可能跨 chunk，保留 8 字节尾巴拼接后再数。
        const text = frameCarry[kind] + chunk.toString('latin1');
        let count = 0;
        for (let at = text.indexOf('--frame'); at >= 0; at = text.indexOf('--frame', at + 7)) count++;
        frameCarry[kind] = text.slice(-8);
        if (count) previewFrames[kind] += count;
    }
    async function gateway(endpoint, body) {
        const res = await fetch(base + endpoint, {
            method: body ? 'POST' : 'GET', signal: AbortSignal.timeout(1500),
            ...(body ? { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {})
        });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || `视觉网关 HTTP ${res.status}`);
        return data;
    }
    async function status() {
        try {
            const state = await gateway('/status');
            if (state.service !== 'ldc_vision') throw new Error('端口被其他服务占用');
            return { ...state, simulation, starting, error, previewFrames: { ...previewFrames } };
        } catch (e) {
            return { running: false, simulation, starting, error: error || (proc ? '视觉网关不可用' : ''),
                camera: { ok: false, detail: '无实时画面' }, detector: { ok: false, detail: '识别服务未就绪' },
                detections: [], frame: null, previewFrames: { ...previewFrames } };
        }
    }
    async function stop() {
        for (const stream of previews) stream.destroy();
        const state = await status();
        if (state.running) await gateway('/shutdown', {});
        if (proc) { const owned = proc; await terminateProcess(owned); if (proc === owned) proc = null; }
        // This also supports stopping a launch started from the CLI gateway.
        for (let i = 0; i < 40; i++) {
            if (!(await status()).running) return;
            await new Promise(resolve => setTimeout(resolve, 100));
        }
        throw new Error('视觉进程仍未退出');
    }
    app.get('/api/vision/status', async (_req, res) => res.json(await status()));
    app.post('/api/vision/start', async (req, res) => {
        if (closing) return res.status(503).json({ error: '服务正在退出' });
        if (operation) return res.status(202).json(await status());
        const start = async () => {
            const options = validateVisionOptions(req.body, simulation);
            const existing = await status();
            if (existing.running) return existing;
            error = ''; starting = true;
            validateVisionOptions(options, simulation);
            if (closing) throw new Error('服务正在退出，已取消视觉启动');
            proc = spawn('bash', [path.join(repoRoot, 'scripts/start_vision.sh'), simulation ? '--sim' : '--real',
                `url:=${options.url}`, `device:=${options.device}`, `class_filter:=${options.filter}`], { stdio: ['ignore', 'pipe', 'pipe'] });
            const launched = proc;
            // Consume output continuously without accumulating frames/logs in memory.
            for (const output of [launched.stdout, launched.stderr]) output.on('data', chunk => {
                const text = chunk.toString();
                if (/ERROR|Traceback|Exception|failed/i.test(text)) console.error('[vision]', text.slice(-1200));
            });
            launched.on('error', e => { error = `视觉启动失败：${e.message}`; });
            launched.on('exit', code => {
                if (proc === launched) proc = null;
                if (code && !closing) error = `视觉进程退出 (${code})，请检查 ROS 日志`;
            });
            // Gateway readiness is separate from GPU/camera readiness, reported in status.
            for (let i = 0; i < 60; i++) {
                const state = await status();
                if (state.running) return state;
                if (!proc) throw new Error(error || '视觉启动进程已退出');
                await new Promise(resolve => setTimeout(resolve, 100));
            }
            throw new Error('视觉网关启动超时');
        };
        operation = start();
        try { res.json(await operation); }
        catch (e) { error = e.message; if (proc) await terminateProcess(proc); proc = null; res.status(400).json({ error }); }
        finally { starting = false; operation = null; }
    });
    app.post('/api/vision/stop', async (_req, res) => {
        if (operation) return res.status(409).json({ error: '视觉启停正在执行' });
        operation = stop();
        try { await operation; error = ''; res.json({ success: true }); }
        catch (e) { error = e.message; res.status(500).json({ error }); }
        finally { operation = null; }
    });
    app.post('/api/vision/filter', async (req, res) => {
        try {
            const { filter } = validateVisionOptions({ filter: req.body?.filter });
            res.json(await gateway('/filter', { mode: filter }));
        } catch (e) { res.status(400).json({ error: e.message }); }
    });
    for (const kind of ['raw', 'annotated']) app.get(`/api/vision/${kind}.mjpg`, (_req, res) => {
        if (previews.size >= 4 || closing) return res.status(429).end('预览连接数已达上限');
        const upstream = http.get(`${base}/${kind}.mjpg`, incoming => {
            if (incoming.statusCode !== 200) { incoming.resume(); res.status(incoming.statusCode).end(); return; }
            res.setHeader('Content-Type', incoming.headers['content-type']);
            res.setHeader('Cache-Control', 'no-store');
            // Never backpressure the producer or grow a proxy queue for a slow viewer.
            incoming.on('data', chunk => {
                countFrames(kind, chunk);
                if (res.write(chunk)) return;
                incoming.pause();
                const cleanup = () => {
                    clearTimeout(timeout);
                    res.off('drain', resume);
                    res.off('close', cleanup);
                };
                const resume = () => { cleanup(); incoming.resume(); };
                const timeout = setTimeout(() => { cleanup(); upstream.destroy(); res.destroy(); }, 1000);
                res.once('drain', resume);
                res.once('close', cleanup);
            });
            incoming.on('end', () => res.end());
            incoming.on('error', () => res.destroy());
        });
        previews.add(upstream);
        upstream.setTimeout(2500, () => upstream.destroy());
        upstream.on('error', () => { if (!res.headersSent) res.status(503).end(); else res.destroy(); });
        upstream.on('close', () => previews.delete(upstream));
        res.on('close', () => { upstream.destroy(); previews.delete(upstream); });
    });
    return { status, async close() {
        closing = true;
        if (operation) await operation.catch(() => {});
        for (const stream of previews) stream.destroy();
        // Backend shutdown only owns launches it created. An explicit /stop
        // may stop a CLI session; merely querying it never transfers ownership.
        if (proc) await stop();
    } };
}
