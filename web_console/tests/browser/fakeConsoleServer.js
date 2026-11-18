// 真浏览器验收用的假后端：静态 dist + HTTP API + 最小 Foxglove WebSocket 桥。
// 目的：在没有车、没有 ROS 的情况下，用真实浏览器渲染真实构建产物。
// 只用 Node 内置模块（http/crypto），不引入任何依赖。
import crypto from 'node:crypto';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const DIST = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../dist');
const MIME = {
    '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.map': 'application/json'
};

// 默认场景：地图目录里只有差图 → 开始导航必须被门控住。
export const defaultFixtures = () => ({
    maps: [{
        name: 'map_20261111180316_294', path: '/fake/maps/map_20261111180316_294',
        modifiedAt: 1762850000000, readable: true, navigable: false,
        quality: { width: 384, height: 384, resolution: 0.05, origin: [-10, -10, 0],
            occupied: 1717, free: 0, unknown: 0, total: 147471, unknownRatio: 0.861 },
        qualityError: '地图内容不足：有效占用栅格 1717（最低 2500），未知占比 86.1%（上限 85.0%）'
    }],
    thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85, expectedResolution: 0.05 },
    mappingRunning: false,
    vision: {
        running: true, starting: false, simulation: false,
        camera: { ok: false, detail: '无实时画面' },
        detector: { ok: false, detail: '识别服务未就绪' },
        detections: [], frame: null, previewFrames: { raw: 0, annotated: 0 }
    }
});

function json(res, body, status = 200) {
    const payload = JSON.stringify(body);
    res.writeHead(status, {
        'Content-Type': 'application/json; charset=utf-8',
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Headers': 'Content-Type',
        'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
        'Content-Length': Buffer.byteLength(payload)
    });
    res.end(payload);
}

function serveStatic(req, res) {
    const url = new URL(req.url, 'http://localhost');
    const relative = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
    const target = path.resolve(DIST, relative);
    if (!target.startsWith(DIST)) { res.writeHead(403).end('forbidden'); return; }
    if (!fs.existsSync(target) || !fs.statSync(target).isFile()) {
        // SPA 回退
        const index = path.join(DIST, 'index.html');
        if (!fs.existsSync(index)) { res.writeHead(404).end('dist 未构建：先 npm run build'); return; }
        res.writeHead(200, { 'Content-Type': MIME['.html'] }).end(fs.readFileSync(index));
        return;
    }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(target)] || 'application/octet-stream' })
        .end(fs.readFileSync(target));
}

// ---- 极简 WebSocket 文本/二进制写出 + 客户端帧解析（浏览器帧必带 mask）----
const MAP_CHANNEL_ID = 1;
// 小型、非方形、非零 origin、resolution≠1：能暴露"把地图当方形/忽略 origin"这类错误
const MAP_INFO = { resolution: 0.05, width: 20, height: 12, originX: -1.5, originY: 2.25 };
const MAP_CHANNELS = [{
    id: MAP_CHANNEL_ID, topic: '/map', encoding: 'json',
    schemaName: 'nav_msgs/OccupancyGrid', schemaEncoding: 'jsonschema', schema: ''
}];

function occupancyGrid() {
    const stamp = { sec: Math.floor(Date.now() / 1000), nanosec: 0 };
    return {
        header: { stamp, frame_id: 'map' },
        info: {
            map_load_time: stamp, resolution: MAP_INFO.resolution,
            width: MAP_INFO.width, height: MAP_INFO.height,
            origin: {
                position: { x: MAP_INFO.originX, y: MAP_INFO.originY, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            }
        },
        data: new Array(MAP_INFO.width * MAP_INFO.height).fill(0)
    };
}

function wsHeader(opcode, length) {
    if (length < 126) return Buffer.from([0x80 | opcode, length]);
    if (length < 65536) {
        const header = Buffer.alloc(4);
        header[0] = 0x80 | opcode;
        header[1] = 126;
        header.writeUInt16BE(length, 2);
        return header;
    }
    const header = Buffer.alloc(10);
    header[0] = 0x80 | opcode;
    header[1] = 127;
    header.writeBigUInt64BE(BigInt(length), 2);
    return header;
}

function writeText(socket, text) {
    const payload = Buffer.from(text);
    socket.write(Buffer.concat([wsHeader(0x1, payload.length), payload]));
}

// Message Data：opcode 0x01 + subscriptionId(uint32 LE) + timestamp(uint64 LE) + payload
function writeBinary(socket, subscriptionId, text) {
    const payload = Buffer.from(text);
    const body = Buffer.alloc(13 + payload.length);
    body[0] = 0x01;
    body.writeUInt32LE(subscriptionId, 1);
    body.writeBigUInt64LE(BigInt(Date.now()) * 1_000_000n, 5);
    payload.copy(body, 13);
    socket.write(Buffer.concat([wsHeader(0x2, body.length), body]));
}

function readClientFrames(socket, onText) {
    let buffer = Buffer.alloc(0);
    socket.on('data', chunk => {
        buffer = Buffer.concat([buffer, chunk]);
        for (;;) {
            if (buffer.length < 2) return;
            const opcode = buffer[0] & 0x0f;
            const masked = (buffer[1] & 0x80) !== 0;
            let length = buffer[1] & 0x7f;
            let offset = 2;
            if (length === 126) {
                if (buffer.length < 4) return;
                length = buffer.readUInt16BE(2);
                offset = 4;
            } else if (length === 127) {
                if (buffer.length < 10) return;
                length = Number(buffer.readBigUInt64BE(2));
                offset = 10;
            }
            const maskOffset = offset;
            if (masked) offset += 4;
            if (buffer.length < offset + length) return;
            let payload = Buffer.from(buffer.subarray(offset, offset + length));
            if (masked) {
                const mask = buffer.subarray(maskOffset, maskOffset + 4);
                for (let index = 0; index < payload.length; index++) payload[index] ^= mask[index % 4];
            }
            buffer = buffer.subarray(offset + length);
            if (opcode === 0x1) onText(payload.toString('utf8'));
            else if (opcode === 0x8) return;
            else if (opcode === 0x9) socket.write(Buffer.from([0x8a, 0x00]));
        }
    });
}

export async function startFakeConsole({ apiPort = 3000, wsPort = 8765, fixtures = defaultFixtures() } = {}) {
    const requests = [];
    const state = structuredClone(fixtures);
    state.unexpectedRequests = [];
    state.mapSubscriptions = 0;
    state.mapFramesSent = 0;

    const api = http.createServer((req, res) => {
        if (req.method === 'OPTIONS') { json(res, {}); return; }
        const url = new URL(req.url, 'http://localhost');
        let body = '';
        req.on('data', chunk => { body += chunk; });
        req.on('end', () => {
            let parsed = null;
            try { parsed = body ? JSON.parse(body) : null; } catch { parsed = body; }
            if (!url.pathname.startsWith('/api/')) { serveStatic(req, res); return; }
            requests.push({ method: req.method, path: url.pathname, query: url.search, body: parsed });
            switch (url.pathname) {
                case '/api/runtime_config':
                    return json(res, { topics: { battery: '/battery_state', initial_pose: '/initialpose' },
                        camera: { stream_url: '' }, simulation: false });
                case '/api/list_maps':
                    return json(res, { maps: state.maps, thresholds: state.thresholds });
                case '/api/task_status/mapping':
                case '/api/mapping_status':
                    return json(res, { isRunning: state.mappingRunning, saving: false,
                        mode: 'manual', lastSavedMap: null });
                case '/api/run_task/mapping':
                case '/api/start_mapping':
                    state.mappingRunning = true;
                    return json(res, { success: true, message: '任务 [mapping] 已按配置启动' });
                case '/api/stop_task/mapping':
                case '/api/stop_mapping':
                    if (url.searchParams.get('save') === 'false') {
                        state.mappingRunning = false;
                        return json(res, { success: true, aborted: true, savedMap: null,
                            message: '建图已中止（未保存）' });
                    }
                    return json(res, { success: false, stopped: true, saveFailed: true,
                        error: '地图内容不足：有效占用栅格 1717（最低 2500）' }, 422);
                case '/api/abort_mapping':
                    state.mappingRunning = false;
                    return json(res, { success: true, aborted: true, savedMap: null,
                        message: '建图已中止（未保存）' });
                case '/api/navigation_status':
                    return json(res, { isRunning: false, phase: 'stopped', ready: false,
                        missing: ['实时雷达 /scan：尚未收到有效样本', '前进 10s 仍未到达'],
                        error: '', mapPath: '', mode: 'locked', goalState: 'idle',
                        checks: { lidar: { ok: false, detail: '尚未收到有效样本' },
                            odometry: { ok: false, detail: '尚未收到有效样本' } },
                        nextAction: '检查小车电源与 Wi-Fi，等待雷达和里程计恢复；运动保持锁定' });
                case '/api/navigation/events':
                    return json(res, { events: [], latest: 0 });
                case '/api/vision/status':
                    return json(res, state.vision);
                case '/api/control/claim':
                    return json(res, { success: true, owner: parsed?.sessionId || 'fake-session' });
                case '/api/control/release':
                    return json(res, { success: true });
                case '/api/navigation/heartbeat':
                    return json(res, { success: true });
                case '/api/navigation/cancel':
                    return json(res, { success: true });
                case '/api/navigation/initial_pose':
                    return json(res, { success: true });
                case '/api/client_log':
                    return json(res, { success: true, stored: false });
                default:
                    // 未实现的接口必须显式失败：否则漏实现会被"success"掩盖，
                    // 浏览器用例会自嗨通过。测试末尾断言 unexpectedRequests 为空。
                    state.unexpectedRequests.push({ method: req.method, path: url.pathname });
                    return json(res, { success: false, error: `fake 未实现 ${req.method} ${url.pathname}` }, 404);
            }
        });
    });
    await new Promise(resolve => api.listen(apiPort, '127.0.0.1', resolve));

    // ---- 最小 Foxglove WebSocket 桥：握手 + 一条 serverInfo 文本帧 ----
    const sockets = new Set();
    const ws = http.createServer((_req, res) => res.writeHead(426).end('websocket only'));
    ws.on('upgrade', (req, socket) => {
        const key = req.headers['sec-websocket-key'];
        if (!key) { socket.destroy(); return; }
        const accept = crypto.createHash('sha1')
            .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
        // 浏览器端用 new WebSocket(url, ['foxglove.sdk.v1', ...]) 指定子协议，
        // 服务端不回选会导致握手直接失败（实测：页面显示"连接失败"）。
        const offered = (req.headers['sec-websocket-protocol'] || '')
            .split(',').map(item => item.trim()).filter(Boolean);
        const protocolHeader = offered.length
            ? `Sec-WebSocket-Protocol: ${offered[0]}\r\n` : '';
        socket.write('HTTP/1.1 101 Switching Protocols\r\n'
            + 'Upgrade: websocket\r\nConnection: Upgrade\r\n'
            + `Sec-WebSocket-Accept: ${accept}\r\n${protocolHeader}\r\n`);
        sockets.add(socket);
        socket.on('error', () => sockets.delete(socket));
        socket.on('close', () => sockets.delete(socket));
        socket.on('data', () => { /* 忽略客户端订阅帧 */ });
        writeText(socket, JSON.stringify({ op: 'serverInfo', name: 'fake-foxglove',
            capabilities: [], supportedEncodings: ['json'], metadata: {}, sessionId: 'fake' }));
        // App 只在通道被 advertise 后才会订阅；/map 用 JSON 编码（前端已有 JSON 分支，
        // 不需要实现 CDR OccupancyGrid 编码器）。
        writeText(socket, JSON.stringify({ op: 'advertise', channels: MAP_CHANNELS }));
        readClientFrames(socket, message => {
            let parsed;
            try { parsed = JSON.parse(message); } catch { return; }
            if (parsed.op !== 'subscribe') return;
            for (const subscription of parsed.subscriptions || []) {
                if (subscription.channelId !== MAP_CHANNEL_ID) continue;
                state.mapSubscriptions += 1;
                const send = () => {
                    writeBinary(socket, subscription.id, JSON.stringify(occupancyGrid()));
                    state.mapFramesSent += 1;
                };
                send();
                const timer = setInterval(send, 500);
                socket.on('close', () => clearInterval(timer));
            }
        });
    });
    await new Promise(resolve => ws.listen(wsPort, '127.0.0.1', resolve));

    return {
        requests,
        state,
        apiPort,
        wsPort,
        setFixtures(patch) { Object.assign(state, patch); },
        async close() {
            for (const socket of sockets) socket.destroy();
            await new Promise(resolve => ws.close(resolve));
            await new Promise(resolve => api.close(resolve));
        }
    };
}
