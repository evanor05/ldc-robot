import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import assert from 'node:assert/strict';
import express from 'express';
import { installNavigationRoutes } from '../navigationRoutes.js';
import { terminateProcess } from '../managedProcess.js';

const fakeBridge = path.join(os.tmpdir(), `fake-bridge-${process.pid}.mjs`);
fs.copyFileSync(new URL('./fixtures/fake_bridge.js', import.meta.url), fakeBridge);

async function install({ owner = null, withMap = true } = {}) {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'nav-routes-'));
    const mapsDir = path.join(dir, 'maps');
    fs.mkdirSync(mapsDir);
    if (withMap) {
        fs.writeFileSync(path.join(mapsDir, 'room.yaml'), 'image: room.pgm\nresolution: 0.05\n');
        fs.writeFileSync(path.join(mapsDir, 'room.pgm'), 'P5\n1 1\n255\n');
    }
    const opsFile = path.join(dir, 'ops.log');
    process.env.FAKE_BRIDGE_OPS = opsFile;
    if (owner) process.env.FAKE_OWNER = owner; else delete process.env.FAKE_OWNER;
    const config = {
        maps_dir: mapsDir,
        tasks: { navigation: [{ id: 'nav2-test', description: 'test', command: 'sleep', args: ['5'] }] },
        save_map: { command: 'true', args: [], save_dir: mapsDir }
    };
    const app = express();
    app.use(express.json());
    const activeProcesses = {};
    const routes = installNavigationRoutes({
        app, root: path.dirname(new URL('../navigationRoutes.js', import.meta.url).pathname),
        loadConfig: () => config,
        buildRosCommand: (command, args) =>
            (command === 'python3' && String(args[0]).endsWith('navigation_bridge.py'))
                ? `node ${fakeBridge}`
                : command === 'sleep' ? 'sleep 60' : `${command} ${(args || []).join(' ')}`,
        activeProcesses, mappingBusy: () => false
    });
    const server = await new Promise(resolve => { const s = app.listen(0, '127.0.0.1', () => resolve(s)); });
    const base = `http://127.0.0.1:${server.address().port}/api`;
    const ops = () => fs.existsSync(opsFile)
        ? fs.readFileSync(opsFile, 'utf8').trim().split('\n').filter(Boolean).map(line => JSON.parse(line))
        : [];
    const post = async (endpoint, body) => {
        const response = await fetch(`${base}/${endpoint}`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        return { status: response.status, data: await response.json() };
    };
    const get = async endpoint => {
        const response = await fetch(`${base}/${endpoint}`);
        return { status: response.status, data: await response.json() };
    };
    const cleanup = async () => {
        delete process.env.FAKE_OWNER;
        await post('stop_navigation', {});
        server.closeAllConnections();
        await new Promise(resolve => server.close(resolve));
        await routes.close();
        for (const proc of Object.values(activeProcesses)) if (proc) { try { await terminateProcess(proc); } catch {} }
        fs.rmSync(dir, { recursive: true, force: true });
    };
    return { base, ops, post, get, cleanup, mapsDir, activeProcesses, routes };
}

test('duplicate requestId replays the same goal instead of resubmitting', async (t) => {
    const ctx = await install();
    t.after(ctx.cleanup);
    const body = { points: [{ x: 1, y: 0, yaw: 0 }], sessionId: 'page-a', requestId: 'req-dup-1' };
    const first = await ctx.post('navigation/goals', body);
    assert.equal(first.status, 200);
    assert.ok(first.data.success);
    const replay = await ctx.post('navigation/goals', { ...body });
    assert.deepEqual(replay.data, first.data, '重放必须返回首次结果');
    // 并发重复也只执行一次。
    const [c1, c2] = await Promise.all([
        ctx.post('navigation/goals', { ...body, requestId: 'req-dup-2' }),
        ctx.post('navigation/goals', { ...body, requestId: 'req-dup-2' })
    ]);
    assert.deepEqual(c1.data, c2.data);
    const goalOps = ctx.ops().filter(op => op.op === 'goals');
    assert.equal(goalOps.length, 2, `goals 应只提交两次（两个不同 requestId），实际 ${goalOps.length}`);
    // 不同 requestId 视为新请求。
    assert.equal(goalOps[0].requestId, 'req-dup-1');
    assert.equal(goalOps[1].requestId, 'req-dup-2');
});

test('heartbeat and velocity bypass idempotency and carry session', async (t) => {
    const ctx = await install();
    t.after(ctx.cleanup);
    for (let i = 0; i < 3; i++) {
        await ctx.post('navigation/heartbeat', { sessionId: 'page-a', requestId: 'same-id' });
    }
    await ctx.post('navigation/velocity', { sessionId: 'page-a', controlId: 'c', sequence: 1, linearX: 0, angularZ: 0, requestId: 'same-id' });
    await ctx.post('navigation/velocity', { sessionId: 'page-a', controlId: 'c', sequence: 2, linearX: 0, angularZ: 0, requestId: 'same-id' });
    const heartbeats = ctx.ops().filter(op => op.op === 'heartbeat');
    const velocities = ctx.ops().filter(op => op.op === 'velocity');
    assert.equal(heartbeats.length, 3, '心跳每次都送达（续期不能被缓存吞掉）');
    assert.equal(velocities.length, 2, '速度每次都送达（由 sequence 去重）');
});

test('control endpoints pass session through and map errors to 409', async (t) => {
    const ctx = await install({ owner: 'page-owner' });
    t.after(ctx.cleanup);
    const denied = await ctx.post('control/claim', { sessionId: 'page-b', requestId: 'r1' });
    assert.equal(denied.status, 409);
    assert.match(denied.data.error, /控制权由其他页面持有/);
    const takeover = await ctx.post('control/takeover', { sessionId: 'page-b', requestId: 'r2' });
    assert.equal(takeover.status, 200);
    assert.equal(takeover.data.owner, 'page-b');
    const deniedGoals = await ctx.post('navigation/goals', { points: [{ x: 1, y: 0 }], sessionId: 'page-owner', requestId: 'r3' });
    assert.equal(deniedGoals.status, 409, '旧租约不能发目标');
    const deniedHeartbeat = await ctx.post('navigation/heartbeat', { sessionId: 'page-owner' });
    assert.equal(deniedHeartbeat.status, 409, '旧租约不能续期');
    const estop = await ctx.post('navigation/estop', { sessionId: 'page-owner', requestId: 'r4' });
    assert.equal(estop.status, 200, '任意页面仍可急停');
    const release = await ctx.post('control/release', { sessionId: 'page-b', requestId: 'r5' });
    assert.equal(release.status, 200);
});

test('manual_enable cancels first then enables with session', async (t) => {
    const ctx = await install();
    t.after(ctx.cleanup);
    const result = await ctx.post('control/manual_enable', { sessionId: 'page-a', requestId: 'r6' });
    assert.equal(result.status, 200);
    assert.equal(result.data.controlId, 'fake-control');
    const order = ctx.ops().map(op => op.op);
    assert.ok(order.indexOf('cancel') < order.indexOf('manual_enable'), '必须先取消确认再启用');
    const enable = ctx.ops().find(op => op.op === 'manual_enable');
    assert.equal(enable.sessionId, 'page-a');
    // 幂等：重复 requestId 不再执行 cancel+enable。
    await ctx.post('control/manual_enable', { sessionId: 'page-a', requestId: 'r6' });
    assert.equal(ctx.ops().filter(op => op.op === 'manual_enable').length, 1);
});

test('start_navigation forwards session to begin and rejects foreign session', async (t) => {
    const ctx = await install({ owner: 'page-owner' });
    t.after(ctx.cleanup);
    const mapPath = path.join(ctx.mapsDir, 'room');
    const denied = await ctx.post('start_navigation', { mapPath, sessionId: 'page-b', requestId: 'r7' });
    assert.equal(denied.status, 409);
    assert.match(denied.data.error, /控制权/);
    const begin = ctx.ops().find(op => op.op === 'begin');
    assert.equal(begin.sessionId, 'page-b');
    assert.ok(!ctx.activeProcesses['nav2-test'], 'begin 被拒绝时不得启动导航进程');
    const allowed = await ctx.post('start_navigation', { mapPath, sessionId: 'page-owner', requestId: 'r8' });
    assert.equal(allowed.status, 200);
    assert.ok(allowed.data.success);
});

test('events endpoint forwards since cursor', async (t) => {
    const ctx = await install();
    t.after(ctx.cleanup);
    const result = await ctx.get('navigation/events?since=42');
    assert.equal(result.status, 200);
    assert.ok(Array.isArray(result.data.events));
    const eventsOp = ctx.ops().find(op => op.op === 'events');
    assert.equal(eventsOp.since, 42);
    const fallback = await ctx.get('navigation/events?since=not-a-number');
    assert.equal(fallback.status, 200);
    assert.equal(ctx.ops().filter(op => op.op === 'events').at(-1).since, 0);
});

test('status endpoint exposes control owner and readiness checks', async (t) => {
    const ctx = await install({ owner: 'page-owner' });
    t.after(ctx.cleanup);
    const result = await ctx.get('navigation_status');
    assert.equal(result.status, 200);
    assert.equal(result.data.controlOwner, 'page-owner');
    assert.ok('checks' in result.data && 'nextAction' in result.data);
});
