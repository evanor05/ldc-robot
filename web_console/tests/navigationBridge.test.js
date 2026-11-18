// NavigationBridge（后端 ↔ 车端 python 桥的 JSONL 通道）此前零测试覆盖，
// 而它是所有运动指令的必经之路。这里用一个假桥进程覆盖：id 关联、失败、超时、
// goals 超时补发 estop、子进程退出、close() 语义。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { setTimeout as delay } from 'node:timers/promises';
import { NavigationBridge } from '../navigationBridge.js';

const FAKE_BRIDGE = `
const readline = require('node:readline');
const fs = require('node:fs');
const log = process.env.FAKE_BRIDGE_LOG;
const rl = readline.createInterface({ input: process.stdin });
rl.on('line', line => {
  if (log) fs.appendFileSync(log, line + '\\n');
  let req;
  try { req = JSON.parse(line); } catch { return; }
  if (req.op === 'hang' || req.op === 'goals') return;  // 故意不回（goals 用来验证超时补发 estop）
  if (req.op === 'shutdown') { process.exit(0); }       // 模拟正常退出
  if (req.op === 'fail') {
    console.log(JSON.stringify({ id: req.id, success: false, error: '故意失败' }));
    return;
  }
  console.log(JSON.stringify({ id: req.id, success: true, echo: req.op }));
});
`;

function setup(bridgeOptions = {}) {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'fake-bridge-'));
    const script = path.join(dir, 'bridge.cjs');
    const log = path.join(dir, 'ops.log');
    fs.writeFileSync(script, FAKE_BRIDGE);
    process.env.FAKE_BRIDGE_LOG = log;
    const bridge = new NavigationBridge(`${process.execPath} ${script}`, bridgeOptions);
    const ops = () => (fs.existsSync(log)
        ? fs.readFileSync(log, 'utf8').trim().split('\n').filter(Boolean).map(line => JSON.parse(line))
        : []);
    return { bridge, ops, dir };
}

function teardown({ bridge, dir }) {
    try { bridge.close(); } catch { /* 已退出 */ }
    try { bridge.process?.kill('SIGKILL'); } catch { /* 已退出 */ }
    fs.rmSync(dir, { recursive: true, force: true });
}

test('正常请求按 id 关联并原样返回数据', async () => {
    const fixture = setup();
    try {
        const first = await fixture.bridge.request('status', { a: 1 });
        assert.equal(first.echo, 'status');
        assert.equal(typeof first.id, 'number');
        const second = await fixture.bridge.request('events', { since: 3 });
        assert.equal(second.echo, 'events');
        assert.ok(second.id > first.id, 'id 必须单调递增');
        // 请求体里的字段要原样透传给车端
        assert.deepEqual(fixture.ops()[1], { ...{ since: 3 }, op: 'events', id: second.id });
    } finally { teardown(fixture); }
});

test('车端返回 success=false 时 reject 并带上原因', async () => {
    const fixture = setup();
    try {
        await assert.rejects(() => fixture.bridge.request('fail'), /故意失败/);
        // 失败后 pending 必须清干净，否则后续请求会串号
        assert.equal(fixture.bridge.pending.size, 0);
        const ok = await fixture.bridge.request('status');
        assert.equal(ok.echo, 'status');
    } finally { teardown(fixture); }
});

test('超时 reject，且 goals 超时会补发一次 estop（不让目标悬空）', async () => {
    const fixture = setup({ timeoutMs: 150 });
    try {
        await assert.rejects(() => fixture.bridge.request('hang'), /响应超时/);
        await assert.rejects(() => fixture.bridge.request('goals'), /响应超时/);
        await delay(200);   // 给补发的 estop 一点时间落盘
        const ops = fixture.ops();
        const estops = ops.filter(op => op.op === 'estop');
        assert.equal(estops.length, 1, `goals 超时应补发 1 次 estop，实际 ${ops.length} 条指令`);
        assert.equal(estops[0].id, undefined, 'estop 是兜底指令，不带请求 id');
    } finally { teardown(fixture); }
});

test('车端进程退出后：在途请求立即 reject，下一次请求会重启进程', async () => {
    const fixture = setup();
    try {
        const ok = await fixture.bridge.request('status');
        assert.equal(ok.echo, 'status');
        // 假桥收到 shutdown 后退出，触发 bridge 的退出处理
        await assert.rejects(() => fixture.bridge.request('shutdown'), /已退出|超时/);
        assert.equal(fixture.bridge.process, null, '退出后必须清掉进程句柄');
        // 再发请求应自动重启一个新桥进程
        const restarted = await fixture.bridge.request('status');
        assert.equal(restarted.echo, 'status');
    } finally { teardown(fixture); }
});

test('close() 发送 shutdown 帧而不是直接杀进程', async () => {
    const fixture = setup();
    try {
        await fixture.bridge.request('status');
        fixture.bridge.close();
        await delay(200);
        const ops = fixture.ops();
        assert.ok(ops.some(op => op.op === 'shutdown' && op.id === undefined),
            `close() 应以无 id 的 shutdown 收尾，实际=${JSON.stringify(ops.slice(-2))}`);
    } finally { teardown(fixture); }
});
