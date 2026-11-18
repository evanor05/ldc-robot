#!/usr/bin/env node
// 路由行为测试用的最小 JSON-lines 假桥接：
// 复刻 scripts/navigation_bridge.py 的会话归属语义（仅用于测试路由层，不用于生产）。
// 收到的每个操作追加到 FAKE_BRIDGE_OPS 文件；FAKE_OWNER 预置控制权所有者。
import fs from 'node:fs';
import readline from 'node:readline';

const opsFile = process.env.FAKE_BRIDGE_OPS || '';
const state = { owner: process.env.FAKE_OWNER || null, isRunning: false, seq: 0 };

function log(data) {
    if (!opsFile) return;
    fs.appendFileSync(opsFile, JSON.stringify({ at: Date.now(), ...data }) + '\n');
}
function send(payload) {
    process.stdout.write(JSON.stringify(payload) + '\n');
}

const rl = readline.createInterface({ input: process.stdin });
rl.on('line', line => {
    let data;
    try { data = JSON.parse(line); } catch { return; }
    log(data);
    const ok = extra => send({ id: data.id, success: true, ...extra });
    const fail = error => send({ id: data.id, success: false, error });
    const ownedByOther = sessionId => state.owner && sessionId !== state.owner;
    switch (data.op) {
        case 'status':
            return ok({ isRunning: state.isRunning, missing: [], phase: state.isRunning ? 'ready' : 'stopped',
                ready: state.isRunning, error: '', mapPath: '', mode: 'locked', goalId: null, goalState: 'idle',
                feedback: null, current: null, queue: [], localized: false,
                controlOwner: state.owner, leaseAgeSeconds: null, eventsSeq: 1, checks: {}, nextAction: '' });
        case 'events':
            return ok({ events: [{ seq: 1, at: Date.now() / 1000, runId: 1, goalId: null, type: 'startup_begin', reason: '' }], latest: 1 });
        case 'claim':
            if (!data.sessionId) return fail('缺少控制会话标识');
            if (ownedByOther(data.sessionId)) return fail('控制权由其他页面持有，请使用显式接管');
            state.owner = data.sessionId;
            return ok({ owner: state.owner, mode: 'locked' });
        case 'takeover':
            if (!data.sessionId) return fail('缺少控制会话标识');
            state.owner = data.sessionId;
            return ok({ owner: state.owner, mode: 'locked', controlId: null });
        case 'release':
            if (state.owner === data.sessionId) state.owner = null;
            return ok({ owner: state.owner });
        case 'begin':
            if (!data.sessionId) return fail('缺少控制会话标识');
            if (ownedByOther(data.sessionId)) return fail('控制权由其他页面持有，需先显式接管');
            state.owner = data.sessionId;
            state.isRunning = true;
            return ok({});
        case 'stop':
            state.isRunning = false;
            return ok({});
        case 'goals':
            if (!data.sessionId) return fail('缺少控制会话标识');
            if (ownedByOther(data.sessionId)) return fail('控制权由其他页面持有，需先显式接管');
            state.owner = data.sessionId;
            return ok({ goalId: `fake-goal-${data.requestId || ++state.seq}` });
        case 'cancel':
        case 'estop':
            return ok({});
        case 'manual_enable':
            if (!data.sessionId) return fail('缺少控制会话标识');
            if (ownedByOther(data.sessionId)) return fail('控制权由其他页面持有，需先显式接管');
            state.owner = data.sessionId;
            return ok({ controlId: 'fake-control' });
        case 'velocity':
            return ok({ controlId: data.controlId });
        case 'heartbeat':
            if (state.owner && data.sessionId !== state.owner) return fail('控制权不属于本页面，运动租约未续期');
            return ok({ owner: state.owner });
        case 'shutdown':
            send({ id: data.id, success: true });
            process.exit(0);
            return;
        default:
            return fail('未知操作');
    }
});
rl.on('close', () => process.exit(0));
