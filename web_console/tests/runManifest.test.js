// 建图运行证据采集的离线测试：只断言传给 python 的 argv 与状态机，不真的跑 python。
import assert from 'node:assert/strict';
import test from 'node:test';
import { createRunManifest } from '../runManifest.js';

function recorder(replies = {}) {
    const calls = [];
    const exec = (python, args, options, callback) => {
        calls.push({ python, args, options });
        // 注意：exec 的 args[0] 是脚本路径，子命令在 args[1]
        const key = args[1];
        const reply = replies[key];
        if (reply === undefined) callback(new Error('boom'), '');
        else callback(null, reply);
    };
    return { calls, exec };
}

test('start 把生效参数转成 --param 并解析出 runId', async () => {
    const { calls, exec } = recorder({ start: 'runId=run_20261111T180000_mapping\nrunDir=/tmp/x\n' });
    const manifest = createRunManifest({ scriptPath: '/repo/scripts/run_manifest.py', cwd: '/repo', exec });
    const runId = await manifest.start('mapping', {
        mode: 'auto', mapDir: '/repo/maps', sessionId: 'session-1', empty: '', skipped: null
    });
    assert.equal(runId, 'run_20261111T180000_mapping');
    assert.equal(manifest.runId, runId);
    assert.deepEqual(calls[0].args.slice(0, 3), ['/repo/scripts/run_manifest.py', 'start', '--kind']);
    const params = calls[0].args.filter((_, index) => calls[0].args[index - 1] === '--param');
    assert.deepEqual(params, ['mode=auto', 'mapDir=/repo/maps', 'sessionId=session-1'],
        '空值/null 不应写进证据，避免 manifest 里出现无意义字段');
});

test('start 失败时 runId 为 null，且不会假装已启动', async () => {
    const { exec } = recorder({});          // 回调 error
    const manifest = createRunManifest({ scriptPath: '/s.py', exec });
    assert.equal(await manifest.start('mapping', {}), null);
    assert.equal(manifest.runId, null);
    assert.equal(manifest.everStarted, false);
});

test('finish 用当前 runId 收尾，并在之后清空（不重复 finish）', async () => {
    const { calls, exec } = recorder({ start: 'runId=run_a\n', finish: '{}' });
    const manifest = createRunManifest({ scriptPath: '/s.py', exec });
    await manifest.start('mapping', {});
    await manifest.finish('success');
    assert.deepEqual(calls[1].args.slice(1), ['finish', 'run_a', '--status', 'success']);
    await manifest.finish('success');       // 第二次不能再发
    assert.equal(calls.length, 2, 'finish 必须幂等，不能重复收尾');
});

test('finish 把原因截断后写入，避免超长串进证据', async () => {
    const { calls, exec } = rendererSafe();
    const manifest = createRunManifest({ scriptPath: '/s.py', exec });
    await manifest.start('mapping', {});
    await manifest.finish('failed', 'x'.repeat(1000));
    const reasonIndex = calls[1].args.indexOf('--reason');
    assert.equal(calls[1].args[reasonIndex + 1].length, 400);
});

test('runsDir 通过环境变量传给 python，不写死仓库路径', async () => {
    const { calls, exec } = recorder({ start: 'runId=run_b\n' });
    const manifest = createRunManifest({ scriptPath: '/s.py', runsDir: '/tmp/ldc-runs', exec });
    await manifest.start('mapping', {});
    assert.equal(calls[0].options.env.ROBOT_RUNS_DIR, '/tmp/ldc-runs');
});

test('exec 同步抛错不会让建图流程崩（best-effort）', async () => {
    const exec = () => { throw new Error('EACCES'); };
    const manifest = createRunManifest({ scriptPath: '/s.py', exec });
    assert.equal(await manifest.start('mapping', {}), null);
    assert.equal(await manifest.finish('aborted'), null);
});

function rendererSafe() {
    return recorder({ start: 'runId=run_c\n', finish: '{}' });
}
