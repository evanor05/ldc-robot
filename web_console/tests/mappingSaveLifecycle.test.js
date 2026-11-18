import assert from 'node:assert/strict';
import fs from 'node:fs';
import http from 'node:http';
import { once } from 'node:events';
import test from 'node:test';
import vm from 'node:vm';
import express from 'express';

// Exercise the registered production HTTP handlers, with only ROS/process I/O
// replaced. The full offline acceptance also runs these paths with GMapping.
async function fixture(t) {
    const app = express();
    app.use(express.json());
    const state = { running: true, stops: 0, finished: [], saved: [], aborts: 0,
        error: null, durationError: null, stopError: null };
    const map = { name: 'fixture', path: '/isolated/fixture' };
    const config = { tasks: { mapping: [{ id: 'gmapping' }] }, save_map: {} };
    const source = fs.readFileSync(new URL('../server.js', import.meta.url), 'utf8');
    const start = source.indexOf("app.post('/api/stop_task/:taskName'");
    const end = source.indexOf('// 兼容旧的接口', start);
    assert.ok(start > 0 && end > start);
    vm.runInNewContext(`let mappingStopping=false, mappingStarting=false, abortInFlight=null;
        const mappingStartedAt=1, mappingGeneration=1, mappingMode='manual';
        ${source.slice(start, end)}`, {
        app: { post(route, handler) { app.post(route, (req, res, next) => Promise.resolve(handler(req, res)).catch(next)); },
            get: app.get.bind(app) },
        console: { log() {}, error() {} }, loadConfig: () => config,
        buildRosCommand: () => { throw new Error('ROS must not run in this HTTP fixture'); },
        taskRunning: () => state.running,
        activeProcesses: { get gmapping() { return state.running ? {} : null; } },
        assertMappingDuration() { if (state.durationError) throw state.durationError; },
        automaticMapName: () => 'fixture',
        async saveMap() { if (state.error) throw state.error; return map; },
        rememberMap: saved => state.saved.push(saved),
        async terminateTaskProcesses() {
            state.stops++;
            if (state.stopError) return { failed: [{ id: 'gmapping', error: state.stopError }] };
            state.running = false;
            return { failed: [] };
        },
        finalizeMapping() {}, restoredMap: () => state.saved.at(-1) || null,
        mappingRun: { runId: 'fixture', async finish(...args) { state.finished.push(args); } },
        abortMapping(_req, res) { state.aborts++; state.running = false; res.json({ success: true, aborted: true }); }
    });
    const server = http.createServer(app);
    server.listen(0, '127.0.0.1');
    await once(server, 'listening');
    t.after(async () => { server.closeAllConnections(); await new Promise(resolve => server.close(resolve)); });
    const base = `http://127.0.0.1:${server.address().port}/api/`;
    return { state, async stop(suffix = '') {
        const response = await fetch(base + 'stop_task/mapping' + suffix, {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
        return { status: response.status, body: await response.json() };
    }, async status() { return (await fetch(base + 'task_status/mapping')).json(); } };
}

test('failed map saves retain the running session and permit a successful retry', async t => {
    const f = await fixture(t);
    for (const message of ['EACCES: map directory', '保存地图超时', '地图文件不完整', '地图内容不足']) {
        f.state.error = Object.assign(new Error(message), { statusCode: 422 });
        const failed = await f.stop();
        assert.equal(failed.status, 422, JSON.stringify(failed.body));
        assert.equal(failed.body.success, false);
        assert.equal(failed.body.stopped, false);
        assert.equal(failed.body.savedMap, null);
        assert.match(failed.body.error, new RegExp(message));
        const status = await f.status();
        assert.equal(status.isRunning, true);
        assert.equal(status.saving, false);
        assert.equal(f.state.stops, 0);
        assert.equal(f.state.finished.length, 0, 'Failed saves must not finalize the running manifest');
        assert.equal(f.state.saved.length, 0);
    }
    f.state.error = null;
    const retry = await f.stop();
    assert.equal(retry.body.success, true);
    assert.equal(retry.body.stopped, true);
    assert.equal(f.state.stops, 1);
    assert.equal(f.state.finished[0][0], 'success');
    assert.equal((await f.status()).isRunning, false);
    const repeated = await f.stop();
    assert.equal(repeated.body.savedMap, null);
    assert.equal(f.state.saved.length, 1);
});

test('minimum mapping duration failure keeps the session; explicit abort remains available', async t => {
    const f = await fixture(t);
    f.state.durationError = Object.assign(new Error('建图时间不足'), { statusCode: 409 });
    const failed = await f.stop();
    assert.equal(failed.status, 409);
    assert.equal(failed.body.stopped, false);
    assert.equal((await f.status()).isRunning, true);
    const aborted = await f.stop('?save=false');
    assert.equal(aborted.body.aborted, true);
    assert.equal(f.state.aborts, 1);
    assert.equal(f.state.saved.length, 0);
});

test('saved map does not imply a successful stop when process termination fails', async t => {
    const f = await fixture(t);
    f.state.stopError = 'owned child still running';
    const failed = await f.stop();
    assert.equal(failed.status, 500);
    assert.equal(failed.body.success, false);
    assert.equal(failed.body.stopped, false);
    assert.match(failed.body.error, /owned child/);
    assert.equal(f.state.saved.length, 1);
    assert.equal(f.state.finished.length, 0);
    assert.equal((await f.status()).isRunning, true);
    assert.equal((await f.status()).saving, false);
});
