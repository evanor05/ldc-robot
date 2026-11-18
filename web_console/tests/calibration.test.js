import test from 'node:test';
import assert from 'node:assert/strict';
import express from 'express';
import http from 'node:http';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { once } from 'node:events';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';
import { fileURLToPath } from 'node:url';
import { installCalibrationRoutes } from '../calibrationRoutes.js';
import { terminateProcess } from '../managedProcess.js';
import { parse, compileScript, compileTemplate } from '@vue/compiler-sfc';

test('calibration API preserves candidates, independent sessions, revisions and rejects invalid input', async t => {
    const root = fs.mkdtempSync(path.join(os.tmpdir(), 'ldc-calibration-api-'));
    const repoRoot = fileURLToPath(new URL('../..', import.meta.url));
    const app = express(); app.use(express.json());
    const service = installCalibrationRoutes({ app, repoRoot, simulation: true, dataRoot: root,
        buildRosCommand: () => { throw new Error('This test must not launch ROS or motion'); } });
    const server = http.createServer(app); server.listen(0, '127.0.0.1'); await once(server, 'listening');
    t.after(async () => { await service.close(); server.closeAllConnections(); await new Promise(r => server.close(r)); fs.rmSync(root, { recursive: true }); });
    async function request(route, body, origin) {
        const response = await fetch(`http://127.0.0.1:${server.address().port}/api/calibration${route}`, {
            ...(body ? { method: 'POST', body: JSON.stringify(body) } : {}), headers: { 'Content-Type': 'application/json', ...(origin ? { Origin: origin } : {}) }
        });
        return { code: response.status, data: await response.json() };
    }
    assert.equal((await request('/sessions', { name: 'blocked' }, 'https://unrelated.invalid')).code, 403);
    const created = await request('/sessions', { name: '合成数据工具验证' });
    assert.equal(created.code, 200); const id = created.data.id;
    assert.equal(created.data.simulation, true);
    assert.equal(created.data.dimensions.body_extents_from_base_link_m.front, null);
    assert.equal(created.data.measurements.distance.length, 0);
    assert.equal((await request(`/sessions/${id}/measurement`, { kind: 'distance', row: {} })).code, 409);
    const row = { trial_id: 'synthetic_1', direction: 'forward', reference_m: 2, odom_m: 1.9, end_yaw_deg: 0, valid: 'true', notes: '明确为合成数据，非实车' };
    assert.equal((await request(`/sessions/${id}/measurement`, { kind: 'distance', row })).code, 200);
    assert.equal((await request(`/sessions/${id}/measurement`, { kind: 'distance', row })).code, 409);
    const report = (await request(`/sessions/${id}/analyze`, {})).data;
    assert.equal(report.applied, false);
    assert.ok(report.candidates.common_wheel_diameter_mm > 65);
    assert.ok(report.sourceHashes['distance.csv']);
    const validation = await request('/sessions', { name: '独立复测', validationOf: { sessionId: id, reportId: report.reportId } });
    assert.equal(validation.code, 200);
    assert.equal(validation.data.validationOf.reportId, report.reportId);
    assert.equal(validation.data.measurements.distance.length, 0);
    const validationId = validation.data.id;
    assert.equal((await request(`/sessions/${validationId}/measurement`, { kind: 'distance', row: { ...row, odom_m: 1.8 } })).code, 200);
    const verified = (await request(`/sessions/${validationId}/analyze`, {})).data;
    assert.equal(verified.independentValidation.appliedVersionVerified, false);
    assert.ok(Math.abs(verified.independentValidation.evaluation.distance.value - .1) < 1e-9);
    assert.equal(verified.independentValidation.evaluation.distance.meetsNumericalTarget, false);
    assert.equal(verified.independentValidation.evaluation.distance.sufficientTrials, false);
    assert.equal(verified.measurements.distance.fitted_error.maximum_absolute, 0);
    const boundarySession = (await request('/sessions', { name: '合成边界值', validationOf: { sessionId: id, reportId: report.reportId } })).data.id;
    await request(`/sessions/${boundarySession}/measurement`, { kind: 'distance', row: { ...row, odom_m: 1.98 } });
    const boundary = (await request(`/sessions/${boundarySession}/analyze`, {})).data;
    assert.equal(boundary.independentValidation.evaluation.distance.meetsNumericalTarget, true);
    assert.equal((await request(`/sessions/${id}/dimensions`, { values: { 'body_extents_from_base_link_m.front': -1 } })).code, 409);
    assert.equal((await request(`/sessions/${id}/dimensions`, { values: { 'sensor_extrinsics_from_base_link.camera_link.yaw_deg': 180 } })).code, 200);
    assert.equal((await request('/sessions', { validationOf: { sessionId: '../outside', reportId: 'a' } })).code, 409);
    const restored = (await request(`/sessions/${id}`)).data;
    assert.equal(restored.measurements.distance.length, 1);
    assert.equal(restored.reports[0].reportId, report.reportId);
    assert.ok(fs.readdirSync(path.join(root, id, 'revisions')).length >= 2);
    const interrupted = path.join(root, id, 'captures', 'restart_fixture');
    fs.mkdirSync(interrupted);
    fs.writeFileSync(path.join(interrupted, 'capture.json'), JSON.stringify({ state: 'recording', pid: 123456789 }));
    const afterRestart = (await request(`/sessions/${id}`)).data;
    assert.equal(afterRestart.captures[0].state, 'interrupted');
    // Hold the workspace lock so Stop arrives while record initialization is pending.
    const holder = spawn('/usr/bin/python3', ['-u', '-c',
        "import fcntl,sys; f=open(sys.argv[1],'a'); fcntl.flock(f,fcntl.LOCK_EX); print('ready',flush=True); sys.stdin.readline()", path.join(root, '.lock')]);
    t.after(() => { if (holder.exitCode === null) holder.kill(); });
    await once(holder.stdout, 'data');
    const pendingCapture = request(`/sessions/${id}/captures`, { seconds: 1, maxMb: 1 });
    const deadline = Date.now()+2000;
    while (!service.status().starting && Date.now()<deadline) await delay(10);
    assert.equal(service.status().starting, true);
    assert.equal((await request('/stop', {})).code, 200);
    holder.stdin.end('\n'); await once(holder, 'exit');
    const canceledStart = await pendingCapture;
    assert.equal(canceledStart.code, 409);
    assert.match(canceledStart.data.error, /启动已取消/);
    assert.equal(service.status().job, null);
});

test('calibration wizard and diagnostic details compile', () => {
    for (const name of ['CalibrationPanel', 'ReadinessPanel']) {
        const source = fs.readFileSync(new URL(`../src/components/${name}.vue`, import.meta.url), 'utf8');
        const { descriptor, errors } = parse(source); assert.deepEqual(errors, []);
        assert.ok(compileScript(descriptor, { id: name }).content);
        assert.deepEqual(compileTemplate({ source: descriptor.template.content, filename: `${name}.vue`, id: name }).errors, []);
    }
});

test('stopping an owned recording process is cancellation, not a worker error', async t => {
    const root = fs.mkdtempSync(path.join(os.tmpdir(), 'ldc-calibration-stop-'));
    const app = express(); app.use(express.json());
    // This process fixture exercises ownership and OS signals, without ROS or a robot.
    const service = installCalibrationRoutes({ app, dataRoot: root, simulation: true,
        repoRoot: fileURLToPath(new URL('../..', import.meta.url)),
        buildRosCommand: () => `/usr/bin/python3 -u -c 'import time; print("fixture ready",flush=True); time.sleep(30)'` });
    const server = http.createServer(app); server.listen(0, '127.0.0.1'); await once(server, 'listening');
    t.after(async () => { await service.close(); server.closeAllConnections(); await new Promise(r => server.close(r)); fs.rmSync(root, { recursive: true }); });
    const post = async (route, body) => {
        const response = await fetch(`http://127.0.0.1:${server.address().port}/api/calibration${route}`, {
            method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body) });
        return { code:response.status, data:await response.json() };
    };
    const id = (await post('/sessions', {name:'进程清理测试，非实车'})).data.id;
    assert.equal((await post(`/sessions/${id}/captures`, {seconds:30,maxMb:1})).data.running, true);
    assert.equal((await post(`/sessions/${id}/captures`, {seconds:30,maxMb:1})).code, 409);
    const stopped = (await post('/stop', {})).data.job;
    assert.equal(stopped.running, false);
    assert.equal(stopped.stoppedByUser, true);
    assert.equal(stopped.error, '');
    assert.equal((await post('/stop', {})).code, 200);
});

test('record and replay share startup/stop exclusion and shutdown cleans owned jobs', async t => {
    const root = fs.mkdtempSync(path.join(os.tmpdir(), 'ldc-calibration-race-'));
    const pidFile = path.join(root, 'fixture-pids');
    const app = express(); app.use(express.json());
    const quote = value => "'"+value.replaceAll("'", "'\\''")+"'";
    let launches = 0, holder, pending;
    const service = installCalibrationRoutes({ app, dataRoot: root, simulation: true,
        repoRoot: fileURLToPath(new URL('../..', import.meta.url)),
        buildRosCommand: () => {
            ++launches;
            const script = 'import os,signal,sys,time; signal.signal(signal.SIGTERM,lambda *_:(time.sleep(.15),sys.exit(0))); open(sys.argv[1],"a").write(str(os.getpid())+"\\n"); print(os.getpid(),flush=True); time.sleep(30)';
            return `exec /usr/bin/python3 -u -c ${quote(script)} ${quote(pidFile)}`;
        } });
    const server = http.createServer(app); server.listen(0, '127.0.0.1'); await once(server, 'listening');
    t.after(async () => {
        if (holder && holder.exitCode === null) { holder.stdin.end('\n'); await terminateProcess(holder); }
        if (pending) await pending;
        await service.close();
        // Also clean a process if the regression loses its job reference.
        if (fs.existsSync(pidFile)) for (const pid of fs.readFileSync(pidFile, 'utf8').trim().split('\n').map(Number)) await terminateProcess({ pid });
        server.closeAllConnections(); await new Promise(resolve => server.close(resolve)); fs.rmSync(root, { recursive: true });
    });
    const post = async (route, body = {}) => {
        const response = await fetch(`http://127.0.0.1:${server.address().port}/api/calibration${route}`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        return { code: response.status, data: await response.json() };
    };
    async function until(condition) {
        const deadline = Date.now()+3000;
        while (!condition() && Date.now()<deadline) await delay(10);
        assert.ok(condition(), 'fixture did not reach expected state');
    }
    const id = (await post('/sessions', { name: '并发回归，非实车' })).data.id;
    const folder = path.join(root, id, 'captures', 'previous'); fs.mkdirSync(folder);
    fs.writeFileSync(path.join(folder, 'recording.sqlite3'), 'route fixture; workers never read ROS data');
    const replay = `/sessions/${id}/captures/previous/replay`;
    const record = `/sessions/${id}/captures`;
    holder = spawn('/usr/bin/python3', ['-u', '-c',
        "import fcntl,sys; f=open(sys.argv[1],'a'); fcntl.flock(f,fcntl.LOCK_EX); print('ready',flush=True); sys.stdin.readline()", path.join(root, '.lock')]);
    await once(holder.stdout, 'data');
    pending = post(record, { seconds: 30, maxMb: 1 });
    await until(() => service.status().starting);
    assert.equal((await post(replay)).code, 409, 'replay must not overtake a preparing recording');
    assert.equal(launches, 0);
    await post('/stop');
    holder.stdin.end('\n'); await once(holder, 'exit');
    assert.equal((await pending).code, 409);
    assert.equal(launches, 0, 'Stop cancels preparation before any worker is spawned');

    assert.equal((await post(replay)).code, 200);
    await until(() => !!service.status().job.output.trim());
    const replayPid = Number(service.status().job.output.trim());
    assert.equal((await post(record, { seconds: 30, maxMb: 1 })).code, 409);
    assert.equal((await post(replay)).code, 409);
    const stopping = post('/stop');
    await until(() => service.status().stopping);
    assert.equal((await post(record, { seconds: 30, maxMb: 1 })).code, 409);
    assert.equal((await post(replay)).code, 409);
    assert.equal((await stopping).data.job.running, false);
    assert.throws(() => process.kill(replayPid, 0), { code: 'ESRCH' });
    assert.equal((await post(record, { seconds: 30, maxMb: 1 })).code, 200);
    await until(() => !!service.status().job.output.trim());
    const recordPid = Number(service.status().job.output.trim());
    await service.close();
    assert.throws(() => process.kill(recordPid, 0), { code: 'ESRCH' });
    assert.equal((await post(replay)).code, 409, 'closing service cannot spawn another job');
    assert.equal(launches, 2);
});
