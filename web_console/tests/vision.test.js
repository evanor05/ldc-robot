import test from 'node:test';
import assert from 'node:assert/strict';
import express from 'express';
import http from 'node:http';
import { once } from 'node:events';
import { installVisionRoutes, validateVisionOptions } from '../visionRoutes.js';
import { readFile } from 'node:fs/promises';
import { parse, compileScript, compileTemplate } from '@vue/compiler-sfc';

async function listen(server) { server.listen(0, '127.0.0.1'); await once(server, 'listening'); return server.address().port; }
test('vision rejects invalid devices, filters, credentials and nonlocal simulation URLs', () => {
    for (const body of [{ device: 'cuda' }, { filter: 'stop' }, { url: 'file:///etc/passwd' }, { url: 'http://user:secret@localhost/' }]) {
        assert.throws(() => validateVisionOptions(body));
    }
    assert.throws(() => validateVisionOptions({ url: 'http://192.168.1.20/' }, true));
    assert.throws(() => validateVisionOptions({}, true));
    assert.equal(validateVisionOptions({ url: 'http://127.0.0.1:1234/stream', device: 'cpu' }, true).device, 'cpu');
});
test('vision proxy restores actual state, preserves healthy empty lists and supports CLI shutdown', async t => {
    let running = true, filter = 'all';
    const gateway = http.createServer((req, res) => {
        if (!running) { res.statusCode = 503; res.end('{}'); return; }
        res.setHeader('Content-Type', 'application/json');
        if (req.url === '/status') res.end(JSON.stringify({ service: 'ldc_vision', running, camera: { ok: true }, detector: { ok: true }, detections: [], filter }));
        else {
            let body = ''; req.on('data', chunk => body += chunk);
            req.on('end', () => { if (req.url === '/filter') filter = JSON.parse(body).mode; else if (req.url === '/shutdown') running = false; res.end('{"success":true}'); });
        }
    });
    const gatewayPort = await listen(gateway);
    const app = express(); app.use(express.json());
    const vision = installVisionRoutes({ app, repoRoot: '/unused', simulation: true, gatewayPort });
    const server = http.createServer(app); const port = await listen(server);
    t.after(async () => { await vision.close(); server.closeAllConnections(); gateway.closeAllConnections(); await Promise.all([new Promise(r => server.close(r)),new Promise(r => gateway.close(r))]); });
    const request = async (endpoint, body) => {
        const r = await fetch(`http://127.0.0.1:${port}/api/vision/${endpoint}`, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
        return [r.status, await r.json()];
    };
    const [code, status] = await request('status');assert.equal(code, 200);assert.equal(status.running, true);assert.deepEqual(status.detections, []);
    assert.equal((await request('start', { url: 'http://127.0.0.1/stream' }))[1].running, true);
    assert.equal((await request('filter', { filter: 'traffic' }))[0], 200);
    assert.equal((await request('status'))[1].filter, 'traffic');
    assert.equal((await request('filter', { filter: 'bad' }))[0], 400);
    assert.equal((await request('stop', {}))[1].success, true);
    const offline = (await request('status'))[1]; assert.equal(offline.running, false);assert.deepEqual(offline.detections, []);assert.equal(offline.camera.ok, false);
});
test('camera component compiles with gateway-only preview and explicit traffic-light limitation', async () => {
    const source = await readFile(new URL('../src/components/CameraView.vue', import.meta.url), 'utf8');
    const { descriptor, errors } = parse(source);assert.deepEqual(errors, []);
    assert.ok(compileScript(descriptor, { id: 'camera-test' }).content);
    const template = compileTemplate({ source: descriptor.template.content, filename: 'CameraView.vue', id: 'camera-test' });
    assert.deepEqual(template.errors, []);
    assert.match(source, /灯色未判定/);assert.match(source, /api\/vision/);
    assert.doesNotMatch(source, /:src="(?:draftUrl|activeUrl|defaultUrl)"/);
});
