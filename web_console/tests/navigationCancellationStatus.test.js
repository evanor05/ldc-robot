import assert from 'node:assert/strict';
import fs from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

const source = fs.readFileSync(new URL('../navigationRoutes.js', import.meta.url), 'utf8');
const begin = source.indexOf("app.get('/api/navigation_status'");
const end = source.indexOf("app.get('/api/task_status/navigation'", begin);
assert.ok(begin > 0 && end > begin);

async function statusOf(status, stopping = false) {
    let route, payload;
    vm.runInNewContext(source.slice(begin, end), {
        app: { get(_path, handler) { route = handler; } }, wrap: handler => handler,
        bridge: { async request() { return status; } }, running: () => true,
        stopping, starting: false, loadedMap: '', configuration: null, processError: ''
    });
    await route({}, { json(data) { payload = data; } });
    return payload;
}

test('remote cancellation uncertainty is not mislabeled as a bridge restart', async () => {
    const unresolved = await statusOf({ isRunning:false, ready:false, cancelConfirmed:false,
        phase:'cancel_unconfirmed', error:'远端目标终态未确认' });
    assert.equal(unresolved.phase, 'cancel_unconfirmed');
    assert.equal(unresolved.error, '远端目标终态未确认');
    assert.equal(unresolved.ready, false);
    const stopping = await statusOf({ isRunning:false, phase:'canceling', error:'' }, true);
    assert.equal(stopping.phase, 'canceling');
    const restarted = await statusOf({ isRunning:false, phase:'stopped', cancelConfirmed:true });
    assert.equal(restarted.phase, 'failed');
    assert.match(restarted.error, /重启/);
});
