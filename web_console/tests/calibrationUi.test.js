import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, computed, watch, nextTick } from 'vue';

function wizard() {
    const requests = [];
    const context = vm.createContext({ ref, computed, watch, nextTick, onMounted() {}, onUnmounted() {}, defineEmits: () => () => {},
        window: { location: { hostname: 'localhost', hash: '' } }, AbortSignal, console,
        fetch: async (url, options = {}) => {
            if (options.body) requests.push(JSON.parse(options.body));
            const data = options.method === 'POST' ? { id: 'validation-session' } : url.endsWith('/sessions')
                ? { sessions: [], simulation: true, job: null }
                : { id: 'validation-session', dimensions: {}, schemas: {}, reports: [], captures: [] };
            return { ok: true, json: async () => data };
        } });
    const source = fs.readFileSync(new URL('../src/components/CalibrationPanel.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '').replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + `\nglobalThis.ui = { createValidation, loadSession, selectedId, session, result, error, open, dialogKeydown };`, context);
    const ui = context.ui;
    ui.selectedId.value = 'parent-session';
    ui.session.value = { id: 'parent-session', reports: [
        { sessionId: 'parent-session', reportId: 'candidate_A', applied: false, status: 'candidate_only' },
        { sessionId: 'parent-session', reportId: 'candidate_B', applied: false, status: 'candidate_only' }
    ] };
    return { ui, requests };
}

test('independent validation references the displayed historical report, not the newest', async () => {
    for (const index of [0, 1]) {
        const { ui, requests } = wizard();
        const selected = ui.session.value.reports[index];
        ui.result.value = selected;
        await ui.createValidation();
        assert.equal(requests[0].validationOf.reportId, selected.reportId);
        assert.equal(requests[0].validationOf.sessionId, selected.sessionId);
    }
});

test('validation rejects absent, non-candidate and other-session displayed reports', async () => {
    for (const selection of [null, { snapshotCount: 1, snapshots: [] },
        { sessionId: 'parent-session', reportId: 'empty', applied: false, status: 'awaiting_measurements' },
        { sessionId: 'other-session', reportId: 'candidate_A', applied: false, status: 'candidate_only' }]) {
        const { ui, requests } = wizard(); ui.result.value = selection;
        await ui.createValidation();
        assert.equal(requests.length, 0);
        assert.match(ui.error.value, /选择.*候选报告/);
    }
});

test('calibration dialog consumes Escape instead of triggering the global emergency stop', () => {
    const { ui } = wizard();
    ui.open.value = true;
    let stopped = false, prevented = false;
    ui.dialogKeydown({
        key: 'Escape',
        stopPropagation() { stopped = true; },
        preventDefault() { prevented = true; }
    });
    assert.equal(ui.open.value, false, 'Esc 必须关闭向导自身');
    assert.equal(stopped, true, 'Esc 不得冒泡到全局急停');
    assert.equal(prevented, true);
});

test('session change clears the report so it cannot be validated against another session', async () => {
    const { ui, requests } = wizard();
    ui.result.value = ui.session.value.reports[0];
    ui.selectedId.value = 'validation-session';
    await ui.loadSession();
    assert.equal(ui.result.value, null);
    await ui.createValidation();
    assert.equal(requests.length, 0);
});
