import { composeScript } from './helpers/composeScript.js';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, reactive, computed, watch, nextTick } from 'vue';

function mount() {
    const requests = [];
    const emitted = [];
    const props = reactive({ connected: true, sessionId: 'session-1', canControl: true });
    let statusReply = { isRunning: false, mode: 'manual', lastSavedMap: null };
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {},
        defineProps: () => props,
        defineEmits: () => (...args) => emitted.push(args),
        window: { location: { hostname: 'localhost' }, dispatchEvent() {} },
        CustomEvent: class {},
        console,
        setInterval: () => Symbol('timer'), clearInterval() {},
        fetch: async (url, options) => {
            requests.push({ url, body: options?.body ? JSON.parse(options.body) : null });
            return { json: async () => url.includes('mapping_status') ? statusReply : { success: true } };
        }
    });
    const source = composeScript({
    composables: [new URL('../src/composables/useMappingTask.js', import.meta.url)],
    component: new URL('../src/components/MappingControl.vue', import.meta.url)
});
    vm.runInContext(source + '\nglobalThis.ui = { startMapping, checkStatus, selectedMode, activeMode, isMapping };', context);
    return { ui: context.ui, requests, emitted, setStatus: value => { statusReply = value; } };
}

test('manual/auto mapping selection is sent to the backend', async () => {
    const { ui, requests, emitted } = mount();
    assert.equal(ui.selectedMode.value, 'manual');
    ui.selectedMode.value = 'auto';
    await ui.startMapping();
    const start = requests.find(item => item.url.endsWith('/start_mapping'));
    assert.equal(start.body.mode, 'auto');
    assert.equal(start.body.sessionId, 'session-1');
    assert.equal(ui.activeMode.value, 'auto');
    assert.equal(ui.isMapping.value, true);
    assert.ok(emitted.some(([name]) => name === 'mappingStarted'));
});

test('running mapping reads server mode, stopped status keeps the user selection', async () => {
    const { ui, setStatus } = mount();
    ui.selectedMode.value = 'auto';
    setStatus({ isRunning: true, mode: 'manual', lastSavedMap: null });
    await ui.checkStatus();
    assert.equal(ui.activeMode.value, 'manual', '建图中必须显示服务端实际模式');
    assert.equal(ui.selectedMode.value, 'manual');
    ui.selectedMode.value = 'auto';
    setStatus({ isRunning: false, mode: 'manual', lastSavedMap: null });
    await ui.checkStatus();
    assert.equal(ui.activeMode.value, '');
    assert.equal(ui.selectedMode.value, 'auto', '停止后保留用户选择');
});
