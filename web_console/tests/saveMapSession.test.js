import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, reactive, computed, watch, nextTick } from 'vue';
import * as coordinates from '../src/lib/mapCoordinates.js';
import * as tf from '../src/lib/tfTransforms.js';

function mountSlamMap({ sessionId = 'session-1' } = {}) {
    const requests = [];
    const props = reactive({ sessionId, connected: true, isMapping: true, layers: {} });
    const context = vm.createContext({
        ...coordinates, ...tf, ref, reactive, computed, watch, nextTick,
        defineProps: () => props,
        defineEmits: () => () => {},
        defineExpose: () => {},
        onMounted() {}, onUnmounted() {},
        window: { location: { hostname: 'localhost' }, dispatchEvent() {} },
        CustomEvent: class {},
        document: { createElement: () => ({ getContext: () => ({}) }) },
        requestAnimationFrame() {},
        setTimeout: () => 0, clearTimeout() {},
        ResizeObserver: class { observe() {} disconnect() {} },
        console,
        fetch: async (url, options) => {
            requests.push({ url, body: options?.body ? JSON.parse(options.body) : null });
            return { json: async () => ({ success: true, name: 'room_map' }) };
        }
    });
    const source = fs.readFileSync(new URL('../src/components/SlamMap.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + '\nglobalThis.ui = { confirmSave, mapNameInput, saveMsg, saveSuccess, isSaving };', context);
    return { ...context.ui, requests, props };
}

test('save map sends the control session and an idempotency request id', async () => {
    const ui = mountSlamMap();
    ui.mapNameInput.value = 'room_map';
    await ui.confirmSave();
    assert.equal(ui.requests.length, 1);
    assert.match(ui.requests[0].url, /\/api\/save_map$/);
    assert.equal(ui.requests[0].body.sessionId, 'session-1');
    assert.equal(ui.requests[0].body.mapName, 'room_map');
    assert.ok(ui.requests[0].body.requestId, '保存请求必须携带 requestId');
    assert.equal(ui.isSaving.value, false);
});

test('save map refuses to call the backend without a control session', async () => {
    const ui = mountSlamMap({ sessionId: '' });
    ui.mapNameInput.value = 'room_map';
    await ui.confirmSave();
    assert.equal(ui.requests.length, 0, '缺少会话时不得发请求');
    assert.match(ui.saveMsg.value, /控制会话/);
    assert.equal(ui.saveSuccess.value, false);
});
