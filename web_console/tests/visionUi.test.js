import { composeScript } from './helpers/composeScript.js';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, computed, watch, nextTick } from 'vue';

test('vision UI recovers backend state, clears old frames on connection failure and shows healthy empty results', async () => {
    let offline = false;
    let reply = { running: true, camera: { ok: true }, detector: { ok: true }, detections: [{ name: 'person', score: .8 }], filter: 'all' };
    const calls = [];
    const context = vm.createContext({ ref, computed, watch, nextTick, onMounted() {}, onUnmounted() {},
        defineProps: () => ({ defaultUrl: 'http://camera/' }), defineEmits: () => (() => {}),
        window: { location: { hostname: 'localhost' } }, AbortSignal, setTimeout, clearTimeout, setInterval, clearInterval,
        fetch: async (url, options) => { calls.push({ url, options }); if (offline) throw new Error('offline'); return { ok: true, json: async () => reply }; }
    });
    const script = composeScript({
        composables: [new URL('../src/composables/useCameraHealth.js', import.meta.url)],
        component: new URL('../src/components/CameraView.vue', import.meta.url)
    });
    vm.runInContext(script + '\nglobalThis.ui={state,poll,previewUrl,startVision,classFilter,previewKind,error}', context);
    const ui = context.ui;
    await ui.poll(); assert.equal(ui.state.value.running, true); assert.match(ui.previewUrl.value, /localhost:3000\/api\/vision\/annotated.mjpg/);
    ui.previewKind.value = 'raw'; await nextTick(); assert.match(ui.previewUrl.value, /raw/);
    reply = { ...reply, detections: [] };await ui.poll(); assert.equal(ui.state.value.detector.ok, true);assert.equal(ui.state.value.detections.length, 0);
    offline = true;await ui.poll();assert.equal(ui.previewUrl.value, '');assert.equal(ui.state.value.detections.length, 0);
    offline = false;await ui.poll();assert.equal(ui.state.value.running, true);
    await ui.startVision();assert.ok(calls.some(call => call.url.endsWith('/start') && JSON.parse(call.options.body).url === 'http://camera/'));
});
