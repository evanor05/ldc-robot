// E：相机 running 与 healthy 必须分开；raw / annotated 各自独立看门狗。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import { composeScript } from './helpers/composeScript.js';
import { ref, computed, reactive, watch, nextTick, onMounted, onUnmounted } from 'vue';

function mount() {
    const props = reactive({ defaultUrl: '', connected: true });
    let status = { running: false, detections: [] };
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick, onMounted, onUnmounted,
        defineProps: () => props,
        defineEmits: () => () => {},
        window: { location: { hostname: 'localhost' }, addEventListener() {}, removeEventListener() {} },
        document: { hidden: false, addEventListener() {}, removeEventListener() {} },
        console,
        // 让沙箱与测试共享同一个 Date，便于精确控制"离线了多久"
        Date,
        setInterval: () => Symbol('timer'),
        clearInterval() {},
        setTimeout: () => Symbol('timeout'),
        clearTimeout() {},
        AbortSignal: { timeout: () => undefined },
        fetch: async () => ({ ok: true, json: async () => status })
    });
const source = composeScript({
    composables: [new URL('../src/composables/useCameraHealth.js', import.meta.url)],
    component: new URL('../src/components/CameraView.vue', import.meta.url)
});
    vm.runInContext(source + '\nglobalThis.ui = { state, managed, cameraStatusText, cameraStatusClass, frameWatch, imageKey, previewKind, offlineSeconds, poll };', context);
    return { ui: context.ui, setStatus: value => { status = value; } };
}

test('running 但没有画面时显示离线与重连，而不是"健康"', async () => {
    const { ui, setStatus } = mount();
    setStatus({ running: true, camera: { ok: false, detail: '无实时画面' }, detector: { ok: false }, detections: [] });
    await ui.poll();
    assert.equal(ui.cameraStatusClass.value, 'is-off');
    assert.match(ui.cameraStatusText.value, /摄像头离线/);
    assert.match(ui.cameraStatusText.value, /重连中/);
    assert.doesNotMatch(ui.cameraStatusText.value, /正常/);
});

test('画面正常时才显示健康', async () => {
    const { ui, setStatus } = mount();
    setStatus({ running: true, camera: { ok: true }, detector: { ok: true }, detections: [] });
    await ui.poll();
    assert.equal(ui.cameraStatusClass.value, 'is-ok');
    assert.equal(ui.cameraStatusText.value, '实时画面正常');
});

test('未启动时显示未连接详情，不误报离线', async () => {
    const { ui, setStatus } = mount();
    setStatus({ running: false, camera: { ok: false, detail: '未连接' }, detections: [] });
    await ui.poll();
    assert.equal(ui.cameraStatusText.value, '未连接');
    assert.equal(ui.offlineSeconds.value, 0);
});

test('离线计时随轮询累加，用于重连倒计时文案', async () => {
    const { ui, setStatus } = mount();
    setStatus({ running: true, camera: { ok: false }, detector: { ok: false }, detections: [] });
    await ui.poll();
    const first = ui.offlineSeconds.value;
    assert.ok(first >= 1, `首次离线应至少显示 1s，实际 ${first}`);
    // 人为把离线起点往前挪，模拟已经离线一段时间
    const realNow = Date.now;
    Date.now = () => realNow() + 5000;
    try {
        await ui.poll();
    } finally {
        Date.now = realNow;
    }
    assert.ok(ui.offlineSeconds.value >= 5, `应累加到 5s 以上，实际 ${ui.offlineSeconds.value}`);
});

test('raw 卡住而 annotated 前进时，raw 自己触发重连', async () => {
    const { ui, setStatus } = mount();
    ui.previewKind.value = 'raw';
    setStatus({ running: true, camera: { ok: true }, detector: { ok: true },
        previewFrames: { raw: 10, annotated: 100 }, detections: [] });
    await ui.poll();
    const before = ui.imageKey.value;
    // raw 帧号不动，但 annotated 继续涨：旧实现会因共用身份而看不出 raw 卡死
    ui.frameWatch.raw.at = Date.now() - 4000;
    setStatus({ running: true, camera: { ok: true }, detector: { ok: true },
        previewFrames: { raw: 10, annotated: 500 }, detections: [] });
    await ui.poll();
    assert.equal(ui.imageKey.value, before + 1, 'raw 卡住超过 3s 必须重建连接');
});

test('raw 正常前进时不误判卡死（与 annotated 独立）', async () => {
    const { ui, setStatus } = mount();
    ui.previewKind.value = 'raw';
    setStatus({ running: true, camera: { ok: true }, detector: { ok: true },
        previewFrames: { raw: 10, annotated: 100 }, detections: [] });
    await ui.poll();
    const before = ui.imageKey.value;
    ui.frameWatch.raw.at = Date.now() - 4000;
    setStatus({ running: true, camera: { ok: true }, detector: { ok: true },
        previewFrames: { raw: 11, annotated: 100 }, detections: [] });
    await ui.poll();
    assert.equal(ui.imageKey.value, before, 'raw 有新帧就不该重建连接');
});
