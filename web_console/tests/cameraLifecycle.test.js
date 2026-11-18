// E：相机故障、独立看门狗、冷却防抖、慢请求防并发与组件卸载生命周期退化行为测试
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import { composeScript } from './helpers/composeScript.js';
import { ref, computed, reactive, watch, nextTick } from 'vue';

function mount(options = {}) {
    const props = reactive({ defaultUrl: '', connected: true });
    let status = { running: false, detections: [] };
    let fetchCount = 0;
    let customFetch = null;
    const mountedCallbacks = [];
    const unmountedCallbacks = [];
    const timeouts = [];
    const intervals = [];

    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted: cb => { mountedCallbacks.push(cb); },
        onUnmounted: cb => { unmountedCallbacks.push(cb); },
        defineProps: () => props,
        defineEmits: () => () => {},
        window: { location: { hostname: 'localhost' }, addEventListener() {}, removeEventListener() {} },
        document: { hidden: false, addEventListener() {}, removeEventListener() {} },
        console,
        // 沙箱与外部共享同一个 Date，便于模拟时间流逝与冷却窗口
        Date,
        setInterval: (cb, ms) => {
            const id = Symbol('interval');
            intervals.push({ id, cb, ms, cleared: false });
            return id;
        },
        clearInterval: id => {
            const item = intervals.find(t => t.id === id);
            if (item) item.cleared = true;
        },
        setTimeout: (cb, ms) => {
            const id = Symbol('timeout');
            timeouts.push({ id, cb, ms, cleared: false });
            return id;
        },
        clearTimeout: id => {
            const item = timeouts.find(t => t.id === id);
            if (item) item.cleared = true;
        },
        AbortSignal: { timeout: () => undefined },
        fetch: async (...args) => {
            fetchCount++;
            if (customFetch) return customFetch(...args);
            return { ok: true, json: async () => status };
        }
    });

const source = composeScript({
    composables: [new URL('../src/composables/useCameraHealth.js', import.meta.url)],
    component: new URL('../src/components/CameraView.vue', import.meta.url)
});

    vm.runInContext(
        source + '\nglobalThis.ui = { state, managed, cameraStatusText, cameraStatusClass, frameWatch, imageKey, previewKind, previewUrl, offlineSeconds, poll, streamFailed, reloadStream };',
        context
    );

    return {
        ui: context.ui,
        setStatus: value => { status = value; },
        setFetch: fn => { customFetch = fn; },
        getFetchCount: () => fetchCount,
        triggerMounted: () => mountedCallbacks.forEach(cb => cb()),
        triggerUnmounted: () => unmountedCallbacks.forEach(cb => cb()),
        timeouts,
        intervals
    };
}

test('1. annotated 卡住、raw 仍有新帧 -> annotated 一路触发重连（当前预览切到 annotated 时）', async () => {
    const { ui, setStatus } = mount();
    ui.previewKind.value = 'annotated';
    setStatus({
        running: true,
        camera: { ok: true },
        detector: { ok: true },
        previewFrames: { raw: 10, annotated: 100 },
        completed: 50,
        frame: { id: 50 },
        detections: []
    });
    await ui.poll();
    const before = ui.imageKey.value;

    // 模拟 annotated 已超过 FRAME_STALL_MS (3000ms) 未更新，而 raw 仍在前进
    ui.frameWatch.annotated.at = Date.now() - 4000;
    setStatus({
        running: true,
        camera: { ok: true },
        detector: { ok: true },
        previewFrames: { raw: 999, annotated: 100 },
        completed: 50,
        frame: { id: 50 },
        detections: []
    });
    await ui.poll();

    assert.equal(ui.imageKey.value, before + 1, 'annotated 卡住超过 3s 时必须主动递增 imageKey 重建连接，不能被 raw 的前进掩盖');
});

test('2. 两路都有新帧 -> 零误重连（连续多次 poll 都不重建连接）', async () => {
    const { ui, setStatus } = mount();
    ui.previewKind.value = 'annotated';

    const realNow = Date.now;
    let fakeNow = realNow();
    Date.now = () => fakeNow;
    try {
        for (let step = 1; step <= 6; step++) {
            fakeNow += 2000; // 每次推进 2s，累计 12s（远超 3s 卡死阈值与 5s 冷却窗口）
            setStatus({
                running: true,
                camera: { ok: true },
                detector: { ok: true },
                previewFrames: { raw: 10 + step, annotated: 100 + step },
                completed: 50 + step,
                frame: { id: 50 + step },
                detections: []
            });
            await ui.poll();
        }
    } finally {
        Date.now = realNow;
    }

    assert.equal(ui.imageKey.value, 0, '两路持续有新帧时，连续多次 poll 不应触发任何重连');
});

test('3. 冷却窗口（AUTO_RELOAD_MIN_MS=5000）内重复 poll -> 至多重连一次', async () => {
    const { ui, setStatus } = mount();
    ui.previewKind.value = 'annotated';

    const realNow = Date.now;
    let fakeNow = 1_000_000;
    Date.now = () => fakeNow;
    try {
        // 初始正常帧
        setStatus({
            running: true,
            camera: { ok: true },
            detector: { ok: true },
            previewFrames: { raw: 10, annotated: 100 },
            completed: 1,
            frame: { id: 1 },
            detections: []
        });
        await ui.poll();
        const initialKey = ui.imageKey.value;

        // 推进 3500ms（超过 FRAME_STALL_MS=3000），帧号不变 -> 触发第一次重连
        fakeNow += 3500;
        await ui.poll();
        assert.equal(ui.imageKey.value, initialKey + 1, '首次卡住超过 3s 应触发重连');

        // 在 5000ms 冷却窗口内，即使人为让 watch.at 再次老化且连续多次 poll，也不得再次重连
        ui.frameWatch.annotated.at = fakeNow - 4000;
        fakeNow += 1000; // 距上次重连仅 1000ms (< 5000ms)
        await ui.poll();
        ui.frameWatch.annotated.at = fakeNow - 4000;
        fakeNow += 2000; // 距上次重连 3000ms (< 5000ms)
        await ui.poll();
        assert.equal(ui.imageKey.value, initialKey + 1, '冷却窗口（5000ms）内重复 poll 至多重连一次');

        // 超过 5000ms 冷却窗口且再次卡住超过 3000ms -> 允许下一次重连
        fakeNow += 2500; // 距上次重连 5500ms (> 5000ms)
        ui.frameWatch.annotated.at = fakeNow - 3500;
        await ui.poll();
        assert.equal(ui.imageKey.value, initialKey + 2, '冷却窗口过后若仍卡住则允许再次重连');
    } finally {
        Date.now = realNow;
    }
});

test('4. 慢请求无并发：poll 未返回时再次 poll 不应发出第二个 status 请求', async () => {
    const { ui, setFetch, getFetchCount } = mount();
    let resolvePending;
    setFetch(() => new Promise(resolve => {
        resolvePending = () => resolve({
            ok: true,
            json: async () => ({ running: true, camera: { ok: true }, detector: { ok: true }, detections: [] })
        });
    }));

    // 发起第一个慢 poll（尚未 resolve）
    const p1 = ui.poll();
    assert.equal(getFetchCount(), 1, '首次 poll 应发出 1 次请求');

    // 在第一个请求未返回期间连续调用 poll
    const p2 = ui.poll();
    const p3 = ui.poll();
    assert.equal(getFetchCount(), 1, '前序 poll 未返回时，并发调用不应发出第二个 status 请求');

    // 释放第一个请求
    resolvePending();
    await Promise.all([p1, p2, p3]);
    assert.equal(ui.state.value.running, true);

    // 前序完成后，下一次 poll 应能正常发出
    const p4 = ui.poll();
    resolvePending();
    await p4;
    assert.equal(getFetchCount(), 2, '前序请求结束后，新一次 poll 应正常发出');
});

test('5. 卸载后成功响应回来 -> 无副作用（不再改 state / 不再触发重连 / 不解引用已释放状态）', async () => {
    const { ui, setFetch, getFetchCount, triggerUnmounted, timeouts } = mount();
    let resolvePending;
    setFetch(() => new Promise(resolve => {
        resolvePending = () => resolve({
            ok: true,
            json: async () => ({
                running: true,
                camera: { ok: true },
                detector: { ok: true },
                filter: 'traffic',
                device: 'cpu',
                previewFrames: { raw: 1, annotated: 1 },
                detections: [{ name: 'person', score: 0.9 }]
            })
        });
    }));

    // 先触发一次 streamFailed 注册 retryTimer，并把 frameWatch 拨到超时状态以诱发重连条件
    ui.streamFailed();
    ui.frameWatch.annotated.token = 'pf:1|c:|f:';
    ui.frameWatch.annotated.at = Date.now() - 10000;

    const initialStateSnapshot = JSON.parse(JSON.stringify(ui.state.value));
    const initialKey = ui.imageKey.value;

    // 发出请求但尚未返回
    const pendingPoll = ui.poll();
    assert.equal(getFetchCount(), 1);

    // 请求在途期间组件卸载
    triggerUnmounted();

    // 卸载后成功响应回来
    resolvePending();
    await pendingPoll;

    assert.equal(JSON.stringify(ui.state.value), JSON.stringify(initialStateSnapshot), '卸载后返回的成功响应不得改写 state');
    assert.equal(ui.imageKey.value, initialKey, '卸载后返回的成功响应不得触发重连递增 imageKey');
    assert.equal(ui.offlineSeconds.value, 0, '卸载后不得更新离线计时');

    // 即使之前调度的 retryTimer 回调被执行，也因 disposed 保护而不递增 imageKey
    for (const t of timeouts) {
        if (!t.cleared) t.cb();
    }
    assert.equal(ui.imageKey.value, initialKey, '卸载后定时器回调不得再触发重连');

    // 卸载后再次调用 poll 应直接短路，不发网络请求
    await ui.poll();
    assert.equal(getFetchCount(), 1, '卸载后再次 poll 不得发出请求');
});

test('6. 卸载后失败响应回来 -> 不触发重连、不产生未捕获异常、不改写已卸载组件的 state', async () => {
    const { ui, setFetch, triggerUnmounted } = mount();
    let rejectPending;
    setFetch(() => new Promise((_, reject) => {
        rejectPending = () => reject(new Error('network boom after unmount'));
    }));

    const initialStateSnapshot = JSON.parse(JSON.stringify(ui.state.value));
    const initialKey = ui.imageKey.value;

    // 请求在途
    const pendingPoll = ui.poll();
    // 组件卸载
    triggerUnmounted();

    // 失败响应回来：不得抛未捕获异常，也不得触发重连
    rejectPending();
    await assert.doesNotReject(pendingPoll, '卸载后失败响应返回不应产生未捕获异常');
    assert.equal(ui.imageKey.value, initialKey, '卸载后失败响应不得触发重连');
    // 生产代码已补 disposed 守卫：卸载后失败响应不得写 state
    assert.equal(JSON.stringify(ui.state.value), JSON.stringify(initialStateSnapshot),
        '卸载后失败响应不得改写 state（catch 分支必须有 disposed 守卫）');
});
