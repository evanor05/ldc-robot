// NavigationControl 的生命周期边界：卸载后到达的响应不得写状态，卸载发生在
// onMounted 的 await 之间时不得再创建轮询定时器（否则留下泄漏）。
// 先有测试再改生产代码：这两条在修复前应当是红的。
import assert from 'node:assert/strict';
import test from 'node:test';
import vm from 'node:vm';
import { ref, reactive, computed, watch, nextTick } from 'vue';
import { composeScript } from './helpers/composeScript.js';

function mount() {
    const props = reactive({
        connected: true, sessionId: 'session-1', controlOwner: null,
        controlVersion: () => 0, isSettingPose: false, multiPointMode: false,
        lastSavedMap: null, mappingActive: false, waypoints: [], activeWaypoint: null,
        waypointCount: 0
    });
    const mounted = [];
    const unmounted = [];
    const intervals = [];
    const listeners = { added: [], removed: [] };
    let statusReply = { isRunning: false, phase: 'stopped', mapPath: '', error: '' };
    let statusDeferred = null;
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted: callback => mounted.push(callback),
        onUnmounted: callback => unmounted.push(callback),
        defineProps: () => props,
        defineEmits: () => () => {},
        window: {
            location: { hostname: 'localhost' },
            addEventListener: (name, handler) => listeners.added.push({ name, handler }),
            removeEventListener: (name, handler) => listeners.removed.push({ name, handler }),
            dispatchEvent() {}
        },
        document: { addEventListener() {}, removeEventListener() {}, hidden: false },
        CustomEvent: class {},
        console,
        Date,
        setInterval: (callback, ms) => { const id = Symbol('timer'); intervals.push({ id, callback, ms }); return id; },
        clearInterval: () => {},
        setTimeout: () => Symbol('timeout'),
        clearTimeout: () => {},
        AbortSignal: { timeout: () => undefined },
        fetch: async url => {
            if (url.includes('/list_maps')) return { json: async () => ({ maps: [], thresholds: {} }) };
            if (statusDeferred) return { json: () => statusDeferred };
            return { json: async () => statusReply };
        }
    });
    const source = composeScript({
        composables: [new URL('../src/composables/useMapCatalog.js', import.meta.url)],
        component: new URL('../src/components/NavigationControl.vue', import.meta.url)
    });
    vm.runInContext(source + '\nglobalThis.ui = { navigationState, isNavigating, errorMsg, checkStatus, isSettingPose };', context);
    return {
        ui: context.ui,
        props,
        mounted,
        unmounted,
        intervals,
        listeners,
        setStatus: value => { statusReply = value; },
        deferStatus: () => {
            let resolve;
            const promise = new Promise(done => { resolve = done; });
            statusDeferred = promise;
            return value => { statusDeferred = null; resolve(value); };
        },
        flush: async () => { for (let i = 0; i < 20; i++) await Promise.resolve(); await nextTick(); }
    };
}

test('卸载后到达的状态响应不得写 navigationState（也不得清除错误）', async () => {
    const page = mount();
    const release = page.deferStatus();
    const pending = page.ui.checkStatus();
    // 响应还在路上时组件卸载
    for (const callback of page.unmounted) callback();
    release({ isRunning: true, phase: 'ready' });
    await pending;
    assert.equal(page.ui.navigationState.value.phase, undefined,
        '卸载后不得把响应写进 navigationState');
    assert.equal(page.ui.isNavigating.value, false, '卸载后不得改 isNavigating');
});

test('卸载发生在 onMounted 的 await 之间时不得创建轮询定时器', async () => {
    const page = mount();
    for (const callback of page.mounted) {
        const running = callback();
        // loadMaps/checkStatus 尚未完成就卸载
        for (const unmountCallback of page.unmounted) unmountCallback();
        if (running && typeof running.then === 'function') await running;
    }
    await page.flush();
    assert.equal(page.intervals.length, 0,
        `卸载后不得再 setInterval，实际创建了 ${page.intervals.length} 个定时器`);
});

test('卸载时必须移除 maps-updated 监听', async () => {
    const page = mount();
    for (const callback of page.mounted) {
        const running = callback();
        if (running && typeof running.then === 'function') await running;
    }
    const added = page.listeners.added.filter(item => item.name === 'maps-updated');
    assert.equal(added.length, 1, '挂载时应注册一次 maps-updated');
    for (const callback of page.unmounted) callback();
    const removed = page.listeners.removed.filter(item => item.name === 'maps-updated');
    assert.equal(removed.length, 1, '卸载时必须移除 maps-updated 监听');
});
