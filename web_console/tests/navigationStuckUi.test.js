import { composeScript } from './helpers/composeScript.js';
// D 的前端可见性：无位移锁存必须在面板上说清原因与解除方式。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import { ref, reactive, computed, watch, nextTick } from 'vue';

function mount() {
    const props = reactive({
        connected: true, sessionId: 'session-1', controlOwner: null,
        controlVersion: () => 0, isSettingPose: false, multiPointMode: false,
        lastSavedMap: null, mappingActive: false
    });
    let status = { isRunning: true, mapPath: '', phase: 'failed', error: '' };
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {},
        defineProps: () => props,
        defineEmits: () => () => {},
        window: {
            location: { hostname: 'localhost' },
            addEventListener() {}, removeEventListener() {}, dispatchEvent() {}
        },
        document: { addEventListener() {}, removeEventListener() {} },
        CustomEvent: class {},
        console,
        setInterval: () => Symbol('timer'),
        clearInterval() {},
        fetch: async url => ({
            json: async () => (url.includes('/list_maps') ? { maps: [] } : status)
        })
    });
const source = composeScript({
    composables: [new URL('../src/composables/useMapCatalog.js', import.meta.url)],
    component: new URL('../src/components/NavigationControl.vue', import.meta.url)
});
    vm.runInContext(source + '\nglobalThis.ui = { stuckLatched, stuckReason, checkStatus, errorMsg };', context);
    return { ui: context.ui, setStatus: value => { status = value; } };
}

test('锁存后提示原因与解除方式，而不是只显示"导航失败"', async () => {
    const { ui, setStatus } = mount();
    setStatus({
        isRunning: true, phase: 'failed',
        stuckPolicy: {
            latched: true, consecutive: 2,
            latchReason: '连续 2 个目标无位移失败（最近一次位移 0.02m / 5.0s）：车辆可能贴墙或在膨胀区内'
        }
    });
    await ui.checkStatus();
    assert.equal(ui.stuckLatched.value, true);
    assert.match(ui.stuckReason.value, /连续 2 个目标无位移失败/);
});

test('未锁存时不显示锁存提示', async () => {
    const { ui, setStatus } = mount();
    setStatus({ isRunning: true, phase: 'ready', stuckPolicy: { latched: false, consecutive: 0 } });
    await ui.checkStatus();
    assert.equal(ui.stuckLatched.value, false);
});

test('老后端没有 stuckPolicy 字段时按未锁存处理，不报错', async () => {
    const { ui, setStatus } = mount();
    setStatus({ isRunning: true, phase: 'ready' });
    await ui.checkStatus();
    assert.equal(ui.stuckLatched.value, false);
});

// E：就绪面板必须在"开始导航"之前就能看到 blocker，pollFailed 也要可响应。
test('就绪面板启动前可见，且 pollFailed 是可响应的 ref', () => {
    const source = fs.readFileSync(new URL('../src/components/NavigationControl.vue', import.meta.url), 'utf8');
    assert.doesNotMatch(source, /<ReadinessPanel\s+v-if/, '就绪面板不能被 isNavigating 门控');
    assert.match(source, /<ReadinessPanel\s+:status="navigationState"/);
    assert.match(source, /const pollFailed = ref\(false\)/, 'pollFailed 必须是 ref 才能驱动渲染');
    assert.doesNotMatch(source, /^\s*let pollFailed = false/m);
});
