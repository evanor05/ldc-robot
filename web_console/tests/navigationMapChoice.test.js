import { composeScript } from './helpers/composeScript.js';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import { ref, reactive, computed, watch, nextTick } from 'vue';

const bad = (name, path) => ({
    name, path, navigable: false, readable: true,
    quality: { occupied: 1717, unknownRatio: 0.861, total: 147471, resolution: 0.05 },
    qualityError: '地图内容不足：有效占用栅格 1717（最低 2500），未知占比 86.1%（上限 85.0%）'
});
const good = (name, path) => ({
    name, path, navigable: true, readable: true,
    quality: { occupied: 3348, unknownRatio: 0.764, total: 147471, resolution: 0.05 },
    qualityError: null
});

function mount() {
    const emitted = [];
    const props = reactive({
        connected: true, sessionId: 'session-1', controlOwner: null,
        controlVersion: () => 0, isSettingPose: false, multiPointMode: false,
        lastSavedMap: null, mappingActive: false
    });
    let mapReply = { maps: [], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } };
    let statusReply = { isRunning: false, mapPath: '' };
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {},
        defineProps: () => props,
        defineEmits: () => (...args) => emitted.push(args),
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
            json: async () => (url.includes('/list_maps') ? mapReply : statusReply)
        })
    });
const source = composeScript({
    composables: [new URL('../src/composables/useMapCatalog.js', import.meta.url)],
    component: new URL('../src/components/NavigationControl.vue', import.meta.url)
});
    vm.runInContext(source + '\nglobalThis.ui = { maps, selectedMap, mapChoiceTouched, thresholds, loadMaps, onMapChange, clearMapChoice, startBtnDisabled, startBtnTitle, selectedMapMissing, hasNavigableMap, mapQualityLabel, checkStatus };', context);
    return {
        ui: context.ui, emitted, props,
        setMaps: value => { mapReply = value; },
        setStatus: value => { statusReply = value; }
    };
}

test('默认只选第一张合格图，跳过排在前面的差图', async () => {
    const { ui, setMaps } = mount();
    setMaps({
        maps: [bad('map_20261111180316_294', '/maps/map_20261111180316_294'),
            good('map_20261111165629_640', '/maps/map_20261111165629_640')],
        thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 }
    });
    await ui.loadMaps();
    assert.equal(ui.selectedMap.value, '/maps/map_20261111165629_640');
    assert.equal(ui.mapChoiceTouched.value, false, '自动选择不算用户选择');
    assert.equal(ui.startBtnDisabled.value, false);
});

test('一张合格图都没有时不选任何图，并给出建图提示', async () => {
    const { ui, setMaps } = mount();
    setMaps({ maps: [bad('a', '/maps/a'), bad('b', '/maps/b')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    assert.equal(ui.selectedMap.value, '');
    assert.equal(ui.hasNavigableMap.value, false);
    assert.equal(ui.startBtnDisabled.value, true);
    assert.match(ui.startBtnTitle.value, /请先选择一张地图|暂无地图/);
});

test('用户手选后，状态轮询不得覆盖选择', async () => {
    const { ui, setMaps, setStatus } = mount();
    setMaps({ maps: [good('a', '/maps/a'), good('b', '/maps/b')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    // 用户改选 b：模拟界面 change 事件
    ui.selectedMap.value = '/maps/b';
    ui.onMapChange();
    assert.equal(ui.mapChoiceTouched.value, true);
    // 服务端说当前加载的是 a（导航中），轮询不得把选择改回 a
    setStatus({ isRunning: true, mapPath: '/maps/a' });
    await ui.checkStatus();
    assert.equal(ui.selectedMap.value, '/maps/b');
});

test('所选地图从列表消失：保留选择、禁止启动并提示重选（不静默改选）', async () => {
    const { ui, setMaps } = mount();
    setMaps({ maps: [good('a', '/maps/a'), good('b', '/maps/b')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    ui.selectedMap.value = '/maps/b';
    ui.onMapChange();
    setMaps({ maps: [good('a', '/maps/a')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    assert.equal(ui.selectedMap.value, '/maps/b', '不得自动改选别的地图');
    assert.equal(ui.selectedMapMissing.value, true);
    assert.equal(ui.startBtnDisabled.value, true);
    assert.match(ui.startBtnTitle.value, /已不可用/);
});

test('刷新后同名地图 path 变化时按 name 重新绑定', async () => {
    const { ui, setMaps } = mount();
    setMaps({ maps: [good('map_a', '/old/map_a')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    ui.onMapChange();
    setMaps({ maps: [good('map_a', '/new/map_a')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    assert.equal(ui.selectedMap.value, '/new/map_a');
    assert.equal(ui.selectedMapMissing.value, false);
});

test('清除选择是唯一复位 mapChoiceTouched 的操作，之后可自动重选', async () => {
    const { ui, setMaps } = mount();
    setMaps({ maps: [good('a', '/maps/a')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    ui.onMapChange();
    ui.clearMapChoice();
    assert.equal(ui.selectedMap.value, '');
    assert.equal(ui.mapChoiceTouched.value, false);
    await ui.loadMaps();
    assert.equal(ui.selectedMap.value, '/maps/a');
});

test('差图被选中时启动按钮禁用并说明原因', async () => {
    const { ui, setMaps } = mount();
    setMaps({ maps: [good('a', '/maps/a'), bad('c', '/maps/c')], thresholds: { minOccupiedCells: 2500, maxUnknownRatio: 0.85 } });
    await ui.loadMaps();
    ui.selectedMap.value = '/maps/c';
    ui.onMapChange();
    assert.equal(ui.startBtnDisabled.value, true);
    assert.match(ui.startBtnTitle.value, /不可导航/);
});

test('质量缺失时文案不出现 NaN 或伪造的 0 栅格', async () => {
    const { ui } = mount();
    // 老后端：没有 navigable / quality 字段
    assert.equal(ui.mapQualityLabel({ name: 'legacy' }), '质量未知');
    assert.doesNotMatch(ui.mapQualityLabel({ name: 'legacy' }), /NaN/);
    assert.match(ui.mapQualityLabel(good('g', '/g')), /^3348 栅格 · 未知 76\.4%$/);
    assert.match(ui.mapQualityLabel(bad('b', '/b')), /^不可导航：地图内容不足/);
});
