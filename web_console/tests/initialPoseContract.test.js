// 「设点模式的关闭权」契约：初始位姿必须等后端 ack 才退出设点模式。
//
// 静态矛盾（复现任一）：SlamMap.vue 在 emit('initial-pose-set') 之后紧跟着
// emit('update:isSettingPose', false)，而 App.vue 用 v-model:isSettingPose 绑定 ——
// 子组件的提前 emit 会直接改父级 ref，使 App 里"等 ack 再关"的代码形同虚设。
// 用户可见后果：HTTP 还没返回，界面已经退出设点模式并提示成功。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';
import { ref, computed, reactive, watch, nextTick } from 'vue';
import { composeScript } from './helpers/composeScript.js';

const SLAM_MAP = new URL('../src/components/SlamMap.vue', import.meta.url);

test('SlamMap 提交初始位姿后不得自行关闭设点模式（关闭权归 App，等 ack）', () => {
    const source = fs.readFileSync(SLAM_MAP, 'utf8');
    const lines = source.split('\n');
    const commit = lines.findIndex(line => line.includes("emit('initial-pose-set'"));
    assert.ok(commit >= 0, '找不到 initial-pose-set 的提交点');
    // 提交点之后若干行内不允许出现自行关闭设点模式的 emit
    const window = lines.slice(commit, commit + 6).join('\n');
    assert.doesNotMatch(window, /emit\('update:isSettingPose',\s*false\)/,
        'SlamMap 不得在提交后自行退出设点模式：App 必须等 HTTP ack 才关，'
        + '否则失败时用户已退出设点模式、无法原地重试');
});

test('App：ack 未返回前保持设点模式，确认后才退出；失败则保留', async () => {
    const props = reactive({
        connected: true, sessionId: 'session-1', controlOwner: null,
        controlVersion: () => 0, isSettingPose: true, multiPointMode: false,
        lastSavedMap: null, mappingActive: false, waypoints: [], activeWaypoint: null,
        waypointCount: 0
    });
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {},
        defineProps: () => props,
        defineEmits: () => () => {},
        useFoxglove: () => ({ connectionState: ref('connected'), errorMessage: ref(''),
            connect() {}, disconnect() {}, subscribe: () => 1 }),
        window: {
            location: { hostname: 'localhost' },
            addEventListener() {}, removeEventListener() {}, dispatchEvent() {}, confirm: () => true
        },
        document: { hidden: false, addEventListener() {}, removeEventListener() {} },
        sessionStorage: { getItem: () => null, setItem() {} },
        console,
        Date,
        setInterval: () => Symbol('timer'), clearInterval() {},
        setTimeout: () => Symbol('timeout'), clearTimeout() {},
        CustomEvent: class {},
        fetch: async (url, options) => {
            if (String(url).endsWith('navigation/initial_pose')) {
                return new Promise(resolve => {
                    // 由测试决定何时 ack
                    context.__ack = () => resolve({ ok: true, json: async () => ({ success: true }) });
                });
            }
            return { ok: true, json: async () => ({ success: true }) };
        }
    });
    const source = fs.readFileSync(new URL('../src/App.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + '\nglobalThis.c = { isSettingPose, controlError, onInitialPoseSet, mapData };', context);
    context.c.mapData.value = { info: { width: 4, height: 4, resolution: 0.05 }, data: new Array(16).fill(0) };
    await nextTick();
    // 模拟用户已经进入设点模式（App 自己持有该状态）
    context.c.isSettingPose.value = true;

    const pending = context.c.onInitialPoseSet({ x: 1, y: 1, yaw: 0 });
    await new Promise(resolve => setTimeout(resolve, 10));
    assert.equal(context.c.isSettingPose.value, true,
        'HTTP ack 未返回时不得退出设点模式');
    context.__ack();
    await pending;
    assert.equal(context.c.isSettingPose.value, false, 'ack 成功后应退出设点模式');
});
