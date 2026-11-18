import { composeScript } from './helpers/composeScript.js';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import { ref, reactive, computed, watch, nextTick } from 'vue';

// 载入 MappingControl.vue 的 <script setup> 并在沙箱里执行（沿用 mappingMode.test.js 的做法）。
function mount({ stopReply, abortReply } = {}) {
    const requests = [];
    const emitted = [];
    const props = reactive({ connected: true, sessionId: 'session-1', canControl: true });
    const context = vm.createContext({
        ref, reactive, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {},
        defineProps: () => props,
        defineEmits: () => (...args) => emitted.push(args),
        window: {
            location: { hostname: 'localhost' },
            dispatchEvent() {},
            confirm: () => context.__confirmAnswer
        },
        __confirmAnswer: true,
        CustomEvent: class {},
        console,
        setInterval: () => Symbol('timer'),
        clearInterval() {},
        fetch: async (url, options) => {
            requests.push({ url, body: options?.body ? JSON.parse(options.body) : null });
            let reply = { success: true };
            if (url.includes('/abort_mapping')) reply = abortReply || { success: true, aborted: true, message: '建图已中止（未保存）' };
            else if (url.includes('/stop_mapping')) reply = stopReply || { success: true, savedMap: null };
            return { json: async () => reply };
        }
    });
    const source = composeScript({
    composables: [new URL('../src/composables/useMappingTask.js', import.meta.url)],
    component: new URL('../src/components/MappingControl.vue', import.meta.url)
});
    vm.runInContext(source + '\nglobalThis.ui = { isMapping, isLoading, saveFailed, saveErrorText, aborting, abortNotice, errorMsg, startMapping, stopMapping, abortMapping };', context);
    return {
        ui: context.ui, requests, emitted, props,
        setConfirm: value => { context.__confirmAnswer = value; }
    };
}

test('「停止但不保存」走 /api/abort_mapping 并带会话标识，成功后回到未建图状态', async () => {
    const { ui, requests, emitted } = mount();
    ui.isMapping.value = true;
    await ui.abortMapping();
    const abort = requests.find(item => item.url.endsWith('/abort_mapping'));
    assert.ok(abort, '必须调用 abort_mapping');
    assert.equal(abort.body.sessionId, 'session-1');
    assert.equal(ui.isMapping.value, false);
    assert.equal(ui.abortNotice.value, '建图已中止（未保存）');
    assert.ok(emitted.some(([name, value]) => name === 'update:isMapping' && value === false));
});

test('abort 前必须二次确认；取消则不发送任何请求', async () => {
    const { ui, requests, setConfirm } = mount();
    ui.isMapping.value = true;
    setConfirm(false);
    await ui.abortMapping();
    assert.equal(requests.length, 0);
    assert.equal(ui.isMapping.value, true);
});

test('保存失败但后端已停止：面板不得停留在“建图中”，并标出保存失败原因', async () => {
    const { ui } = mount({
        stopReply: { success: false, stopped: true, saveFailed: true, error: '地图内容不足：有效占用栅格 1717（最低 2500）' }
    });
    ui.isMapping.value = true;
    await ui.stopMapping();
    assert.equal(ui.isMapping.value, false, '停止已确认，必须退出建图中状态');
    assert.equal(ui.saveFailed.value, true);
    assert.match(ui.saveErrorText.value, /1717/);
    assert.equal(ui.isLoading.value, false, '失败后必须恢复可点，不能卡 loading');
});

test('保存失败且未确认停止：保持建图中，且不自动重试保存', async () => {
    const { ui, requests } = mount({ stopReply: { success: false, error: '保存地图超时，建图仍保持运行' } });
    ui.isMapping.value = true;
    await ui.stopMapping();
    assert.equal(ui.isMapping.value, true, '未确认停止就不能假装已停止');
    assert.equal(ui.saveFailed.value, false);
    assert.match(ui.errorMsg.value, /保存地图超时/);
    const stopCalls = requests.filter(item => item.url.includes('/stop_mapping'));
    assert.equal(stopCalls.length, 1, '保存失败不得自动重试');
    assert.equal(ui.isLoading.value, false);
});

test('abort 失败要把错误显示出来并允许再次点击', async () => {
    const { ui } = mount({ abortReply: { success: false, error: '停止建图失败：部分 ROS 子进程未能退出' } });
    ui.isMapping.value = true;
    await ui.abortMapping();
    assert.equal(ui.isMapping.value, true, '失败时不得伪装成已停止');
    assert.match(ui.errorMsg.value, /未能退出/);
    assert.equal(ui.isLoading.value, false);
    assert.equal(ui.aborting.value, false);
});
