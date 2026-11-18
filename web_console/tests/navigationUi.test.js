import { composeScript } from './helpers/composeScript.js';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, reactive, computed, watch, nextTick } from 'vue';

test('pose mode follows map completion and status polling preserves actionable errors', async () => {
    let version = 0;
    const props = reactive({connected:true,isSettingPose:false,multiPointMode:false,waypointCount:0,
        controlVersion: () => version});
    let state = { isRunning:true,phase:'waiting_localization',missing:['初始位置'],error:'定位失败，请重试' };
    const emitted=[];
    let offline = false;
    let deferredReply = null;
    const context=vm.createContext({ref,computed,watch,onMounted(){},onUnmounted(){},
        defineProps:()=>props,defineEmits:()=>((...args)=>emitted.push(args)),console,
        window:{location:{hostname:'localhost'}},
        fetch:async()=>{ if (deferredReply) return deferredReply; if (offline) throw new Error('offline'); return {json:async()=>state} },
    });
    const source = composeScript({
        composables: [new URL('../src/composables/useMapCatalog.js', import.meta.url)],
        component: new URL('../src/components/NavigationControl.vue', import.meta.url)
    });
    vm.runInContext(source+'\nglobalThis.ui={isSettingPose,errorMsg,checkStatus,navigationState}',context);
    props.isSettingPose=true;await nextTick();assert.equal(context.ui.isSettingPose.value,true);
    props.isSettingPose=false;await nextTick();assert.equal(context.ui.isSettingPose.value,false);
    await context.ui.checkStatus();assert.equal(context.ui.errorMsg.value,'定位失败，请重试');
    state={isRunning:false,phase:'stopped',error:''};await context.ui.checkStatus();
    assert.equal(context.ui.errorMsg.value,'定位失败，请重试');
    offline=true;await context.ui.checkStatus();assert.match(context.ui.errorMsg.value,/状态不可用/);
    offline=false;await context.ui.checkStatus();assert.equal(context.ui.errorMsg.value,'');
    assert.ok(emitted.some(([name,data])=>name==='state' && data.phase==='waiting_localization'));

    // 查询在取消/启用期间发出，启用结束后才返回：不能更新面板或控制状态。
    let resolveReply;
    deferredReply = new Promise(resolve => { resolveReply = resolve });
    const before = emitted.length;
    const pending = context.ui.checkStatus();
    ++version;
    resolveReply({json:async()=>({isRunning:true, mode:'locked', goalState:'idle'})});
    await pending;
    assert.equal(emitted.length, before, '旧查询不得发出状态或导航开关事件');
    assert.equal(context.ui.navigationState.value.isRunning, false);
    deferredReply = null;
    await context.ui.checkStatus();
    assert.ok(emitted.some(([name,,generation])=>name==='state' && generation===version),
        '下一轮新查询仍正常更新');
});
