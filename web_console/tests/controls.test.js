import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, computed, watch, nextTick } from 'vue';
test('hold/cruise, gears, blur, ownership and Action queue', async () => {
    const requests = [], timers = new Map();
    const connection = ref('connected');
    let overlayOpen = false;
    const context = vm.createContext({ ref, computed, watch, onMounted() {}, onUnmounted() {},
        useFoxglove: () => ({ connectionState: connection, errorMessage: ref('') }),
        window: { location: { hostname: 'localhost' }, addEventListener() {}, removeEventListener() {} },
        document: { hidden: false, querySelector: selector => overlayOpen && selector.includes('.modal-overlay') ? {} : null }, console,
        setInterval: callback => { const id = Symbol(); timers.set(id, callback); return id; },
        clearInterval: id => timers.delete(id),
        fetch: async (url, options) => {
            const body = JSON.parse(options.body); requests.push({ endpoint: url.split('/api/')[1], ...body });
            return { ok: true, json: async () => ({ success: true, controlId: 'manual-session' }) };
        }
    });
    const source = fs.readFileSync(new URL('../src/App.vue', import.meta.url), 'utf8').split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '').replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + `\nglobalThis.c = { startDirection, endDirection, handleWindowBlur, setEmergencyStop,
        onNavigationState, onWaypointAdd, executeWaypoints, speedId, driveMode, manualEnabled, multiPointMode, waypointQueue,
        calibrationOpen, handleKeyDown, mapData, canSendGoal,
        prepare() {lastScanAt.value=lastOdomAt.value=telemetryClock.value=backendStatusAt.value=Date.now();
          navigationState.value={checks:{lidar:{ok:true},odometry:{ok:true}}}} };`, context);
    const c = context.c, flush = async () => { for (let i = 0; i < 8; i++) await Promise.resolve(); await nextTick(); };
    const pointer = { pointerId: 1, currentTarget: { setPointerCapture() {} } };
    c.prepare(); await c.setEmergencyStop(false); assert.equal(c.manualEnabled.value, true);
    c.calibrationOpen.value = true;
    const beforeModal = requests.length;
    c.handleKeyDown({key:'w', code:'KeyW', target:{tagName:'BODY'}, preventDefault(){}});
    await flush(); assert.equal(requests.length, beforeModal); assert.equal(timers.size, 0);
    // 标定向导打开时 Esc 只应关闭自身，不能再向整车发硬件急停。
    c.handleKeyDown({key:'Escape'}); await flush();
    assert.equal(requests.length, beforeModal, '向导打开时 Esc 不得触发急停');
    c.calibrationOpen.value = false;
    // 保存地图/摄像头灯箱等 overlay 存在时同理。
    overlayOpen = true;
    c.handleKeyDown({key:'Escape'}); await flush();
    assert.equal(requests.length, beforeModal, 'overlay 打开时 Esc 不得触发急停');
    overlayOpen = false;
    c.handleKeyDown({key:'Escape'}); await flush();
    assert.equal(requests.at(-1).endpoint, 'navigation/estop');
    await c.setEmergencyStop(false);
    c.startDirection(pointer, 'w'); await flush();
    for (let i = 0; i < 60; i++) { for (const tick of [...timers.values()]) tick(); await flush(); }
    assert.equal(requests.filter(r => r.linearX === .1).length, 61);
    c.endDirection(); await flush(); assert.equal(timers.size, 0); assert.equal(requests.at(-1).linearX, 0);
    c.driveMode.value = 'cruise'; await flush(); await c.setEmergencyStop(false);
    c.startDirection(pointer, 'w'); c.endDirection(); await flush(); assert.equal(timers.size, 1);
    const beforeBlur = requests.length;
    c.handleWindowBlur(); await flush(); assert.equal(timers.size, 0);
    // 失焦只做软停车：发零速度，绝不向整车发硬件急停。
    assert.equal(requests.at(-1).endpoint, 'navigation/velocity');
    assert.equal(requests.at(-1).linearX, 0);
    assert.ok(!requests.slice(beforeBlur).some(r => r.endpoint === 'navigation/estop'), '失焦不得触发硬件急停');
    c.speedId.value = 'fast'; await flush(); assert.equal(c.manualEnabled.value, false);
    await c.setEmergencyStop(false); c.startDirection(pointer, 'w'); await flush(); assert.equal(requests.at(-1).linearX, .4);
    await c.setEmergencyStop(true); c.multiPointMode.value = true; await flush();
    c.mapData.value = { info: { width: 1, height: 1 }, data: [0] };
    c.onWaypointAdd({x: 1, y: 0}); c.onWaypointAdd({x: 2, y: 0}); c.executeWaypoints(); await flush();
    assert.equal(requests.at(-1).points.length, 2);
    const sent = requests.filter(r => r.endpoint === 'navigation/goals').length;
    c.onNavigationState({ goalId: 'a', goalState: 'executing', current: {x: 1, y: 0}, queue: [{x: 2, y: 0}], feedback: {distance_remaining: .01}, mode: 'auto' });
    await flush(); assert.equal(requests.filter(r => r.endpoint === 'navigation/goals').length, sent);
    assert.equal(c.waypointQueue.value.length, 1);
    connection.value = 'disconnected'; await flush(); assert.equal(c.manualEnabled.value, false);
});
