import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, computed, watch, nextTick } from 'vue';

const usableMap = { info: { width: 1, height: 1, resolution: .05 }, data: [0] };

// 在 vm 沙箱中执行 App.vue 的控制脚本，模拟一个浏览器页面实例。
// 每个实例有独立 sessionStorage/心跳定时器/请求记录，可组合成"双页面"场景。
function makePage({ owner = null, cryptoAvailable = true, failEndpoints = [] } = {}) {
    const requests = [];
    const timers = new Map();
    const connection = ref('connected');
    let statusOwner = owner;
    const sessionStore = new Map();
    const mountedHooks = [];
    const context = vm.createContext({
        ref, computed, watch, onMounted(cb) { mountedHooks.push(cb) }, onUnmounted() {},
        useFoxglove: () => ({ connectionState: connection, errorMessage: ref(''),
            connect() {}, disconnect() {}, subscribe: () => 1 }),
        window: { location: { hostname: 'localhost' }, addEventListener() {}, removeEventListener() {},
            confirm: () => true },
        document: { hidden: false, addEventListener() {}, removeEventListener() {} }, console,
        ...(cryptoAvailable ? { crypto: { randomUUID: () => `uuid-${Math.random().toString(36).slice(2)}` } } : {}),
        sessionStorage: {
            getItem: key => sessionStore.get(key) ?? null,
            setItem: (key, value) => sessionStore.set(key, value)
        },
        setInterval: callback => { const id = Symbol(); timers.set(id, callback); return id; },
        clearInterval: id => timers.delete(id),
        fetch: async (url, options) => {
            const endpoint = url.split('/api/')[1];
            const body = options?.body ? JSON.parse(options.body) : {};
            requests.push({ endpoint, ...body });
            if (failEndpoints.includes(endpoint)) {
                return { ok: false, json: async () => ({ success: false, error: 'AMCL 未接受初始位置' }) };
            }
            if (endpoint === 'control/claim') {
                if (statusOwner && statusOwner !== body.sessionId) {
                    return { ok: false, json: async () => ({ success: false, error: '控制权由其他页面持有，请使用显式接管' }) };
                }
                statusOwner = body.sessionId;
                return { ok: true, json: async () => ({ success: true, owner: statusOwner }) };
            }
            if (endpoint === 'control/takeover') {
                statusOwner = body.sessionId;
                return { ok: true, json: async () => ({ success: true, owner: statusOwner, mode: 'locked' }) };
            }
            if (endpoint === 'control/release') {
                statusOwner = null;
                return { ok: true, json: async () => ({ success: true }) };
            }
            if (endpoint === 'navigation/heartbeat') {
                if (statusOwner && body.sessionId !== statusOwner) {
                    return { ok: false, json: async () => ({ success: false, error: '控制权不属于本页面，运动租约未续期' }) };
                }
                return { ok: true, json: async () => ({ success: true, owner: statusOwner }) };
            }
            if (endpoint === 'navigation/goals') {
                if (statusOwner && body.sessionId !== statusOwner) {
                    return { ok: false, json: async () => ({ success: false, error: '控制权由其他页面持有，需先显式接管' }) };
                }
                return { ok: true, json: async () => ({ success: true, goalId: 'goal-1' }) };
            }
            return { ok: true, json: async () => ({ success: true, controlId: 'manual-session' }) };
        }
    });
    const source = fs.readFileSync(new URL('../src/App.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '').replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + `\nglobalThis.c = { sessionId, controlOwner, isOwner, canControl, manualEnabled,
        controlError, pendingGoal, navigationState, staleManualSession, multiPointMode, waypointQueue,
        submitGoals, setEmergencyStop, onGoalSet, confirmPendingGoal, cancelPendingGoal, onNavigationState,
        clearQueue, cancelGoal, onInitialPoseSet, isSettingPose, onWaypointAdd,
        takeoverControl, releaseControl, ensureOwnership, mapData, canSendGoal, readControlGeneration: () => controlGeneration,
        readVehicleStatus: () => ({ online: vehicleOnline.value, text: statusText.value,
            indicator: typeof vehicleConnectionState === 'undefined' ? connectionState.value : vehicleConnectionState.value }),
        expireSensors() { telemetryClock.value = Math.max(lastScanAt.value, lastOdomAt.value) + 2000 },
        prepare() { lastScanAt.value = lastOdomAt.value = telemetryClock.value = backendStatusAt.value = Date.now();
            navigationState.value = { ...navigationState.value, checks: { lidar: {ok:true}, odometry: {ok:true} } } },
        expireBackend() { telemetryClock.value = backendStatusAt.value + 3100; lastScanAt.value = lastOdomAt.value = telemetryClock.value } };`, context);
    const c = context.c;
    const flush = async () => { for (let i = 0; i < 40; i++) await Promise.resolve(); await nextTick(); };
    const mount = async () => { for (const hook of mountedHooks) await hook(); await flush(); };
    return { c, requests, timers, flush, mount, setOwner: value => { statusOwner = value; },
        getOwner: () => statusOwner, connection };
}

test('goals auto-claim when control is free and carry session + request id', async () => {
    const page = makePage();
    const sent = await page.c.submitGoals([{ x: 1, y: 0, yaw: 0 }]);
    await page.flush();
    assert.equal(sent, true);
    const endpoints = page.requests.map(r => r.endpoint);
    assert.deepEqual(endpoints, ['control/claim', 'navigation/goals'], '无控制页面时应先取得控制权再发目标');
    const goals = page.requests.at(-1);
    assert.equal(goals.sessionId, page.c.sessionId);
    assert.ok(goals.requestId, '目标请求必须携带请求标识');
    assert.equal(page.c.controlOwner.value, page.c.sessionId);
    assert.equal(page.c.isOwner.value, true);
});

test('frozen Foxglove telemetry never takes the vehicle offline or triggers a hardware estop', async () => {
    const page = makePage();
    page.c.prepare();
    await page.c.setEmergencyStop(false);
    assert.equal(page.c.readVehicleStatus().indicator, 'connected');
    page.c.expireSensors();
    await page.flush();
    // 画面通道（Foxglove）只是显示数据，卡顿不能等同车端离线，更不能发急停。
    assert.equal(page.connection.value, 'connected', '桥接连接仍在线');
    assert.equal(page.c.readVehicleStatus().online, true, '遥测过期不得判定车端离线');
    assert.equal(page.c.readVehicleStatus().indicator, 'connected');
    assert.equal(page.c.manualEnabled.value, true, '画面卡顿不得解除遥控');
    assert.ok(!page.requests.some(r => r.endpoint === 'navigation/estop'),
        '遥测过期绝不允许前端向整车发急停；保护停车由后端负责');
});

test('backend sensor rejection disables manual control without hardware estop', async () => {
    const page = makePage(); page.c.prepare();
    await page.c.setEmergencyStop(false);
    page.c.onNavigationState({ mode: 'manual', lockKind: 'protective',
        checks: { lidar: { ok: true }, odometry: { ok: false, detail: '样本过期' } } });
    await page.flush();
    assert.equal(page.c.readVehicleStatus().online, false);
    assert.equal(page.c.manualEnabled.value, false, '后端判定数据失效后本地应解除遥控');
    assert.ok(!page.requests.some(r => r.endpoint === 'navigation/estop'),
        '保护停车不发硬件急停');
});

test('late locked polling snapshots cannot undo a newly enabled manual session', async () => {
    const page = makePage();
    page.c.prepare();
    const beforeEnable = page.c.readControlGeneration();
    const enabling = page.c.setEmergencyStop(false);
    const duringEnable = page.c.readControlGeneration();
    await enabling;
    for (const version of [beforeEnable, duringEnable]) {
        page.c.onNavigationState({ controlOwner: page.c.sessionId, mode: 'locked', goalState: 'idle' }, version);
        await page.flush();
        assert.equal(page.c.manualEnabled.value, true, '启用前及取消阶段发出的查询晚到时不能重新锁定遥控');
    }
    page.c.onNavigationState({ controlOwner: page.c.sessionId, mode: 'locked', goalState: 'idle' },
        page.c.readControlGeneration());
    await page.flush();
    assert.equal(page.c.manualEnabled.value, false, '新的后端锁定仍须立即禁用遥控');
    page.c.onNavigationState({ controlOwner: page.c.sessionId, mode: 'manual', goalState: 'idle' }, beforeEnable);
    await page.flush();
    assert.equal(page.c.manualEnabled.value, false, '旧 manual 状态不得恢复控制');
});

test('observer page cannot send goals or renew lease; estop still allowed', async () => {
    // 页面 A 是控制页；页面 B 是观察页（后端 owner=A）。
    const pageB = makePage({ owner: 'page-session-A' });
    pageB.c.onNavigationState({ controlOwner: 'page-session-A', mode: 'auto', goalState: 'executing' });
    await pageB.flush();
    assert.equal(pageB.c.isOwner.value, false);
    assert.equal(pageB.c.canControl.value, false);
    const sent = await pageB.c.submitGoals([{ x: 1, y: 0, yaw: 0 }]);
    await pageB.flush();
    assert.equal(sent, false);
    assert.equal(pageB.requests.filter(r => r.endpoint === 'navigation/goals').length, 0,
        '观察页不得发出目标请求');
    assert.match(pageB.c.controlError.value, /接管/);
    // 急停属于安全操作：观察页仍可发出。
    await pageB.c.setEmergencyStop(true);
    await pageB.flush();
    assert.ok(pageB.requests.some(r => r.endpoint === 'navigation/estop'));
});

test('explicit takeover switches ownership and keeps motion locked', async () => {
    const pageB = makePage({ owner: 'page-session-A' });
    pageB.c.onNavigationState({ controlOwner: 'page-session-A', mode: 'auto', goalState: 'executing' });
    await pageB.flush();
    await pageB.c.takeoverControl();
    await pageB.flush();
    assert.equal(pageB.getOwner(), pageB.c.sessionId);
    assert.equal(pageB.c.controlOwner.value, pageB.c.sessionId);
    assert.equal(pageB.c.manualEnabled.value, false, '接管后不得自动恢复运动');
    const takeover = pageB.requests.find(r => r.endpoint === 'control/takeover');
    assert.ok(takeover.requestId);
});

test('heartbeat carries page session and only runs while this page holds the lease', async () => {
    const page = makePage({ owner: null });
    await page.mount();
    for (const tick of [...page.timers.values()]) tick();
    await page.flush();
    // 未持有控制权的页面不发心跳：避免多页面互相干扰后端租约。
    assert.equal(page.requests.filter(r => r.endpoint === 'navigation/heartbeat').length, 0,
        '未持有控制权时不应发送心跳');
    assert.equal(page.c.controlOwner.value, null, '未取得控制权前不得因心跳而获得所有权');
    await page.c.ensureOwnership();
    await page.flush();
    for (const tick of [...page.timers.values()]) tick();
    await page.flush();
    const heartbeats = page.requests.filter(r => r.endpoint === 'navigation/heartbeat');
    assert.ok(heartbeats.length >= 1, '取得控制权后必须发送心跳');
    assert.equal(heartbeats[0].sessionId, page.c.sessionId, '心跳必须携带本页会话标识');
    // 心跳回显所有者：取得控制权后页面同步为控制页。
    assert.equal(page.c.controlOwner.value, page.c.sessionId);
    // 观察页不持有租约：既不发心跳，也不会把自己当成控制页。
    const observer = makePage({ owner: 'page-session-A' });
    await observer.mount();
    for (const tick of [...observer.timers.values()]) tick();
    await observer.flush();
    assert.equal(observer.requests.filter(r => r.endpoint === 'navigation/heartbeat').length, 0,
        '观察页不应发送心跳');
    assert.notEqual(observer.c.controlOwner.value, observer.c.sessionId, '观察页不能取得控制权');
    assert.equal(observer.c.manualEnabled.value, false);
});

test('refresh restoration: stale manual session is shown, never auto-resumed', async () => {
    const page = makePage({ owner: null });
    page.c.prepare();
    await page.c.setEmergencyStop(false);
    await page.flush();
    assert.equal(page.c.manualEnabled.value, true);
    // 模拟刷新：新页面实例（同 sessionStorage 会话），后端仍是 manual 模式。
    const refreshed = makePage({ owner: null });
    // 同一标签页刷新后会话 ID 相同——通过预置 sessionStorage 模拟：
    refreshed.c.onNavigationState({ controlOwner: refreshed.c.sessionId, mode: 'manual', goalState: 'idle' });
    await refreshed.flush();
    assert.equal(refreshed.c.manualEnabled.value, false, '刷新后不得自动恢复遥控');
    assert.equal(refreshed.c.staleManualSession.value, true, '页面状态必须反映后端 manual 模式并提示重新启用');
    // 明确重新启用后恢复。
    refreshed.c.prepare();
    refreshed.setOwner(refreshed.c.sessionId);
    await refreshed.c.setEmergencyStop(false);
    await refreshed.flush();
    assert.equal(refreshed.c.manualEnabled.value, true);
    assert.equal(refreshed.c.staleManualSession.value, false);
});

test('single goal goes through preview with position and yaw before sending', async () => {
    const page = makePage();
    page.c.onNavigationState({ controlOwner: null, mode: 'locked', goalState: 'idle', ready: true });
    page.c.mapData.value = usableMap;
    page.c.onGoalSet({ x: 1.25, y: -0.5, yaw: Math.PI / 2 });
    await page.flush();
    assert.equal(page.requests.length, 0, '预览阶段不得发送目标');
    assert.deepEqual({ x: page.c.pendingGoal.value.x, y: page.c.pendingGoal.value.y }, { x: 1.25, y: -0.5 });
    assert.equal(page.c.pendingGoal.value.yaw, Math.PI / 2);
    page.c.confirmPendingGoal();
    await page.flush();
    const goals = page.requests.find(r => r.endpoint === 'navigation/goals');
    assert.ok(goals, '确认后必须发送目标');
    assert.equal(goals.points[0].x, 1.25);
    assert.equal(goals.points[0].y, -0.5);
    assert.equal(goals.points[0].yaw, Math.PI / 2, '目标姿态不得重置为朝向零');
    assert.equal(page.c.pendingGoal.value, null, '发送成功后清除预览');
    // 取消预览不发送。
    page.c.onGoalSet({ x: 2, y: 2 });
    await page.flush();
    page.c.cancelPendingGoal();
    await page.flush();
    assert.equal(page.c.pendingGoal.value, null);
    assert.equal(page.requests.filter(r => r.endpoint === 'navigation/goals').length, 1);
});

test('session id survives simulated same-tab refresh and differs across tabs', async () => {
    const page1 = makePage();
    const page2 = makePage();
    assert.ok(page1.c.sessionId && page2.c.sessionId);
    assert.notEqual(page1.c.sessionId, page2.c.sessionId, '不同标签页必须是不同会话');
});


test('repeated confirmation submits one goal and preserves a newer preview', async () => {
    const page = makePage();
    page.c.mapData.value = usableMap;
    page.c.onGoalSet({ x: 1, y: 2, yaw: .7 });
    page.c.confirmPendingGoal();
    page.c.confirmPendingGoal();
    page.c.onGoalSet({ x: 3, y: 4, yaw: -.7 });
    await page.flush();
    assert.equal(page.requests.filter(r => r.endpoint === 'navigation/goals').length, 1);
    assert.equal(page.c.pendingGoal.value.x, 3, '旧请求完成不能清除新预览');
});

 test('no usable map blocks goals until map data arrives', async () => {
    const page = makePage();
    page.c.onNavigationState({ controlOwner: null, mode: 'locked', goalState: 'idle', ready: true,
        checks: { lidar: { ok: true }, odometry: { ok: true } } });
    await page.flush();
    assert.equal(page.c.canSendGoal.value, false, '无地图时不得允许发送目标');
    page.c.onGoalSet({ x: 1, y: 2 });
    await page.flush();
    assert.equal(page.c.pendingGoal.value, null, '无地图时不得生成目标预览');
    assert.match(page.c.controlError.value, /地图/);
    page.c.mapData.value = usableMap;
    await page.flush();
    assert.equal(page.c.canSendGoal.value, true, '地图到达后允许目标');
    page.c.onGoalSet({ x: 1, y: 2, yaw: 0 });
    await page.flush();
    assert.deepEqual({ x: page.c.pendingGoal.value.x, y: page.c.pendingGoal.value.y }, { x: 1, y: 2 });
});

test('bridge disconnect clears manual session and stops heartbeat renewal', async () => {
    const page = makePage();
    await page.mount();
    page.c.prepare();
    await page.c.ensureOwnership();
    await page.c.setEmergencyStop(false);
    await page.flush();
    assert.equal(page.c.manualEnabled.value, true);
    for (const tick of [...page.timers.values()]) tick();
    await page.flush();
    const before = page.requests.filter(r => r.endpoint === 'navigation/heartbeat').length;
    assert.ok(before >= 1, '持租约时应发送心跳');
    page.connection.value = 'disconnected';
    await page.flush();
    assert.equal(page.c.manualEnabled.value, false, '桥接断开必须清除遥控会话');
    const after = page.requests.filter(r => r.endpoint === 'navigation/heartbeat').length;
    for (const tick of [...page.timers.values()]) tick();
    await page.flush();
    assert.equal(page.requests.filter(r => r.endpoint === 'navigation/heartbeat').length, after,
        '桥接断开后不得继续用本页心跳维持租约');
    assert.ok(page.requests.some(r => r.endpoint === 'control/release'), '断开时应尽力释放控制租约');
});

test('fresh bridge packets cannot hide backend sensor rejection or stalled status polling', async () => {
    const page = makePage(); page.c.prepare();
    assert.equal(page.c.readVehicleStatus().online, true);
    await page.c.setEmergencyStop(false);
    page.c.onNavigationState({ mode:'manual', checks:{lidar:{ok:true},odometry:{ok:false,detail:'样本过期'}} });
    await page.flush();
    assert.equal(page.c.readVehicleStatus().online, false);
    assert.equal(page.c.manualEnabled.value, false);
    page.c.prepare(); page.c.expireBackend();
    assert.equal(page.c.readVehicleStatus().online, false);
 });

// ---------------- E：clearQueue ≠ cancelGoal ----------------
test('清空待执行航点不得隐式取消正在执行的目标', async () => {
    const page = makePage(); page.c.mapData.value = usableMap;
    await page.mount();
    page.c.waypointQueue.value = [{ x: 1, y: 0, yaw: 0 }, { x: 2, y: 0, yaw: 0 }];
    page.requests.length = 0;
    page.c.clearQueue();
    assert.equal(page.c.waypointQueue.value.length, 0, '待执行航点应清空');
    assert.ok(!page.requests.some(r => r.endpoint === 'navigation/cancel'),
        '清空航点绝不能发 navigation/cancel');
});

test('关闭多点模式只清队列，不取消正在执行的目标', async () => {
    const page = makePage(); page.c.mapData.value = usableMap;
    await page.mount();
    page.c.multiPointMode.value = true;
    await page.flush();                       // 先让 true 的 watch 结算，避免与挂载时的轮询互相干扰
    page.c.waypointQueue.value = [{ x: 1, y: 0, yaw: 0 }];
    page.requests.length = 0;
    page.c.multiPointMode.value = false;
    await page.flush();
    assert.equal(page.c.waypointQueue.value.length, 0);
    assert.ok(!page.requests.some(r => r.endpoint === 'navigation/cancel'),
        '关闭多点模式不得取消导航');
});

test('显式取消才发 navigation/cancel', async () => {
    const page = makePage(); page.c.mapData.value = usableMap;
    await page.mount();
    page.c.waypointQueue.value = [{ x: 1, y: 0, yaw: 0 }];
    page.requests.length = 0;
    await page.c.cancelGoal();
    await page.flush();
    const cancel = page.requests.find(r => r.endpoint === 'navigation/cancel');
    assert.ok(cancel, '显式取消必须发 navigation/cancel');
    assert.equal(typeof cancel.sessionId, 'string');
    assert.equal(page.c.waypointQueue.value.length, 0);
});

// ---------------- E：初始位姿等 ack ----------------
test('初始位姿成功后退出设点模式', async () => {
    const page = makePage(); page.c.mapData.value = usableMap;
    await page.mount();
    page.c.isSettingPose.value = true;
    await page.c.onInitialPoseSet({ x: 1, y: 2, yaw: 0 });
    assert.equal(page.c.isSettingPose.value, false);
    assert.equal(page.c.controlError.value, '');
});

test('初始位姿失败保留设点模式并显示可重试的错误', async () => {
    const page = makePage({ failEndpoints: ['navigation/initial_pose'] });
    page.c.mapData.value = usableMap;
    await page.mount();
    page.c.isSettingPose.value = true;
    await page.c.onInitialPoseSet({ x: 1, y: 2, yaw: 0 });
    assert.equal(page.c.isSettingPose.value, true, '失败时不能退出设点模式');
    assert.match(page.c.controlError.value, /初始位置未生效/);
    assert.match(page.c.controlError.value, /重新点击设置/);
});
