// 决策负责人提出的对抗问题：航点「执行」按钮在 ACK 返回前被快速点两次，
// 会不会提交两次不同 requestId？
// 静态上可疑：App.submitGoals 没有提交中闸门，NavigationControl 只靠 500ms 轮询的
// goalBusy 禁用按钮，存在窗口。这里用可控 ack 直接验证，不靠假后端去重掩盖。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';
import { ref, computed, reactive, watch, nextTick } from 'vue';

function mount() {
    const requests = [];
    const props = reactive({ connected: true, sessionId: 'session-1', controlOwner: null });
    let goalsAck = null;
    let mapData = ref(null);
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
        crypto: { randomUUID: () => `uuid-${requests.length}-${Math.random().toString(36).slice(2, 8)}` },
        setInterval: () => Symbol('timer'), clearInterval() {},
        setTimeout: () => Symbol('timeout'), clearTimeout() {},
        CustomEvent: class {},
        fetch: async (url, options) => {
            const endpoint = String(url).split('/api/')[1];
            const body = options?.body ? JSON.parse(options.body) : null;
            requests.push({ endpoint, body });
            if (endpoint === 'navigation/goals') {
                // 闸门：由测试决定何时放行，模拟 ack 未返回的窗口
                return new Promise(resolve => {
                    goalsAck = () => resolve({ ok: true, json: async () => ({ success: true }) });
                });
            }
            // 控制权必须回本页自己的 sessionId（App 自己生成，不是测试传的 props），
            // 否则第二次提交会被"控制权在他人手里"拦下，测的就不是闸门了。
            return { ok: true, json: async () => ({ success: true, owner: body?.sessionId }) };
        }
    });
    const source = fs.readFileSync(new URL('../src/App.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0]
        .replace(/^import .*$/gm, '')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + '\nglobalThis.c = { waypointQueue, executeWaypoints, submitGoals, mapData, controlError, submitting: () => goalsSubmitting };', context);
    context.c.mapData.value = { info: { width: 4, height: 4, resolution: 0.05 }, data: new Array(16).fill(0) };
    return {
        c: context.c,
        requests,
        goalsRequests: () => requests.filter(item => item.endpoint === 'navigation/goals'),
        releaseAck: () => goalsAck?.(),
        flush: async () => { for (let i = 0; i < 30; i++) await Promise.resolve(); await nextTick(); }
    };
}

test('ack 未返回前快速双击「执行航点」只允许提交一次', async () => {
    const page = mount();
    page.c.waypointQueue.value = [
        { x: 1.0, y: 0.0, yaw: 0 },
        { x: 2.0, y: 1.5, yaw: 0.785 },
        { x: 0.5, y: 2.0, yaw: 3.14 }
    ];
    await nextTick();

    // 用户快速点两次（第一次的 HTTP 还没回来）
    page.c.executeWaypoints();
    page.c.executeWaypoints();
    await page.flush();

    const posts = page.goalsRequests();
    page.releaseAck();
    await page.flush();
    assert.equal(posts.length, 1,
        `ack 前双击不得重复提交目标，实际提交 ${posts.length} 次：`
        + JSON.stringify(posts.map(item => item.body?.requestId)));
    assert.deepEqual(posts[0].body.points.map(point => [point.x, point.y, point.yaw]),
        [[1, 0, 0], [2, 1.5, 0.785], [0.5, 2, 3.14]], '坐标与朝向顺序必须保留');
});

test('第一次 ACK 返回后再点才允许提交第二次（闸门不得永久卡死）', async () => {
    const page = mount();
    page.c.waypointQueue.value = [{ x: 1.0, y: 0.0, yaw: 0 }];
    await nextTick();
    page.c.executeWaypoints();
    await page.flush();
    page.releaseAck();
    await page.flush();
    await new Promise(resolve => setImmediate(resolve));   // 让 finally 里的闸门复位跑完
    await page.flush();
    assert.equal(page.goalsRequests().length, 1);
    // 第二次提交应被允许（新的 ack 闸门）
    page.c.executeWaypoints();
    await page.flush();
    assert.equal(page.goalsRequests().length, 2,
        `ACK 之后的新点击必须放行（队列=${page.c.waypointQueue.value.length}，`
        + `提交中=${page.c.submitting()}）`);
});
