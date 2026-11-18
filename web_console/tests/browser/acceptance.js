// 真浏览器验收：用真实 Chromium 打开真实构建产物，验证之前只能靠 vm 沙箱推断的交互。
// 零依赖：Chromium 走 CDP（DevTools 协议，Node 内置 WebSocket），后端与 Foxglove 桥用
// tests/browser/fakeConsoleServer.js 的假实现。不连车、不连 ESP32、不碰生产代码。
//
// 运行：cd web_console && npm run build && node tests/browser/acceptance.js
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';
import { startFakeConsole } from './fakeConsoleServer.js';

const API_PORT = Number(process.env.FAKE_API_PORT || 3000);
const WS_PORT = Number(process.env.FAKE_WS_PORT || 8765);
const DEBUG_PORT = Number(process.env.CDP_PORT || 0);
const checks = [];
let failures = 0;

function findChrome() {
    const explicit = process.env.CHROME_PATH;
    if (explicit && fs.existsSync(explicit)) return explicit;
    const base = process.env.PLAYWRIGHT_BROWSERS_PATH
        || path.join(os.homedir(), '.cache', 'ms-playwright');
    if (!fs.existsSync(base)) return null;
    for (const entry of fs.readdirSync(base).sort().reverse()) {
        for (const tail of ['chrome-linux64/chrome', 'chrome-linux/chrome', 'chrome-linux/headless_shell']) {
            const candidate = path.join(base, entry, tail);
            if (fs.existsSync(candidate)) return candidate;
        }
    }
    return null;
}

class Cdp {
    constructor(socket) {
        this.socket = socket;
        this.nextId = 0;
        this.pending = new Map();
        this.listeners = [];
        socket.addEventListener('message', event => this.onMessage(event.data));
    }

    onMessage(raw) {
        let message;
        try { message = JSON.parse(typeof raw === 'string' ? raw : raw.toString()); } catch { return; }
        if (message.id) {
            const entry = this.pending.get(message.id);
            if (!entry) return;
            this.pending.delete(message.id);
            if (message.error) entry.reject(new Error(message.error.message));
            else entry.resolve(message.result);
            return;
        }
        for (const listener of this.listeners) listener(message);
    }

    send(method, params = {}) {
        const id = ++this.nextId;
        return new Promise((resolve, reject) => {
            this.pending.set(id, { resolve, reject });
            this.socket.send(JSON.stringify({ id, method, params }));
        });
    }

    on(listener) { this.listeners.push(listener); }

    async evaluate(expression) {
        const result = await this.send('Runtime.evaluate', {
            expression, awaitPromise: true, returnByValue: true, userGesture: true
        });
        if (result.exceptionDetails) {
            throw new Error(result.exceptionDetails.exception?.description
                || result.exceptionDetails.text || '页面执行异常');
        }
        return result.result.value;
    }
}

async function openCdp(port, timeoutMs = 15000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        try {
            const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
            const page = list.find(item => item.type === 'page');
            if (page?.webSocketDebuggerUrl) {
                const socket = new WebSocket(page.webSocketDebuggerUrl);
                await new Promise((resolve, reject) => {
                    socket.addEventListener('open', resolve, { once: true });
                    socket.addEventListener('error', () => reject(new Error('CDP 连接失败')), { once: true });
                });
                return new Cdp(socket);
            }
        } catch { /* 还没起来 */ }
        await delay(150);
    }
    throw new Error('无法连接 Chromium 调试端口');
}

async function waitFor(cdp, expression, { timeoutMs = 12000, label = '条件' } = {}) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        if (await cdp.evaluate(expression)) return true;
        await delay(120);
    }
    throw new Error(`等待超时：${label}`);
}

async function record(name, fn) {
    try {
        const detail = await fn();
        checks.push({ name, ok: true, detail });
    } catch (error) {
        failures += 1;
        checks.push({ name, ok: false, error: error.message });
    }
}

const bodyText = 'document.body ? document.body.innerText : ""';
const clickByText = text => `(() => {
    const buttons = [...document.querySelectorAll('button')];
    const target = buttons.find(b => (b.innerText || '').trim().includes(${JSON.stringify(text)}));
    if (!target) return { clicked: false, reason: 'not-found' };
    if (target.disabled) return { clicked: false, reason: 'disabled' };
    target.click();
    return { clicked: true };
})()`;

async function main() {
    const chrome = findChrome();
    if (!chrome) throw new Error('未找到 Chromium：请设 CHROME_PATH 或安装 playwright 浏览器');
    const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'ldc-cdp-'));
    const fake = await startFakeConsole({ apiPort: API_PORT, wsPort: WS_PORT });

    const browser = spawn(chrome, [
        '--headless=new', '--disable-gpu', '--no-sandbox', '--disable-dev-shm-usage',
        '--no-first-run', '--no-default-browser-check', '--mute-audio',
        `--remote-debugging-port=${DEBUG_PORT}`, `--user-data-dir=${profile}`, 'about:blank'
    ], { stdio: 'ignore' });

    let cdp;
    try {
        const portFile = path.join(profile, 'DevToolsActivePort');
        let port = DEBUG_PORT;
        if (!port) {
            const deadline = Date.now() + 15000;
            while (Date.now() < deadline && !fs.existsSync(portFile)) await delay(100);
            port = Number(fs.readFileSync(portFile, 'utf8').split('\n')[0]);
        }
        cdp = await openCdp(port);
        const dialogs = [];
        cdp.on(message => {
            if (message.method === 'Page.javascriptDialogOpening') {
                dialogs.push(message.params.message);
                cdp.send('Page.handleJavaScriptDialog', { accept: true }).catch(() => {});
            }
        });
        await cdp.send('Page.enable');
        await cdp.send('Runtime.enable');
        // 在页面脚本前挂事件计数器：用 App 自己的 foxglove:connected 事件判定 WS 握手，
        // 而不是靠 SlamMap 的占位文案（那条文案还依赖 vehicleOnline，会抖动）。
        await cdp.send('Page.addScriptToEvaluateOnNewDocument', {
            source: "window.__fgConnected = 0;"
                + "window.addEventListener('foxglove:connected', () => { window.__fgConnected += 1; });"
        });
        // 固定桌面视口：否则小窗口会同时渲染移动端布局，出现与桌面不同的占位文案。
        await cdp.send('Emulation.setDeviceMetricsOverride', {
            width: 1440, height: 900, deviceScaleFactor: 1, mobile: false
        });
        await cdp.send('Page.navigate', { url: `http://127.0.0.1:${API_PORT}/` });

        await record('真实浏览器渲染出导航/建图面板（Foxglove 握手成功）', async () => {
            await waitFor(cdp, `${bodyText}.includes('导航控制') && ${bodyText}.includes('建图控制')`,
                { label: '面板渲染' });
            await waitFor(cdp, 'window.__fgConnected > 0', { label: 'foxglove:connected 事件' });
            const count = await cdp.evaluate('window.__fgConnected');
            return `导航/建图面板已渲染；foxglove:connected ×${count}（真实 WS 握手）`;
        });

        await record('差图不得被默认选中：开始导航按钮禁用并说明原因', async () => {
            const state = await cdp.evaluate(`(() => {
                const button = [...document.querySelectorAll('button')].find(b => (b.innerText || '').includes('开始导航'));
                return { found: Boolean(button), disabled: button ? button.disabled : null,
                    title: button ? (button.getAttribute('title') || '') : '',
                    text: ${bodyText} };
            })()`);
            if (!state.found) throw new Error('没有找到"开始导航"按钮');
            if (!state.disabled) throw new Error('差图时按钮仍可点击');
            if (!/不可导航|没有可导航的地图|请先选择一张地图/.test(state.text + state.title)) {
                throw new Error(`缺 blocker 说明：title=${state.title}`);
            }
            return `disabled=true，title=${state.title || '(空)'}`;
        });

        await record('就绪面板在"开始导航"之前就可见', async () => {
            await waitFor(cdp, `${bodyText}.includes('下一步：') || ${bodyText}.includes('检查')`,
                { label: '就绪面板' });
            return '就绪面板已渲染（含下一步提示）';
        });

        await record('开始建图后出现「保存并停止」与「停止但不保存」两个按钮', async () => {
            const click = await cdp.evaluate(clickByText('开始建图'));
            if (!click.clicked) throw new Error(`点击失败：${click.reason}`);
            await waitFor(cdp, `${bodyText}.includes('停止但不保存')`, { label: '停止但不保存按钮' });
            const text = await cdp.evaluate(bodyText);
            if (!text.includes('保存并停止建图')) throw new Error('缺少保存并停止按钮');
            return '两个按钮同时存在';
        });

        await record('点「停止但不保存」走 abort 接口并带会话标识，不触发保存', async () => {
            const click = await cdp.evaluate(clickByText('停止但不保存'));
            if (!click.clicked) throw new Error(`点击失败：${click.reason}`);
            const deadline = Date.now() + 12000;
            while (Date.now() < deadline
                && !fake.requests.some(item => item.path === '/api/abort_mapping')) {
                await delay(120);
            }
            const path = await cdp.evaluate('document.body.innerText');
            if (!fake.requests.some(item => item.path === '/api/abort_mapping')) {
                throw new Error(`abort 请求未到达后端；页面片段=${JSON.stringify(path.slice(-300))}`
                    + `；已收到请求=${JSON.stringify(fake.requests.slice(-4).map(r => r.path))}`);
            }
            // 精确判定"可见的「停止但不保存」按钮是否消失"，避免被事件日志/提示文案干扰
            const visibleAbortButton = `[...document.querySelectorAll('button')].some(b =>
                b.offsetParent !== null && (b.textContent || '').includes('停止但不保存'))`;
            try {
                await waitFor(cdp, `!(${visibleAbortButton})`, { label: '退出建图状态' });
            } catch (error) {
                const snippet = await cdp.evaluate(
                    "document.body.innerText.split('建图控制')[1]?.slice(0, 200) || ''");
                throw new Error(`${error.message}；建图面板片段=${JSON.stringify(snippet)}`
                    + `；后端已收=${JSON.stringify(fake.requests.slice(-5).map(r => r.path + ':' + r.method))}`);
            }
            const aborts = fake.requests.filter(item => item.path === '/api/abort_mapping');
            if (aborts.length !== 1) throw new Error(`abort 请求次数=${aborts.length}，期望 1`);
            if (!aborts[0].body?.sessionId) throw new Error('abort 请求缺少 sessionId');
            const saves = fake.requests.filter(item => item.path.includes('/api/save_map'));
            if (saves.length) throw new Error('abort 路径不应调用 save_map');
            return `abort=1（sessionId=${String(aborts[0].body.sessionId).slice(0, 8)}…），save_map=0`;
        });

        await record('相机 running 但无画面：显示离线文案，不显示健康', async () => {
            const text = await cdp.evaluate(bodyText);
            if (!text.includes('摄像头离线')) throw new Error('缺少"摄像头离线"文案');
            if (text.includes('实时画面正常')) throw new Error('无画面时仍显示健康文案');
            return '显示离线/重连，未显示"实时画面正常"';
        });

        await record('假桥收到真实 /map 订阅并推送 JSON OccupancyGrid', async () => {
            await waitFor(cdp, 'true', { label: 'noop' });
            const deadline = Date.now() + 8000;
            while (Date.now() < deadline && fake.state.mapFramesSent < 2) await delay(150);
            if (fake.state.mapSubscriptions < 1) throw new Error('/map 订阅数为 0：前端没有订阅地图');
            if (fake.state.mapFramesSent < 2) throw new Error(`已推地图帧 ${fake.state.mapFramesSent} < 2`);
            const hasCanvas = await cdp.evaluate(
                '[...document.querySelectorAll("canvas")].some(c => c.width > 0 && c.height > 0)');
            if (!hasCanvas) throw new Error('没有可见 canvas：地图可能没渲染');
            return `订阅 ${fake.state.mapSubscriptions} 次，已推 ${fake.state.mapFramesSent} 帧，canvas 正常`;
        });

        await record('没有调用假后端未实现的接口（404 会被记录）', async () => {
            const unexpected = fake.state.unexpectedRequests;
            if (unexpected.length) {
                throw new Error(`出现未实现接口：${JSON.stringify(unexpected.slice(0, 5))}`);
            }
            return '全部请求都有明确实现';
        });

        console.log(JSON.stringify({
            result: failures === 0 ? 'PASS' : 'FAIL',
            checks, dialogsHandled: dialogs.length
        }, null, 2));
    } finally {
        try { cdp?.socket?.close(); } catch {}
        browser.kill('SIGKILL');
        await fake.close();
        fs.rmSync(profile, { recursive: true, force: true });
    }
    if (failures) process.exitCode = 1;
}

main().catch(error => {
    console.log(JSON.stringify({ result: 'FAIL', error: error.message, checks }, null, 2));
    process.exitCode = 1;
});
