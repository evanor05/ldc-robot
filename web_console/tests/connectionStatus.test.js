import fs from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';
import * as Vue from 'vue';
import { renderToString } from '@vue/server-renderer';
import { parse, compileScript } from '@vue/compiler-sfc';

async function render(name, props) {
    const filename = new URL(`../src/components/${name}.vue`, import.meta.url);
    const { descriptor } = parse(fs.readFileSync(filename, 'utf8'));
    const script = compileScript(descriptor, { id: name, inlineTemplate: true }).content
        .replace(/import\s*\{([^}]+)\}\s*from\s*['"]vue['"];?/g,
            (_match, bindings) => `const {${bindings.replace(/\bas\b/g, ':')}} = Vue;`)
        .replace('export default', 'return');
    const component = new Function('Vue', script)(Vue);
    return renderToString(Vue.createSSRApp(component, props));
}

test('connection panel distinguishes vehicle loss from a live data bridge', async () => {
    const html = await render('ConnectionPanel', { connectionState: 'connected',
        vehicleOnline: false, vehicleStatusText: '车端已断开或数据中断 · 桥接在线' });
    assert.match(html, /数据桥接已连接/);
    assert.match(html, /车端已断开或数据中断/);
    assert.match(html, /status-dot disconnected/);
});

test('offline battery hides the cached percentage, voltage and gauge fill', async () => {
    const html = await render('BatteryStatus', { connected: false, percentage: 72, voltage: 7.73 });
    assert.doesNotMatch(html, /72%|7\.73/);
    assert.match(html, /width:0%/);
    assert.match(html, /--/);
});
