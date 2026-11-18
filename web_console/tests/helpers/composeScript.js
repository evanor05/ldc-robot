// 测试装载助手：把组件 <script setup> 与它依赖的 composables 拼成一个可在 vm 里执行的脚本。
//
// 为什么需要它：抽 composables 之后，组件脚本里出现 `import { useX } from ...`，
// 而测试沙箱没有模块系统。直接剥掉 import 会让 useX 未定义，测试就只能复制一份逻辑
// （那等于没测生产代码）。这里改为按顺序把 composable 源码也拼进同一作用域，
// 只去 import 行与 export 关键字，其余源码原样保留。
import fs from 'node:fs';

function stripModuleSyntax(source) {
    return source
        .replace(/^import .*$/gm, '')
        .replace(/^export (const|let|function|async function|class)/gm, '$1')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
}

/**
 * @param {{ component: URL|string, composables?: Array<URL|string> }} options
 * @returns {string} 可直接 vm.runInContext 的脚本
 */
export function composeScript({ component, composables = [] }) {
    const parts = composables.map(url => stripModuleSyntax(fs.readFileSync(url, 'utf8')));
    const vue = fs.readFileSync(component, 'utf8');
    parts.push(stripModuleSyntax(vue.split('<script setup>')[1].split('</script>')[0]));
    return parts.join('\n');
}
