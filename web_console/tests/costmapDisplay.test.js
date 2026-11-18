import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, reactive, computed, watch, nextTick } from 'vue';
import { worldToMap, mapToWorld, yawOf, headingToGoal, gridTransform } from '../src/lib/mapCoordinates.js';
import { cacheTransforms, lookupTransform, transformPoint, frameName } from '../src/lib/tfTransforms.js';

test('navigation costmap is replaced and cleared across mapping, stop and empty data', async () => {
    const props = reactive({ connected: true, isMapping: false, isNavigating: true,
        layers: {}, costmapData: null });
    const mounted = [];
    const canvases = [];
    const context = vm.createContext({ ref, computed, watch, nextTick, worldToMap, mapToWorld, yawOf,
        headingToGoal, gridTransform, cacheTransforms, lookupTransform, transformPoint, frameName,
        defineProps: () => props, defineEmits: () => () => {}, defineExpose() {},
        onMounted: callback => mounted.push(callback), onUnmounted() {},
        requestAnimationFrame() {}, console,
        window: { location: { hostname: 'localhost' } },
        document: { createElement() {
            const canvas = { getContext: () => ({
                createImageData: (w, h) => ({ data: new Uint8ClampedArray(w * h * 4) }),
                putImageData: image => { canvas.image = image; },
            }) };
            canvases.push(canvas);
            return canvas;
        } },
    });
    const source = fs.readFileSync(new URL('../src/components/SlamMap.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0].replace(/^import .*$/gm, '')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + '\nglobalThis.state = {costmap:()=>costmapCanvas, visible:()=>showNavigationCostmap.value}', context);
    const grid = value => ({ info: { width: 1, height: 1 }, data: [value] });

    props.costmapData = grid(100);
    await nextTick();
    assert.ok(context.state.costmap().image.data[3] > 0);
    props.costmapData = grid(0);
    await nextTick();
    assert.equal(context.state.costmap().image.data[3], 0, 'free cells erase previous blue pixels');

    props.isMapping = true;
    await nextTick();
    assert.equal(context.state.visible(), false);
    assert.equal(context.state.costmap(), null);
    props.costmapData = grid(100);
    await nextTick();
    assert.equal(context.state.costmap(), null, 'late navigation messages cannot overlay SLAM');
    mounted.forEach(callback => callback());
    assert.equal(context.state.costmap(), null, 'mounting during SLAM ignores cached navigation data');

    props.isMapping = false;
    await nextTick();
    assert.equal(context.state.costmap(), null, 'new session waits for a new costmap');
    props.costmapData = grid(100);
    await nextTick();
    assert.ok(context.state.costmap());
    props.costmapData = null;
    await nextTick();
    assert.equal(context.state.costmap(), null, 'empty input clears the rendered cache');
    props.costmapData = grid(100);
    await nextTick();
    props.isNavigating = false;
    await nextTick();
    assert.equal(context.state.costmap(), null, 'stopping navigation clears its overlay');
});
