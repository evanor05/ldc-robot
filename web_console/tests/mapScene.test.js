import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';
import { ref, reactive, computed, watch, nextTick } from 'vue';
import * as coordinates from '../src/lib/mapCoordinates.js';
import * as tf from '../src/lib/tfTransforms.js';
const q = yaw => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });
const edge = (parent, child, yaw = 0, x = 0, y = 0) => ({ header: { frame_id: parent }, child_frame_id: child,
    transform: { translation: { x, y, z: 0 }, rotation: q(yaw) } });
const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`);

test('TF composes arbitrary chains and inverse edges; static transforms do not expire', () => {
    const tree = {};
    tf.cacheTransforms(tree, { transforms: [edge('/map', 'odom', Math.PI / 2, 2, 3), edge('odom', 'base_footprint', 0, 1)] }, 1000);
    tf.cacheTransforms(tree, { static: true, transforms: [edge('base_footprint', 'base_link'), edge('base_link', 'laser_frame', 0, .2)] }, 0);
    const forward = tf.lookupTransform(tree, 'map', 'laser_frame', 1001);
    const point = tf.transformPoint(forward, { x: 1, y: 0 });
    close(point.x, 2); close(point.y, 5.2);
    const original = tf.transformPoint(tf.lookupTransform(tree, 'laser_frame', 'map', 1001), point);
    close(original.x, 1); close(original.y, 0);
    assert.equal(tf.lookupTransform(tree, 'map', 'base_link', 3000), null);
    assert.ok(tf.lookupTransform(tree, 'base_footprint', 'laser_frame', 3000));
});

test('full quaternion transform handles inverted laser mounting', () => {
    const tree = {};
    const item = edge('base', 'laser');
    item.transform.rotation = { x: 1, y: 0, z: 0, w: 0 };
    tf.cacheTransforms(tree, { static: true, transforms: [item] });
    const point = tf.transformPoint(tf.lookupTransform(tree, 'base', 'laser'), { x: 0, y: 2 });
    close(point.y, -2);
});

test('costmap canvas follows its own origin, rotation and resolution', () => {
    const map = { height: 100, resolution: .1, origin: { position: { x: -2, y: 3 }, orientation: q(.7) } };
    const grid = { height: 20, resolution: .05, origin: { position: { x: 1, y: -2 }, orientation: q(-.3) } };
    const [a,b,c,d,e,f] = coordinates.gridTransform(map, grid);
    for (const [x,y] of [[0,0], [9,12], [2,19]]) {
        const world = coordinates.mapToWorld(grid, x, y), expected = coordinates.worldToMap(map, world.x, world.y);
        close(a*x+c*y+e, expected.x); close(b*x+d*y+f, expected.y);
    }
});

test('scene uses map TF, keeps supplied target yaw and cancels drag on mouse leave', async () => {
    const props = reactive({ connected: true, isNavigating: true, canSetGoal: true, layers: {},
        mapData: { header: { frame_id: 'map' }, info: { width: 100, height: 100, resolution: .1,
            origin: { position: { x: 0, y: 0 }, orientation: q(Math.PI / 2) } }, data: [] } });
    const emitted = [];
    const context = vm.createContext({ ...coordinates, ...tf, ref, computed, watch, nextTick,
        onMounted() {}, onUnmounted() {}, defineExpose() {}, defineProps: () => props,
        defineEmits: () => (...args) => emitted.push(args), requestAnimationFrame() {},
        ResizeObserver: class { observe() {} disconnect() {} },
        setTimeout() {}, window: { location: { hostname: 'localhost' } }, console });
    const source = fs.readFileSync(new URL('../src/components/SlamMap.vue', import.meta.url), 'utf8')
        .split('<script setup>')[1].split('</script>')[0].replace(/^import .*$/gm, '')
        .replace(/import\.meta\.env\.[A-Z_]+/g, 'undefined');
    vm.runInContext(source + '\nglobalThis.ui={robotPose,laserPoints,tfTree,updateScene,onMouseDown,onMouseMove,onMouseUp,onDblClick,cancelGesture,containerRef}', context);
    const ui = context.ui;
    tf.cacheTransforms(ui.tfTree, { static: true, transforms: [edge('base_footprint', 'base_link'), edge('base_link', 'laser_frame')] });
    props.tfData = { transforms: [edge('odom', 'base_footprint', 0, 2)] };
    await nextTick();
    assert.equal(ui.robotPose.value, null, 'map exists: do not silently draw odom coordinates on it');
    props.tfData = { transforms: [edge('map', 'odom', Math.PI / 2, 1, 3)] };
    props.scanData = { header: { frame_id: 'laser_frame' }, ranges: [1, Infinity], range_min: .1, range_max: 10, angle_min: 0, angle_increment: 1 };
    await nextTick();
    close(ui.robotPose.value.x, 1); close(ui.robotPose.value.y, 5);
    close(ui.robotPose.value.yaw, Math.PI / 2);
    assert.equal(ui.laserPoints.value.length, 1);
    close(ui.laserPoints.value[0].y, 6);
    // Display origin rotates 90deg: dragging right should produce +90deg world yaw.
    ui.containerRef.value = { getBoundingClientRect: () => ({left: 0, top: 0}) };
    await nextTick();
    ui.onMouseDown({ shiftKey: true, clientX: 10, clientY: 20 });
    ui.onMouseMove({ clientX: 30, clientY: 20 });
    ui.onMouseUp({});
    const goal = emitted.find(([event]) => event === 'goal-set')[1];
    close(goal.yaw, Math.PI / 2);
    const count = emitted.length;
    ui.onMouseDown({ shiftKey: true, clientX: 10, clientY: 20 });
    ui.cancelGesture(); ui.onMouseUp({});
    assert.equal(emitted.length, count, 'leaving canvas must cancel, not submit a pose/goal');
});
