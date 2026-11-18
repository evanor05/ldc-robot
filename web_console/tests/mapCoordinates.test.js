import test from 'node:test';
import assert from 'node:assert/strict';
import { worldToMap, mapToWorld, screenToWorld } from '../src/lib/mapCoordinates.js';
test('click inverts rendering for translated, rotated, zoomed maps', () => {
    for (const yaw of [0, .7, -1.5, Math.PI]) {
        const info = { height: 384, resolution: .05, origin: { position: { x: -4, y: 2 },
            orientation: { z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } } };
        for (const scale of [.1, 1, 3.6, 20]) {
            const point = { x: 1.25, y: -.6 }, view = { scale, offsetX: 147, offsetY: -34 };
            const pixel = worldToMap(info, point.x, point.y);
            const result = screenToWorld(info, pixel.x * scale + view.offsetX, pixel.y * scale + view.offsetY, view);
            assert.ok(Math.abs(result.x - point.x) < 1e-10);
            assert.ok(Math.abs(result.y - point.y) < 1e-10);
            assert.deepEqual(mapToWorld(info, 0, info.height - 1), info.origin.position);
        }
    }
});
test('drag direction follows rotated map world heading', () => {
    const info = { height: 10, resolution: .1, origin: { position: { x: 0, y: 0 }, orientation: { z: Math.SQRT1_2, w: Math.SQRT1_2 } } };
    const a = mapToWorld(info, 0, 9), b = mapToWorld(info, 5, 9);
    assert.ok(Math.abs(Math.atan2(b.y - a.y, b.x - a.x) - Math.PI / 2) < 1e-10);
});
