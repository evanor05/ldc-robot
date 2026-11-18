import assert from 'node:assert/strict';
import path from 'node:path';
import test from 'node:test';
import { MAP_QUALITY, inspectMapPair, inspectMapQuality, qualityOptions } from '../mapStorage.js';

const MAPS = path.resolve(import.meta.dirname, '..', '..', 'maps');
// 18:03 手动 GMapping 图：1717 occupied / 86.1% unknown
const BAD_HANDHELD = path.join(MAPS, 'map_20261111180316_294');
// 18:00 纯自动图：1191 occupied / 81.8% unknown
const BAD_AUTO = path.join(MAPS, 'map_20261111180059_663');
// 16:56 / 17:11 合格图：3348/76.4% 与 3485/81.2%
const GOOD_A = path.join(MAPS, 'map_20261111165629_640');
const GOOD_B = path.join(MAPS, 'map_20261111171102_657');

test('统一门槛默认值就是 2500/0.85，不再回落到 300/0.95', () => {
    assert.deepEqual(qualityOptions(), MAP_QUALITY);
    assert.deepEqual(qualityOptions({}), MAP_QUALITY);
    assert.equal(qualityOptions({ min_occupied_cells: 2500, max_unknown_ratio: 0.85 }).minOccupiedCells, 2500);
    // 显式配置仍然优先（允许现场调严/调宽，但有唯一入口）
    assert.equal(qualityOptions({ min_occupied_cells: 3000 }).minOccupiedCells, 3000);
    assert.equal(qualityOptions({ max_unknown_ratio: 0.7 }).maxUnknownRatio, 0.7);
});

test('显式非法配置必须抛错，不能静默放宽门禁', () => {
    assert.throws(() => qualityOptions({ min_occupied_cells: '2500' }), /非法/);
    assert.throws(() => qualityOptions({ min_occupied_cells: 0 }), /非法/);
    assert.throws(() => qualityOptions({ max_unknown_ratio: 1.5 }), /非法/);
    assert.throws(() => qualityOptions({ max_unknown_ratio: 0 }), /非法/);
    assert.throws(() => qualityOptions({ expected_resolution: 0 }), /非法/);
});

test('差图 180316 / 180059 在统一门槛下不可导航，且仍能给出数字', () => {
    const handheld = inspectMapQuality(BAD_HANDHELD);
    assert.equal(handheld.navigable, false);
    assert.equal(handheld.ok, true, '结构可读');
    assert.equal(handheld.quality.occupied, 1717);
    assert.ok(Math.abs(handheld.quality.unknownRatio - 0.861) < 0.002);
    assert.match(handheld.error, /有效占用栅格 1717（最低 2500）/);
    assert.match(handheld.error, /未知占比 86\.1%（上限 85\.0%）/);

    const auto = inspectMapQuality(BAD_AUTO);
    assert.equal(auto.navigable, false);
    assert.equal(auto.quality.occupied, 1191);
    assert.match(auto.error, /有效占用栅格 1191（最低 2500）/);

    // 旧默认 300/95 会放行这两张图 —— 这就是本次修掉的"两套门槛"
    assert.doesNotThrow(() => inspectMapPair(BAD_AUTO, { minOccupiedCells: 300, maxUnknownRatio: 0.95 }));
});

test('合格图 165629 / 171102 在统一门槛下可导航', () => {
    for (const [file, occupied] of [[GOOD_A, 3348], [GOOD_B, 3485]]) {
        const result = inspectMapQuality(file);
        assert.equal(result.navigable, true, result.error || file);
        assert.equal(result.ok, true);
        assert.equal(result.quality.occupied, occupied);
        assert.ok(result.quality.unknownRatio <= MAP_QUALITY.maxUnknownRatio);
        assert.equal(result.error, null);
    }
});

test('inspectMapQuality 对缺失文件返回不可导航而不是抛异常', () => {
    const missing = inspectMapQuality(path.join(MAPS, 'map_does_not_exist_000000'));
    assert.equal(missing.ok, false);
    assert.equal(missing.navigable, false);
    assert.equal(missing.quality, null);
    assert.match(missing.error, /不存在/);
});
