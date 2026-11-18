import assert from 'node:assert/strict';
import path from 'node:path';
import test from 'node:test';
import { inspectMapPair, qualityOptions } from '../mapStorage.js';
import { installNavigationRoutes } from '../navigationRoutes.js';

const ROOT = path.resolve(import.meta.dirname, '..');
const MAPS = path.resolve(ROOT, '../maps');
const BLANK = path.join(MAPS, 'map_20261111145429_359');
// 统一门槛（2500/0.85）下的合格图与差图，与现场实测数据一一对应。
const GOOD = path.join(MAPS, 'map_20261111165629_640');
const GOOD_2 = path.join(MAPS, 'map_20261111171102_657');
const BAD_HANDHELD = path.join(MAPS, 'map_20261111180316_294');
const BAD_AUTO = path.join(MAPS, 'map_20261111180059_663');
// Node 测试默认跳过生产门禁（其它用例使用 1x1 假地图）；本文件显式打开。
process.env.ROBOT_MAP_QUALITY = 'strict';

function routes(config = {}) {
    const handlers = new Map();
    const app = {
        get: (route, handler) => handlers.set('GET ' + route, handler),
        post: (route, handler) => handlers.set('POST ' + route, handler)
    };
    const activeProcesses = {};
    installNavigationRoutes({
        app, root: ROOT, activeProcesses,
        loadConfig: () => ({ maps_dir: MAPS, tasks: { navigation: [], mapping: [] }, ...config }),
        buildRosCommand: (command, args) => [command, ...(args || [])].join(' '),
        mappingBusy: () => false
    });
    return { handlers, activeProcesses };
}

function response() {
    return {
        statusCode: 200,
        body: null,
        status(code) { this.statusCode = code; return this; },
        json(body) { this.body = body; return this; }
    };
}

test('blank saved map is rejected before navigation spawns', async () => {
    const { handlers, activeProcesses } = routes();
    const handler = handlers.get('POST /api/start_navigation');
    const res = response();
    await handler({ body: { mapPath: BLANK, sessionId: 'test-session', requestId: 'map-gate-blank' } }, res);
    assert.equal(res.statusCode, 409);
    assert.match(String(res.body.error), /地图质量校验失败/);
    assert.match(String(res.body.error), /占用|unknown|未知/);
    assert.match(String(res.body.error), /origin=/);
    assert.deepEqual(activeProcesses, {}, '质量门禁失败不得 spawn 导航');
});

test('normal saved map passes the same unified gate', () => {
    const quality = inspectMapPair(GOOD, qualityOptions());
    assert.ok(quality.occupied >= 2500, `occupied=${quality.occupied}`);
    assert.ok(quality.unknownRatio <= 0.85, `unknownRatio=${quality.unknownRatio}`);
    assert.equal(quality.resolution, 0.05);
    assert.deepEqual(quality.origin, [-10, -10, 0]);
});

test('差图 180316 / 180059 启动导航必须首次即 409，且不得 spawn 任何进程', async () => {
    for (const [mapPath, requestId] of [[BAD_HANDHELD, 'gate-180316'], [BAD_AUTO, 'gate-180059']]) {
        const { handlers, activeProcesses } = routes();
        const res = response();
        await handlers.get('POST /api/start_navigation')(
            { body: { mapPath, sessionId: 'test-session', requestId } }, res);
        assert.equal(res.statusCode, 409, mapPath);
        assert.match(String(res.body.error), /地图质量校验失败/);
        assert.match(String(res.body.error), /2500|85\.0%/);
        assert.deepEqual(activeProcesses, {}, '质量门禁失败不得 spawn 导航进程');
    }
});

test('合格图 165629 / 171102 通过质量门禁（失败点落在"未配置任务"而不是质量校验）', async () => {
    for (const [mapPath, requestId] of [[GOOD, 'gate-165629'], [GOOD_2, 'gate-171102']]) {
        const { handlers } = routes();
        const res = response();
        await handlers.get('POST /api/start_navigation')(
            { body: { mapPath, sessionId: 'test-session', requestId } }, res);
        // 测试配置里没有 navigation 任务，能走到这一步就说明地图门禁已放行。
        assert.match(String(res.body.error), /未配置 navigation 任务/, mapPath);
        assert.doesNotMatch(String(res.body.error), /地图质量校验失败/);
    }
});

test('blank map cannot be exposed as a valid last-saved candidate', () => {
    assert.throws(() => inspectMapPair(BLANK, qualityOptions()), /地图内容不足|未知占比/);
});
