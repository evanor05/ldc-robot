import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { automaticMapName, saveMap } from '../mapStorage.js';

const root = fs.mkdtempSync(path.join(os.tmpdir(), 'leap-map-save-test-'));
const fixture = path.join(root, 'fixture.cjs');
fs.writeFileSync(fixture, `
const fs = require('node:fs');
const path = require('node:path');
const target = process.argv[2];
const mode = process.argv[3] || 'ok';
const name = path.basename(target);
const header = (w, h) => Buffer.from('P5\\n' + w + ' ' + h + '\\n255\\n', 'ascii');
const pixels = (w, h, occupied, free) => {
  const total = w * h;
  const data = Buffer.alloc(total, 205);
  for (let i = 0; i < occupied; i++) data[i] = 0;
  for (let i = occupied; i < occupied + free && i < total; i++) data[i] = 254;
  return data;
};
const pair = (w, h, occupied, free, options = {}) => {
  const yaml = [
    'image: ' + JSON.stringify(options.image || (name + '.pgm')),
    'mode: trinary',
    'resolution: ' + (options.resolution ?? 0.050),
    'origin: [' + (options.origin || '-10.000, -10.000, 0') + ']',
    'negate: 0',
    'occupied_thresh: 0.65',
    'free_thresh: 0.196',
    ''
  ].join('\\n');
  fs.writeFileSync(target + '.yaml', yaml);
  const headerBuffer = header(w, h);
  const data = options.truncate ? pixels(w, h, occupied, free).subarray(0, 64) : pixels(w, h, occupied, free);
  fs.writeFileSync(target + '.pgm', Buffer.concat([headerBuffer, data]));
};
if (mode === 'fail') process.exit(9);
if (mode === 'missing') {
  fs.writeFileSync(target + '.yaml', 'image: "' + name + '.pgm"\\nresolution: 0.05\\norigin: [-10, -10, 0]\\n');
  process.exit(0);
}
if (mode === 'blank') pair(608, 384, 99, 4891);
else if (mode === 'corrupt') pair(384, 384, 400, 1000, { truncate: true });
else if (mode === 'mismatch') pair(384, 384, 500, 5000, { image: 'other.pgm' });
else if (mode === 'badres') pair(384, 384, 500, 5000, { resolution: 0.100 });
else if (mode === 'badorigin') pair(384, 384, 500, 5000, { origin: '0, nope, 0' });
else pair(384, 384, 500, 100000);
`);
const quote = value => "'" + String(value).replaceAll("'", "'\\''") + "'";
const config = {
  maps_dir: root,
  save_map: { command: 'fixture', args: [], save_dir: root,
    min_occupied_cells: 300, max_unknown_ratio: 0.95, expected_resolution: 0.05 }
};
const command = mode => (_cmd, args) => [process.execPath, fixture, args[0], mode].map(quote).join(' ');

test('automatic names carry a timestamp', () => {
    assert.match(automaticMapName(), /^map_\d{14}_\d{3}$/);
});
test('success requires both real output files and valid map content', async () => {
    const result = await saveMap(config, 'success', command('ok'));
    assert.ok(fs.statSync(result.yaml).size > 0);
    assert.ok(fs.statSync(result.image).size > 0);
    assert.equal(result.quality.width, 384);
    assert.equal(result.quality.occupied, 500);
    assert.equal(result.quality.resolution, 0.05);
});
test('existing maps are not overwritten', async () => {
    const previous = fs.readFileSync(path.join(root, 'success.yaml'), 'utf8');
    await assert.rejects(saveMap(config, 'success', command('ok')), /同名地图/);
    assert.equal(fs.readFileSync(path.join(root, 'success.yaml'), 'utf8'), previous);
});
test('failed or incomplete commands reject instead of reporting saved', async () => {
    await assert.rejects(saveMap(config, 'failure', command('fail')), /保存地图失败/);
    await assert.rejects(saveMap(config, 'incomplete', command('missing')), /缺少有效/);
});
test('rejects traversal and empty names', async () => {
    await assert.rejects(saveMap(config, '../bad', command('ok')), /名称只能/);
    await assert.rejects(saveMap(config, '', command('ok')), /不能为空/);
});
test('incomplete output leaves no listed map and can be retried', async () => {
    await assert.rejects(saveMap(config, 'retry', command('missing')));
    assert.equal(fs.existsSync(path.join(root, 'retry.yaml')), false);
    assert.equal(fs.readdirSync(root).some(name => name.startsWith('.map-saving-')), false);
    await saveMap(config, 'retry', command('ok'));
    assert.match(fs.readFileSync(path.join(root, 'retry.yaml'), 'utf8'), /image: "retry.pgm"/);
});
test('timeout releases save lock without publishing a partial map', async () => {
    const slow = { ...config, save_map: { ...config.save_map, timeout_ms: 80 } };
    await assert.rejects(saveMap(slow, 'timeout', () => 'sleep 2'), /超时/);
    assert.equal(fs.existsSync(path.join(root, 'timeout.yaml')), false);
    await saveMap(config, 'after_timeout', command('ok'));
});
test('simultaneous saves are rejected rather than interleaved', async () => {
    const first = saveMap(config, 'parallel', (_cmd, args) => 'sleep 0.1 && ' + command('ok')(_cmd, args));
    await assert.rejects(saveMap(config, 'parallel_other', command('ok')), /正在保存/);
    await first;
});
test('single-scan blank map is rejected and not published', async () => {
    await assert.rejects(saveMap(config, 'blank', command('blank')), /地图内容不足/);
    assert.equal(fs.existsSync(path.join(root, 'blank.yaml')), false);
    assert.equal(fs.existsSync(path.join(root, 'blank.pgm')), false);
});
test('truncated/corrupt PGM is rejected', async () => {
    await assert.rejects(saveMap(config, 'corrupt', command('corrupt')), /PGM 数据不完整/);
    assert.equal(fs.existsSync(path.join(root, 'corrupt.yaml')), false);
});
test('YAML image mismatch is rejected', async () => {
    await assert.rejects(saveMap(config, 'mismatch', command('mismatch')), /地图文件不一致/);
});
test('resolution and origin are validated', async () => {
    await assert.rejects(saveMap(config, 'badres', command('badres')), /分辨率异常/);
    await assert.rejects(saveMap(config, 'badorigin', command('badorigin')), /origin 无效/);
});
test('saving within 10 seconds of mapping start is rejected', async () => {
    await assert.rejects(saveMap(config, 'early', command('ok'), { mappingStartedAt: Date.now() - 1000 }), /建图时间不足/);
    assert.equal(fs.existsSync(path.join(root, 'early.yaml')), false);
});
test('saving after the minimum mapping duration succeeds', async () => {
    const result = await saveMap(config, 'late', command('ok'), { mappingStartedAt: Date.now() - 11000 });
    assert.ok(result.quality.occupied >= 300);
});
