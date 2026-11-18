import { spawn } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';

let saving = false;

// 解析层默认值：只用于 inspectMapPair 被单独调用时保持向后兼容。
// 生产质量门槛见 MAP_QUALITY，必须由调用点显式注入 qualityOptions()。
const DEFAULT_MIN_OCCUPIED_CELLS = 300;
const DEFAULT_MAX_UNKNOWN_RATIO = 0.95;
const DEFAULT_EXPECTED_RESOLUTION = 0.05;
const DEFAULT_MIN_MAPPING_MS = 10000;

// 统一地图质量门槛（唯一真源）。此前"保存用 2500/85、导航与列表用 300/95"
// 两套标准并存，导致 1717 occupied / 86.1% unknown 的差图仍被放行导航。
export const MAP_QUALITY = Object.freeze({
    minOccupiedCells: 2500,
    maxUnknownRatio: 0.85,
    expectedResolution: 0.05
});

// 从 ros_config.json 的 save_map 段解析门槛；显式配置优先，否则回落 MAP_QUALITY。
// 保存、列表、导航启动三处必须调用同一个函数，避免再次漂移。
// 显式写了非法值（字符串、NaN、越界）时抛错，不允许静默放宽。
export function qualityOptions(saveMapConfig = {}) {
    return resolveQualityGate(saveMapConfig);
}

export function automaticMapName() {
    const stamp = new Date().toLocaleString('sv-SE', { timeZone: 'Asia/Shanghai' })
        .replace(/[-: ]/g, '');
    return `map_${stamp}_${String(Date.now() % 1000).padStart(3, '0')}`;
}

// PGM (P5) header parser. map_saver writes binary P5 with maxval 255.
export function parsePgm(buffer) {
    if (!Buffer.isBuffer(buffer) || buffer.length < 8) {
        throw new Error('PGM 文件为空或过短');
    }
    let index = 0;
    const tokens = [];
    const isSpace = value => value === 0x20 || value === 0x09 || value === 0x0a || value === 0x0d;
    while (tokens.length < 4) {
        while (index < buffer.length && isSpace(buffer[index])) index++;
        if (index < buffer.length && buffer[index] === 0x23) { // '#'
            while (index < buffer.length && buffer[index] !== 0x0a && buffer[index] !== 0x0d) index++;
            continue;
        }
        const start = index;
        while (index < buffer.length && !isSpace(buffer[index])) index++;
        if (start === index) throw new Error('PGM 文件头不完整');
        tokens.push(buffer.toString('ascii', start, index));
    }
    const [magic, widthText, heightText, maxvalText] = tokens;
    if (magic !== 'P5') throw new Error(`不支持的 PGM 格式：${magic}`);
    const width = Number(widthText);
    const height = Number(heightText);
    const maxval = Number(maxvalText);
    if (!Number.isInteger(width) || width <= 0 || !Number.isInteger(height) || height <= 0) {
        throw new Error('PGM 宽高无效');
    }
    if (!Number.isInteger(maxval) || maxval < 1 || maxval > 65535) throw new Error('PGM maxval 无效');
    if (index >= buffer.length) throw new Error('PGM 缺少像素数据');
    // PGM allows exactly one whitespace character after maxval.
    if (buffer[index] === 0x0d && buffer[index + 1] === 0x0a) index += 2;
    else index += 1;
    const bytesPerSample = maxval > 255 ? 2 : 1;
    const expectedBytes = width * height * bytesPerSample;
    const data = buffer.subarray(index);
    if (data.length < expectedBytes) {
        throw new Error(`PGM 数据不完整：${data.length} < ${expectedBytes}（${width}x${height}）`);
    }
    let occupied = 0;
    let free = 0;
    let unknown = 0;
    let other = 0;
    if (bytesPerSample === 1) {
        for (let i = 0; i < expectedBytes; i++) {
            const value = data[i];
            if (value === 0) occupied++;
            else if (value === 254) free++;
            else if (value === 205) unknown++;
            else other++;
        }
    } else {
        for (let i = 0; i < expectedBytes; i += 2) {
            const value = data.readUInt16BE(i);
            if (value === 0) occupied++;
            else if (value === 65534) free++;
            else if (value === 52505) unknown++;
            else other++;
        }
    }
    return { width, height, maxval, bytesPerSample, total: width * height,
        occupied, free, unknown, other };
}

// Minimal YAML reader for map_saver output. Only flat scalars/arrays are needed.
export function parseMapYaml(text) {
    const result = {};
    for (const rawLine of String(text).split(/\r?\n/)) {
        const line = rawLine.replace(/#.*$/, '').trim();
        if (!line) continue;
        const separator = line.indexOf(':');
        if (separator <= 0) continue;
        const key = line.slice(0, separator).trim();
        let value = line.slice(separator + 1).trim();
        if (!value) continue;
        if (/^\[.*\]$/.test(value)) {
            result[key] = value.slice(1, -1).split(',').map(item => Number(item.trim()));
        } else if (/^".*"$/.test(value) || /^'.*'$/.test(value)) {
            result[key] = value.slice(1, -1);
        } else if (value === 'true' || value === 'false') {
            result[key] = value === 'true';
        } else if (value !== '' && Number.isFinite(Number(value))) {
            result[key] = Number(value);
        } else {
            result[key] = value;
        }
    }
    return result;
}

// 结构校验 + 栅格统计（不含质量门槛）。文件缺失/结构损坏直接抛错。
function readMapPair(prefix, options = {}) {
    const yamlPath = prefix + '.yaml';
    const imagePath = prefix + '.pgm';
    if (!fs.existsSync(yamlPath)) throw new Error(`地图 YAML 不存在：${yamlPath}`);
    if (!fs.existsSync(imagePath)) throw new Error(`地图 PGM 不存在：${imagePath}`);
    const yaml = parseMapYaml(fs.readFileSync(yamlPath, 'utf8'));
    const pgm = parsePgm(fs.readFileSync(imagePath));

    const expectedImage = path.basename(prefix) + '.pgm';
    if (typeof yaml.image !== 'string' || path.basename(yaml.image) !== expectedImage) {
        throw new Error(`地图文件不一致：YAML image=${yaml.image || '<missing>'}，期望 ${expectedImage}`);
    }
    if (!Number.isFinite(yaml.resolution) || yaml.resolution <= 0) {
        throw new Error(`地图分辨率无效：${yaml.resolution}`);
    }
    const expectedResolution = options.expectedResolution ?? DEFAULT_EXPECTED_RESOLUTION;
    if (expectedResolution && Math.abs(yaml.resolution - expectedResolution) > 1e-4) {
        throw new Error(`地图分辨率异常：${yaml.resolution}，期望 ${expectedResolution}`);
    }
    if (!Array.isArray(yaml.origin) || yaml.origin.length !== 3 || !yaml.origin.every(Number.isFinite)) {
        throw new Error(`地图 origin 无效：${JSON.stringify(yaml.origin)}`);
    }
    if (pgm.other > 0) {
        throw new Error(`地图 PGM 含非标准像素值：${pgm.other} 个（仅支持 0/205/254）`);
    }
    const unknownRatio = pgm.total > 0 ? pgm.unknown / pgm.total : 1;
    return { width: pgm.width, height: pgm.height, resolution: yaml.resolution,
        origin: yaml.origin, occupied: pgm.occupied, free: pgm.free,
        unknown: pgm.unknown, total: pgm.total, unknownRatio };
}

// 质量门槛归一化：缺失回落 MAP_QUALITY；显式非法值抛错（宁可报错也不静默放宽）。
// 业务准入必须走 inspectMapQuality/inspectMapPair 并显式传这里的结果。
export function resolveQualityGate(options = {}) {
    const source = options && typeof options === 'object' ? options : {};
    const gate = { ...MAP_QUALITY };
    const keys = [['minOccupiedCells', 'min_occupied_cells', '有效占用栅格下限'],
        ['maxUnknownRatio', 'max_unknown_ratio', '未知占比上限'],
        ['expectedResolution', 'expected_resolution', '分辨率']];
    for (const [camel, snake, label] of keys) {
        const value = source[camel] ?? source[snake];
        if (value === undefined || value === null) continue;
        if (!Number.isFinite(value)) throw new Error(`地图质量配置 ${label}非法：${JSON.stringify(value)}`);
        if (camel === 'minOccupiedCells' && (!Number.isInteger(value) || value < 1)) {
            throw new Error(`地图质量配置 ${label}非法：${value}（需为正整数）`);
        }
        if (camel === 'maxUnknownRatio' && (value <= 0 || value > 1)) {
            throw new Error(`地图质量配置 ${label}非法：${value}（需在 0~1 之间）`);
        }
        if (camel === 'expectedResolution' && value <= 0) {
            throw new Error(`地图质量配置 ${label}非法：${value}（需为正数）`);
        }
        gate[camel] = value;
    }
    return gate;
}

// Validate a saved YAML/PGM pair before publishing it. Throws on malformed or
// empty maps so a single-scan blank map can never become the "last saved" map.
export function inspectMapPair(prefix, options = {}) {
    const quality = readMapPair(prefix, options);
    const minOccupiedCells = options.minOccupiedCells ?? DEFAULT_MIN_OCCUPIED_CELLS;
    const maxUnknownRatio = options.maxUnknownRatio ?? DEFAULT_MAX_UNKNOWN_RATIO;
    if (quality.occupied < minOccupiedCells || quality.unknownRatio > maxUnknownRatio) {
        throw Object.assign(new Error(
            `地图内容不足：有效占用栅格 ${quality.occupied}（最低 ${minOccupiedCells}），` +
            `未知占比 ${(quality.unknownRatio * 100).toFixed(1)}%（上限 ${(maxUnknownRatio * 100).toFixed(1)}%）；` +
            '建图仍在运行，请继续建图后重试'), { statusCode: 422 });
    }
    return quality;
}

// 不抛异常的质量评估入口（列表/恢复/UI 标注共用）。
// 缺省自身就用 MAP_QUALITY 安全门槛，不依赖调用方传参，避免"漏注入就静默退化"。
// ok=结构可读，navigable=通过质量门禁；被拒时仍尽量给出数字供前端标注原因。
export function inspectMapQuality(prefix, options = {}) {
    const gate = resolveQualityGate(options);
    try {
        return { ok: true, navigable: true, gate, quality: inspectMapPair(prefix, gate), error: null };
    } catch (error) {
        let quality = null;
        try { quality = readMapPair(prefix, gate); } catch { /* 结构损坏时没有可展示的数字 */ }
        // ok=结构可读（能解析出栅格统计），与是否通过质量门禁显式区分。
        return { ok: Boolean(quality), navigable: false, gate, quality, error: error.message };
    }
}

export function assertMappingDuration(startedAt, minimumMs = DEFAULT_MIN_MAPPING_MS) {
    if (!Number.isFinite(startedAt) || startedAt <= 0 || !(minimumMs > 0)) return;
    const elapsed = Date.now() - startedAt;
    if (elapsed < minimumMs) {
        throw Object.assign(new Error(
            `建图时间不足 ${(minimumMs / 1000).toFixed(0)} 秒（当前 ${(elapsed / 1000).toFixed(1)} 秒）；` +
            '建图仍在运行，请继续建图后重试'), { statusCode: 409 });
    }
}

export async function saveMap(config, mapName, buildCommand, options = {}) {
    if (saving) throw Object.assign(new Error('已有地图正在保存，请稍后再试'), { statusCode: 409 });
    if (typeof mapName !== 'string' || !mapName.trim()) {
        throw Object.assign(new Error('地图名称不能为空'), { statusCode: 400 });
    }
    const name = mapName.trim();
    if (!/^[a-zA-Z0-9_\-\u4e00-\u9fa5]+$/.test(name)) {
        throw Object.assign(new Error('地图名称只能包含文字、数字、下划线和连字符'), { statusCode: 400 });
    }
    const settings = config.save_map;
    if (!settings?.command) throw new Error('未配置地图保存命令');
    assertMappingDuration(options.mappingStartedAt, settings.min_mapping_ms ?? DEFAULT_MIN_MAPPING_MS);
    const prefix = path.join(settings.save_dir || config.maps_dir, name);
    if (['.yaml', '.pgm', '.png'].some(ext => fs.existsSync(prefix + ext))) {
        throw Object.assign(new Error('同名地图已存在，请使用新名称'), { statusCode: 409 });
    }
    saving = true;
    let staging;
    const published = [];
    try {
        fs.mkdirSync(path.dirname(prefix), { recursive: true });
        staging = fs.mkdtempSync(path.join(path.dirname(prefix), '.map-saving-'));
        const temporary = path.join(staging, name);
        const args = [...(settings.args || []), temporary, '--ros-args',
            '-p', 'save_map_timeout:=8.0', '-p', 'map_subscribe_transient_local:=true'];
        await new Promise((resolve, reject) => {
            const proc = spawn('bash', ['-c', buildCommand(settings.command, args)], {
                detached: true, stdio: ['ignore', 'pipe', 'pipe']
            });
            let output = '';
            let timedOut = false;
            const collect = chunk => { output = (output + chunk.toString()).slice(-4000); };
            proc.stdout.on('data', collect);
            proc.stderr.on('data', collect);
            const timer = setTimeout(() => {
                timedOut = true;
                try { process.kill(-proc.pid, 'SIGKILL'); } catch (error) {
                    if (error.code !== 'ESRCH') console.error('[save_map]', error.message);
                }
            }, settings.timeout_ms || 15000);
            proc.once('error', error => { clearTimeout(timer); reject(error); });
            proc.once('close', code => {
                clearTimeout(timer);
                if (code === 0 && !timedOut) resolve();
                else reject(new Error(timedOut ? '保存地图超时，建图仍保持运行' :
                    `保存地图失败（${code}）：${output.slice(-1200)}`));
            });
        });
        if (!['.yaml', '.pgm'].every(ext => fs.existsSync(temporary + ext) && fs.statSync(temporary + ext).size > 0)) {
            throw new Error('地图保存命令结束，但缺少有效的 YAML/PGM 文件；建图仍保持运行');
        }
        const yaml = fs.readFileSync(temporary + '.yaml', 'utf8');
        const declaredImage = parseMapYaml(yaml).image;
        if (typeof declaredImage !== 'string' || path.basename(declaredImage) !== name + '.pgm') {
            throw new Error(`地图文件不一致：YAML image=${declaredImage || '<missing>'}，期望 ${name}.pgm`);
        }
        fs.writeFileSync(temporary + '.yaml', yaml.replace(/^image:.*$/m, 'image: ' + JSON.stringify(name + '.pgm')));
        // 保存、列表、导航启动共用同一套门槛（qualityOptions 负责缺失回落与非法值校验）。
        const quality = inspectMapPair(temporary, qualityOptions(settings));
        // Publish YAML last. Hard links fail atomically if another writer owns the name.
        for (const ext of ['.pgm', '.yaml']) {
            fs.linkSync(temporary + ext, prefix + ext);
            published.push(prefix + ext);
        }
        return { name, path: prefix, yaml: prefix + '.yaml', image: prefix + '.pgm', savedAt: Date.now(), quality };
    } catch (error) {
        for (const file of published) fs.unlinkSync(file);
        throw error;
    } finally {
        if (staging) fs.rmSync(staging, { recursive: true, force: true });
        saving = false;
    }
}
