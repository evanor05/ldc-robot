// 后端 HTTP 层集成测试：真起 server.js，验证路由装配（单元测试覆盖不到的一层）。
// 运行：npm run test:integration（需空闲端口 18300；不依赖 ROS，无 ROS 时应优雅降级为 409）。
import assert from 'node:assert/strict'
import { spawn } from 'node:child_process'
import { fileURLToPath } from 'node:url'
import { setTimeout as delay } from 'node:timers/promises'

const BASE = 'http://localhost:18300'
const checks = []
const server = spawn('node', [fileURLToPath(new URL('../server.js', import.meta.url))], {
  env: { ...globalThis.process.env, ROBOT_API_PORT: '18300' }, stdio: 'ignore'
})

async function json(path, options) {
  const response = await fetch(BASE + path, options)
  let body = null
  try { body = await response.json() } catch { body = null }
  return { status: response.status, ok: response.ok, body }
}
const post = (path, body) => json(path, {
  method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body ?? {})
})

try {
  let ready
  for (let i = 0; i < 30; i++) {
    try { ready = await json('/api/runtime_config'); break } catch { await delay(100) }
  }
  assert.ok(ready?.ok, '后端未能在 3 秒内启动')
  checks.push('startup')

  const config = ready.body
  assert.equal(config.topics.battery, '/battery_state')
  assert.equal(config.topics.initial_pose, '/initialpose')
  checks.push('runtime topics')

  // ---- 地图目录 + 质量门槛（A）----
  const maps = (await json('/api/list_maps')).body
  assert.ok(Array.isArray(maps.maps))
  assert.ok(maps.maps.some(m => m.name === 'room' && m.path.endsWith('/maps/room')))
  assert.ok(maps.maps.every(m => !m.path.includes('/home/<user>/')))
  // 门槛必须与 ros_config.json 的 save_map 一致，且随响应返回"实际评估所用值"
  assert.deepEqual(maps.thresholds, {
    minOccupiedCells: 2500, maxUnknownRatio: 0.85, expectedResolution: 0.05
  })
  checks.push('map thresholds 2500/0.85')
  for (const map of maps.maps) {
    assert.equal(typeof map.navigable, 'boolean', `${map.name} 缺少 navigable`)
    if (!map.quality) {
      assert.equal(map.navigable, false, `${map.name} 无可读质量时不得判为可导航`)
      assert.ok(map.qualityError, `${map.name} 缺 qualityError`)
      continue
    }
    // 自校验：navigable 必须与响应自带的 quality + thresholds 一致，防止"报的门槛"和"判的门槛"分裂
    const expected = map.quality.occupied >= maps.thresholds.minOccupiedCells &&
      map.quality.unknownRatio <= maps.thresholds.maxUnknownRatio
    assert.equal(map.navigable, expected, `${map.name} navigable 与 quality 不自洽`)
  }
  if (maps.maps.some(m => m.quality && m.quality.occupied < 2500)) {
    assert.ok(maps.maps.some(m => m.navigable === false), '低占用图必须被判为不可导航')
    checks.push('low-occupancy map rejected')
  }

  // ---- 停止建图"不保存"路径（B）----
  const abortIdle = await post('/api/abort_mapping', {})
  assert.equal(abortIdle.status, 200, '无建图运行时 abort 应幂等成功')
  assert.equal(abortIdle.body.success, true)
  assert.equal(abortIdle.body.alreadyStopped, true)
  assert.equal(abortIdle.body.savedMap, null, 'abort 绝不能保存地图')
  const abortAgain = await post('/api/abort_mapping', {})
  assert.equal(abortAgain.body.success, true, '重复 abort 必须幂等')
  checks.push('abort idempotent, never saves')

  const alias = await post('/api/stop_task/mapping?save=false', {})
  assert.equal(alias.status, 200, 'stop_task?save=false 必须走不保存路径')
  assert.equal(alias.body.aborted, true)
  checks.push('stop_task?save=false -> abort')

  // ---- 任务状态形状（建图面板依赖）----
  const mappingStatus = (await json('/api/task_status/mapping')).body
  assert.equal(mappingStatus.isRunning, false)
  assert.equal(typeof mappingStatus.saving, 'boolean')
  assert.ok(['manual', 'auto'].includes(mappingStatus.mode))
  if (mappingStatus.lastSavedMap) {
    // "最近保存"不得是差图（A 的核心回归点）
    const saved = (await json('/api/list_maps')).body.maps.find(m => m.name === mappingStatus.lastSavedMap.name)
    if (saved) assert.equal(saved.navigable, true, '最近保存的图必须可导航')
  }
  checks.push('mapping status shape')

  // ---- 导航状态在无 ROS 时降级为 409，不得 500 ----
  const nav = await json('/api/navigation_status')
  assert.ok([200, 409].includes(nav.status), `导航状态应为 200 或 409，实际 ${nav.status}`)
  if (nav.status === 409) {
    assert.equal(nav.body.success, false)
    assert.ok(nav.body.error, '降级时必须有可读错误')
  }
  checks.push('navigation_status degrades to 409, not 500')

  // ---- 启动导航的差图门禁（A 的 HTTP 入口）----
  const bad = maps.maps.find(m => m.navigable === false)
  if (bad) {
    const rejected = await post('/api/start_navigation', { mapPath: bad.path, sessionId: 'integration', requestId: 'gate-1' })
    assert.equal(rejected.status, 409, '差图必须 409')
    assert.match(String(rejected.body.error), /地图质量校验失败|地图内容不足|未知占比/)
    checks.push('start_navigation rejects non-navigable map')
  }

  console.log(JSON.stringify({ result: 'PASS', checks }))
} finally {
  server.kill('SIGTERM')
  await new Promise(resolve => server.once('exit', resolve))
}
