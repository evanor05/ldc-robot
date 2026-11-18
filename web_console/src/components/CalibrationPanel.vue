<template>
  <div class="card calibration-entry">
    <div class="card-header">诊断采集与标定</div>
    <div class="card-body">
      <p>记录导航问题，填写实测数据，查看候选与复测报告。</p>
      <button @click="open = true; refresh()">打开标定向导</button>
    </div>
    <Teleport to="body">
        <dialog v-if="open" ref="dialog" class="calibration-dialog" tabindex="-1" aria-label="LDC 标定与诊断向导" @keydown="dialogKeydown" @cancel.prevent="closeDialog($event)">
          <header><h2>标定与诊断 <small>{{ simulation ? '仿真资料 · 与实车隔离' : '实车资料' }}</small></h2><button @click="open = false">关闭</button></header>
          <p class="muted">采集只记录数据；关闭向导后可遥控。向导内禁用行驶快捷键；按 Esc 仅关闭向导，急停请使用页面上的「急停」按钮。报告生成候选参数，应用前需独立复测。</p>
          <p v-if="error" class="error" role="alert">{{ error }}</p>
          <div class="session-bar">
            <label>当前会话<select v-model="selectedId" :disabled="busy" @change="loadSession()"><option value="">选择已有会话</option><option v-for="item in sessions" :key="item.id" :value="item.id">{{ item.name }} · {{ localTime(item.createdAt) }}</option></select></label>
            <label>新会话名称<input v-model="name" maxlength="80" placeholder="例如：走廊导航问题 / 第一次直行标定"></label>
            <button :disabled="busy" @click="createSession()">新建会话</button>
          </div>
          <template v-if="session">
            <p v-if="session.validationOf" class="notice">独立复测：关联报告 {{ session.validationOf.reportId }}。请记录实际应用的参数版本，不能把拟合残差当复测精度。</p>
            <nav class="steps"><button v-for="(label, index) in ['选择项目', '开始记录', '填写真值', '候选与复测']" :key="label" :class="{ active: step === index }" @click="step = index">{{ index + 1 }} · {{ label }}</button></nav>
            <section v-if="step === 0" class="step-body">
              <label>标定或诊断项目<select v-model="project" @change="resetRow"><option v-for="item in projects" :key="item.id" :value="item.id">{{ item.label }}</option></select></label>
              <p>{{ projectInfo.hint }}</p>
              <p class="muted">建议顺序：车体与参考点 → 编码器 → 直行 → 转角 → IMU → 雷达 → 相机 → 停车距离。</p>
              <button @click="step = 1">下一步：记录</button>
            </section>
            <section v-else-if="step === 1" class="step-body">
              <div class="fields">
                <label>记录时长（秒）<input v-model.number="seconds" type="number" min="1" max="600"></label>
                <label>容量上限（MiB）<input v-model.number="maxMb" type="number" min="1" max="512"></label>
                <label>相机采样间隔（秒，0 不保存）<input v-model.number="cameraEvery" type="number" min="0" max="60" step="0.1"></label>
              </div>
              <p>{{ projectInfo.hint }}</p>
              <a v-if="project === 'camera'" :href="`${base}/sessions/${selectedId}/board`">下载 A4 棋盘（9×6 内角点；按实际大小打印并尺量）</a>
              <div class="actions"><button :disabled="busy || job?.running" @click="startCapture">开始只读采集</button><button :disabled="busy || !job?.running" @click="stopCapture">结束本次采集 / 回放</button><button @click="step = 2">下一步：填写真值</button></div>
              <p v-if="job?.running" class="notice">{{ job.kind === 'record' ? '正在采集' : '正在隔离回放 · Domain 196' }}；关闭向导后任务仍在后端运行。</p>
              <p v-if="job?.error" class="error">{{ job.error }}</p>
              <table><thead><tr><th>采集</th><th>状态</th><th>时长 / 容量</th><th>操作</th></tr></thead><tbody>
                <tr v-for="capture in session.captures" :key="capture.id"><td>{{ capture.id.slice(0, 8) }}</td><td>{{ captureLabel(capture.state) }}<span v-if="capture.hasSensorData === false"> · 无雷达/里程计/IMU数据</span></td><td>{{ Number(capture.seconds || 0).toFixed(1) }}s / {{ (Number(capture.bytes || 0) / 1048576).toFixed(1) }}MiB</td><td><button :disabled="busy || job?.running" @click="captureReport(capture.id, 'report')">故障时间线</button><button :disabled="busy || job?.running" @click="captureReport(capture.id, 'summarize')">测量总结</button></td></tr>
              </tbody></table>
              <details><summary>开发回放（独立环境）</summary><p>仅在本机 Domain 196 发布历史传感器，控制指令进入观察话题。日常实车启动不依赖此功能。</p><select v-model="captureId"><option value="">选择采集</option><option v-for="capture in session.captures" :key="capture.id" :value="capture.id">{{ capture.id }}</option></select><button :disabled="busy || job?.running || !captureId" @click="replayCapture">隔离回放</button></details>
            </section>
            <section v-else-if="step === 2" class="step-body">
              <p>{{ projectInfo.hint }}</p>
              <template v-if="project === 'dimensions'">
                <p class="muted">尺寸和安装位置使用 m，安装角使用 deg；未测量留空。</p>
                <div class="fields"><label v-for="(value, key) in dimensions" :key="key">{{ dimensionLabel(key) }}<input v-model="dimensions[key]" type="number" step="any"></label></div>
                <button :disabled="busy" @click="saveDimensions">保存实测尺寸与外参</button>
              </template>
              <template v-else-if="schema.length">
                <div class="fields"><label v-for="key in schema" :key="key">{{ fieldLabels[key] || key }}
                  <select v-if="key === 'direction'" v-model="row[key]"><option v-for="option in project === 'turn' ? ['ccw', 'cw'] : ['forward', 'reverse']" :key="option" :value="option">{{ choices[option] }}</option></select>
                  <select v-else-if="key === 'valid'" v-model="row[key]"><option value="true">有效试验</option><option value="false">排除，保留原因</option></select>
                  <select v-else-if="key === 'odom_source'" v-model="row[key]"><option value="unknown">未确认</option><option value="wheel_only">已核对纯轮式数据</option><option value="imu">IMU</option><option value="ekf">EKF</option></select>
                  <input v-else v-model="row[key]" :type="['trial_id', 'sensor', 'notes'].includes(key) ? 'text' : 'number'" step="any" maxlength="500">
                </label></div>
                <button :disabled="busy" @click="saveMeasurement">保存本次试验</button>
                <p>已记录 {{ session.measurementCounts?.[project] ?? session.measurements?.[project]?.length ?? 0 }} 次；无效试验仍保留，避免挑选结果。</p>
              </template>
              <template v-else-if="project === 'camera'">
                <label>棋盘采集<select v-model="captureId"><option value="">选择采集</option><option v-for="capture in session.captures" :key="capture.id" :value="capture.id">{{ capture.id }} · {{ captureLabel(capture.state) }}</option></select></label>
                <label>尺量后的格长（mm）<input v-model.number="squareMm" type="number" min="1" max="100" step="0.1"></label>
                <button :disabled="busy || job?.running || !captureId" @click="cameraCandidate">生成相机候选与留出验证</button>
              </template>
              <label>人工观测 / 已核对的固件与参数版本<textarea v-model="note" rows="3" maxlength="2000" placeholder="例如：轮子方向、每圈计数、地面与载荷、静止 IMU、异常发生时的动作"></textarea></label>
              <button :disabled="busy || !note.trim()" @click="saveObservation">保存观测记录</button>
              <details v-if="session.observations?.length"><summary>已保存的人工观测</summary><ul><li v-for="item in session.observations" :key="item.createdAt">{{ localTime(item.createdAt) }} · {{ item.note }}</li></ul></details>
              <button @click="step = 3">下一步：报告</button>
            </section>
            <section v-else class="step-body">
              <p>候选与线上参数分开保存；需要另一批独立试验验证效果。</p>
              <div class="actions"><button :disabled="busy || job?.running" @click="analyze">生成尺寸 / 里程计候选报告</button><button :disabled="busy || !selectedReport" @click="createValidation">新建当前报告的独立复测</button></div>
              <p v-if="selectedReport" class="notice">独立复测将关联报告：{{ selectedReport.reportId }}</p>
              <p v-else class="muted">请先选择一份候选报告，再创建独立复测。</p>
              <ul><li v-for="report in session.reports" :key="report.id"><button :aria-pressed="result?.reportId === report.reportId" @click="result = report">{{ localTime(report.createdAt) }} · {{ report.status }} · {{ report.reportId }}</button></li></ul>
            </section>
            <section v-if="result" class="report">
              <div class="actions"><h3>{{ result.snapshots ? '故障记录与导航时间线' : '测量 / 候选报告' }}</h3><button @click="downloadReport">下载 JSON 报告</button></div>
              <p v-if="result.reportId">报告版本：{{ result.reportId }}</p>
              <p v-if="result.applied === false" class="notice">候选尚未应用到车辆。</p>
              <ul v-if="result.warnings"><li v-for="warning in result.warnings" :key="warning">{{ warning }}</li></ul>
              <p v-if="result.odom">纵向位移 {{ result.odom.forward_delta_m.toFixed(4) }} m；累计转角 {{ result.odom.unwrapped_yaw_deg.toFixed(2) }}°。请另填尺量真值。</p>
              <p v-if="result.snapshots" class="muted">当前展示最后 {{ result.snapshots.length }} / {{ result.snapshotCount }} 个快照；完整时间线保存在采集数据库。</p>
              <ol v-if="result.snapshots" class="timeline"><template v-for="snapshot in result.snapshots" :key="snapshot.elapsed"><li v-if="snapshot.eventsGap">{{ snapshot.elapsed.toFixed(1) }}s · {{ snapshot.eventsGap }}</li><li v-for="event in snapshot.events?.events || []" :key="`${snapshot.elapsed}-${event.seq}`">{{ snapshot.elapsed.toFixed(1) }}s · {{ event.type }} · {{ event.reason }}</li></template></ol>
              <table v-if="reportRows.length"><thead><tr><th>指标</th><th>结果</th><th>说明</th></tr></thead><tbody><tr v-for="metric in reportRows" :key="metric.label"><td>{{ metric.label }}</td><td>{{ metric.value }}</td><td>{{ metric.note }}</td></tr></tbody></table>
              <p v-if="result.independentValidation" class="notice">{{ result.independentValidation.note }}</p>
              <details><summary>详细数据与来源</summary><pre>{{ JSON.stringify(result, null, 2) }}</pre></details>
            </section>
          </template>
          <p v-else class="muted">新建会话后按步骤记录。未测量字段保持空白。</p>
        </dialog>
    </Teleport>
  </div>
</template>

<script setup>
import { ref, computed, watch, nextTick, onMounted, onUnmounted } from 'vue'
const emit = defineEmits(['opened', 'active-change'])
const dialog = ref(null)
const base = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api/calibration`
const open = ref(false), busy = ref(false), error = ref(''), simulation = ref(false)
watch(open, async visible => {
  emit('active-change', visible)
  if (!visible) return
  await nextTick()
  dialog.value?.showModal()
  dialog.value?.focus()
  emit('opened')
})
function dialogKeydown(event) {
  // 向导内的 Esc 只关闭向导，必须阻止冒泡到 App 的全局急停快捷键。
  if (event.key === 'Escape') {
    event.preventDefault()
    event.stopPropagation()
    open.value = false
    return
  }
  event.stopPropagation()
}
function closeDialog(event) {
  event?.preventDefault?.()
  event?.stopPropagation?.()
  open.value = false
}
const sessions = ref([]), selectedId = ref(''), session = ref(null), name = ref(''), step = ref(0)
const project = ref('diagnostics'), row = ref({}), dimensions = ref({}), note = ref(''), result = ref(null)
const seconds = ref(60), maxMb = ref(128), cameraEvery = ref(0), squareMm = ref(20), captureId = ref(''), job = ref(null)
const projects = [
  { id: 'diagnostics', label: '导航故障复现', hint: '先开始记录，再通过原控制页面复现故障；记录器不会取得控制权或发运动命令。' },
  { id: 'dimensions', label: '1. 车体尺寸与参考点 / 外参', hint: '从确认的 base_link 参考点尺量整车包络，包括轮胎、线缆和固定载荷。' },
  { id: 'encoder', label: '2. 编码器方向与计数', hint: '实车就绪后核对左右轮方向和每圈计数；记录观测，不能以配置值代替实测。' },
  { id: 'distance', label: '3. 直行尺度', hint: '正反方向分别记录，后退距离使用负数；reference 来自尺量，odom 来自该次采集。' },
  { id: 'turn', label: '4. 转角 / 有效轮距', hint: '顺逆时针分别记录，逆时针为正；确认 wheel_only 后才计算有效轮距。' },
  { id: 'imu', label: '5. IMU 静止与轴向', hint: '静止记录至少 30 秒，查看测量总结；未确认静止的数据不能当零偏，covariance=-1 仍视为不可用。' },
  { id: 'laser', label: '6. 雷达距离', hint: '从激光测量原点量到同一平墙，覆盖多个距离；不要从保险杠测量。' },
  { id: 'bearing', label: '6. 雷达 / 相机方向', hint: '记录明确的传感器名称和角度约定；雷达左正、相机射线右正，不可混合。' },
  { id: 'camera', label: '7. 相机内参', hint: '相机采样间隔设为 1 秒。保持分辨率和镜头设置，采集 25–40 张覆盖四角、边缘和倾斜视角的棋盘。' },
  { id: 'braking', label: '8. 停车距离', hint: '受控低速测量停止请求到停稳的距离，记录速度、地面和载荷；少量最大值不是安全上界。' },
  { id: 'navigation', label: '9. 独立导航验收', hint: '实际位置与朝向使用地面标记等独立基准测量，不用 AMCL 自己验证自己。' }
]
const projectInfo = computed(() => projects.find(item => item.id === project.value))
const schema = computed(() => session.value?.schemas?.[project.value] || [])
const selectedReport = computed(() => {
  const report = result.value
  return report?.sessionId === selectedId.value && report?.reportId && report.applied === false &&
    ['candidate_only', 'camera_candidate'].includes(report.status) ? report : null
})
const reportRows = computed(() => {
  const report = result.value || {}, metrics = []
  for (const [key, label] of [['common_wheel_diameter_mm', '共同轮径候选'], ['effective_track_width_mm', '有效轮距候选']]) {
    if (Number.isFinite(report.candidates?.[key])) metrics.push({ label, value: `${report.candidates[key].toFixed(3)} mm`, note: '待独立复测，未应用' })
  }
  for (const [kind, label, unit] of [['distance', '直行', 'm'], ['turn', '转角', '°']]) {
    const measurement = report.measurements?.[kind]
    if (measurement) metrics.push({ label: `${label}原始最大误差`, value: `${measurement.raw_error.maximum_absolute.toFixed(4)} ${unit}`, note: `${measurement.raw_error.n} 次；${Object.entries(measurement.counts).map(([k, v]) => `${choices[k]} ${v}`).join(' / ')}` })
    const validation = report.independentValidation?.evaluation?.[kind]
    if (validation) metrics.push({ label: `${label}独立复测`, value: kind === 'distance' ? `${(100 * validation.value).toFixed(3)}%` : `${validation.value.toFixed(3)}° / 360°`, note: `${validation.meetsNumericalTarget ? '数值达到目标' : '数值未达目标'}；${validation.sufficientTrials ? '样本量足够' : '样本不足（每方向需 5 次）'}；参数版本待核对` })
  }
  return metrics
})
const choices = { forward: '前进（正）', reverse: '后退（负）', ccw: '逆时针（正）', cw: '顺时针（负）' }
const fieldLabels = { trial_id: '试验编号', direction: '方向', reference_m: '独立参考距离 m', odom_m: '里程计距离 m', end_yaw_deg: '结束累计偏航 °', valid: '试验是否有效', notes: '备注 / 排除原因', reference_deg: '独立参考角度 °', odom_deg: '里程计累计角度 °', odom_source: '转角数据来源', measured_m: '传感器读数 m', sensor: '传感器与角度约定', measured_deg: '读数角度 °', speed_mps: '速度 m/s', stop_distance_m: '停车距离 m', target_x_m: '目标 x m', target_y_m: '目标 y m', target_yaw_deg: '目标朝向 °', actual_x_m: '实测 x m', actual_y_m: '实测 y m', actual_yaw_deg: '实测朝向 °' }
function localTime(value) { return value ? new Date(value).toLocaleString() : '' }
function captureLabel(value) { return { recording: '记录中', complete: '已结束', empty: '未收到数据', failed: '记录失败', interrupted: '后台中断', incomplete: '记录不完整' }[value] || value }
function dimensionLabel(key) {
  const labels = { front: '前缘距离 m', back: '后缘距离 m', left: '左缘距离 m', right: '右缘距离 m', height: '整车高度 m', left_wheel_diameter_m: '左轮径 m', right_wheel_diameter_m: '右轮径 m', mechanical_track_m: '机械轮距 m', axle_center_x_m: '轮轴中点 x m' }
  return key.startsWith('sensor_') ? key.replace('sensor_extrinsics_from_base_link.', '') : labels[key.split('.').at(-1)] || key
}
function flatten(value, prefix = '', output = {}) {
  for (const [key, item] of Object.entries(value || {})) {
    const path = prefix ? `${prefix}.${key}` : key
    if (item && typeof item === 'object') flatten(item, path, output)
    else if (item === null || typeof item === 'number') output[path] = item ?? ''
  }
  return output
}
async function api(route, body) {
  const response = await fetch(base + route, { ...(body !== undefined ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {}), signal: AbortSignal.timeout(body === undefined ? 5000 : 125000) })
  const data = await response.json()
  if (!response.ok || data.success === false) throw new Error(data.error || '标定服务返回错误')
  return data
}
async function action(operation) {
  if (busy.value) return
  busy.value = true; error.value = ''
  try { await operation() } catch (err) { error.value = err.message } finally { busy.value = false }
}
async function refresh() {
  try { const data = await api('/sessions'); sessions.value = data.sessions; simulation.value = data.simulation; job.value = data.job }
  catch (err) { error.value = `标定服务不可用：${err.message}`; job.value = null }
}
async function loadSession(preserve = false) {
  if (!selectedId.value) { session.value = null; return }
  if (!preserve) { session.value = null; result.value = null }
  const id = selectedId.value
  try {
    const data = await api(`/sessions/${id}`)
    if (selectedId.value !== id) return
    session.value = data; job.value = data.job
    if (!preserve) { dimensions.value = flatten(data.dimensions); resetRow(); result.value = null; captureId.value = '' }
  } catch (err) { error.value = err.message }
}
function resetRow() {
  row.value = Object.fromEntries(schema.value.map(key => [key, key === 'trial_id' ? `trial_${Date.now()}` : key === 'valid' ? 'true' : key === 'direction' ? (project.value === 'turn' ? 'ccw' : 'forward') : key === 'odom_source' ? 'unknown' : '']))
  if (project.value === 'camera') cameraEvery.value = 1
}
async function createSession(validationOf) {
  await action(async () => {
    const data = await api('/sessions', { name: name.value.trim() || (validationOf ? '独立复测' : '标定与诊断'), validationOf })
    selectedId.value = data.id; step.value = 0; await refresh(); await loadSession()
  })
}
async function createValidation() {
  const report = selectedReport.value
  if (!report) { error.value = '请先选择当前会话的一份候选报告'; return }
  await createSession({ sessionId: report.sessionId, reportId: report.reportId })
}
async function startCapture() { await action(async () => { job.value = await api(`/sessions/${selectedId.value}/captures`, { seconds: seconds.value, maxMb: maxMb.value, cameraEvery: cameraEvery.value }) }) }
async function stopCapture() { await action(async () => { await api('/stop', {}); await loadSession(true) }) }
async function saveDimensions() { await action(async () => { await api(`/sessions/${selectedId.value}/dimensions`, { values: Object.fromEntries(Object.entries(dimensions.value).map(([k, v]) => [k, v === '' ? null : Number(v)])) }); await loadSession(true) }) }
async function saveMeasurement() { await action(async () => { await api(`/sessions/${selectedId.value}/measurement`, { kind: project.value, row: row.value }); await loadSession(true); resetRow() }) }
async function saveObservation() { await action(async () => { await api(`/sessions/${selectedId.value}/observations`, { project: project.value, note: note.value }); note.value = ''; await loadSession(true) }) }
async function analyze() { await action(async () => { result.value = await api(`/sessions/${selectedId.value}/analyze`, {}); await loadSession(true) }) }
async function cameraCandidate() { await action(async () => { result.value = await api(`/sessions/${selectedId.value}/camera`, { captureId: captureId.value, squareMm: squareMm.value }); await loadSession(true); step.value = 3 }) }
async function captureReport(id, kind) { await action(async () => { result.value = await api(`/sessions/${selectedId.value}/captures/${id}/${kind}`, {}) }) }
async function replayCapture() { await action(async () => { job.value = await api(`/sessions/${selectedId.value}/captures/${captureId.value}/replay`, {}) }) }
function downloadReport() {
  const url = URL.createObjectURL(new Blob([JSON.stringify(result.value, null, 2)], { type: 'application/json' }))
  const anchor = document.createElement('a'); anchor.href = url; anchor.download = `LDC-${selectedId.value}-report.json`; anchor.click(); URL.revokeObjectURL(url)
}
let timer, polling = false
onMounted(() => {
  if (window.location.hash === '#calibration') { open.value = true; refresh() }
  timer = setInterval(async () => { if (open.value && !busy.value && !polling) { polling = true; try { if (selectedId.value) await loadSession(true); else await refresh() } finally { polling = false } } }, 2000)
})
onUnmounted(() => clearInterval(timer))
</script>

<style scoped>
.calibration-entry p { font-size: .85rem; margin-bottom: 10px; }
.calibration-dialog { background: #fff; color: #20334c; width: min(1100px, 96vw); max-height: 92vh; margin: auto; border: 0; overflow: auto; border-radius: 12px; padding: 24px; box-shadow: 0 12px 60px #17263e66; }
.calibration-dialog::backdrop { background: #16253a88; }
header, .session-bar, .actions { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
header { justify-content: space-between; } h2 { font-size: 1.25rem; } h3 { font-size: 1rem; }
small, .muted { color: #64748b; } small { font-size: .75rem; font-weight: normal; }
.session-bar { padding: 14px 0; border-bottom: 1px solid #dde4ed; } .session-bar label { flex: 1; min-width: 190px; }
label { display: flex; flex-direction: column; gap: 5px; font-size: .85rem; margin: 8px 0; }
input, select, textarea { padding: 8px; border: 1px solid #bbc7d6; border-radius: 5px; font: inherit; min-width: 0; background: white; color: #20334c; }
button { padding: 8px 12px; border: 1px solid #bfccdd; border-radius: 5px; background: #f5f8ff; color: #234c8d; cursor: pointer; }
button:disabled { opacity: .45; cursor: default; } .steps { display: flex; gap: 8px; margin: 18px 0; flex-wrap: wrap; }
button[aria-pressed="true"] { background: #e7efff; border-color: #285ed5; }
.steps .active { background: #285ed5; color: white; border-color: #285ed5; } .step-body { padding: 4px 0 16px; }
.fields { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 8px 16px; }
.notice { background: #eff6ff; padding: 10px; border-left: 3px solid #3b82f6; } .error { color: #b91c1c; white-space: pre-wrap; }
table { width: 100%; border-collapse: collapse; margin-top: 12px; font-size: .85rem; } th, td { text-align: left; padding: 8px; border-bottom: 1px solid #e4eaf2; } td button { margin: 2px; }
.report { border-top: 1px solid #dde4ed; padding-top: 12px; } .report pre { max-height: 350px; overflow: auto; background: #f3f6fa; padding: 12px; font-size: .78rem; }
details { margin-top: 12px; } summary { cursor: pointer; } .timeline { max-height: 260px; overflow: auto; font-size: .85rem; padding-left: 26px; }
</style>
