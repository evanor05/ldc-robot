<template>
  <div class="card camera-card">
    <div class="card-header">摄像头与 YOLO <span v-if="state.simulation"> · 仿真</span></div>
    <div class="card-body camera-body">
      <div class="camera-frame">
        <!-- 放大时卸载主图：上游 MJPEG 每条预览只允许一条连接，双路并发会互相抢带宽导致放大后卡顿 -->
        <img v-if="previewUrl && !zoomOpen" :key="imageKey" class="camera-stream" :src="previewUrl" alt="摄像头画面"
          decoding="async" title="点击放大" @click="zoomOpen = true"
          @load="streamLoaded = true" @error="streamFailed">
        <button v-if="previewUrl && !zoomOpen" class="zoom-btn" type="button" title="放大画面" aria-label="放大画面"
          @click="zoomOpen = true">⤢ 放大</button>
        <div v-if="zoomOpen && previewUrl" class="camera-placeholder">画面已在放大视图中播放</div>
        <div v-else-if="!streamLoaded || !previewUrl" class="camera-placeholder">{{ previewMessage }}</div>
      </div>

      <Teleport to="body">
        <div v-if="zoomOpen" class="camera-lightbox" role="dialog" aria-modal="true" aria-label="放大摄像头画面"
          @click.self="zoomOpen = false">
          <div class="lightbox-bar">
            <span class="lightbox-title">摄像头放大 · {{ previewKind === 'raw' ? '原图' : 'YOLO 识别图' }}</span>
            <div class="lightbox-actions">
              <button class="lb-switch" type="button" :class="{ active: previewKind === 'raw' }"
                @click="previewKind = 'raw'">🖼 原图</button>
              <button class="lb-switch" type="button" :class="{ active: previewKind === 'annotated' }"
                @click="previewKind = 'annotated'">🎯 YOLO 识别图</button>
              <button class="lb-refresh" type="button" title="重建画面连接（画面卡住时点这里）"
                @click="reloadStream">↻ 刷新画面</button>
              <button class="lb-close" type="button" @click="zoomOpen = false">✕ 退出 (Esc)</button>
            </div>
          </div>
          <div class="lightbox-stage">
            <div class="lightbox-joy" @click.stop>
              <div class="joy-title">遥控摇杆</div>
              <div class="joy-drive-btns">
                <button class="joy-enable" type="button"
                  :disabled="!joystickCanEnable || joystickManualEnabled"
                  @click="emit('joystick-enable')">{{ joystickManualEnabled ? '遥控已启用' : '启用遥控 / 解除急停' }}</button>
                <button class="joy-estop" type="button"
                  :disabled="!joystickCanEstop" @click="emit('joystick-estop')">急停</button>
              </div>
              <JoystickControl
                :connected="joystickConnected"
                :max-linear-speed="joystickMaxLinear"
                :max-angular-speed="joystickMaxAngular"
                :lock-reason="joystickLockReason"
                :lock-hint="joystickLockHint"
                @velocity-change="v => emit('velocity-change', v)"
              />
            </div>
            <div class="lightbox-view">
              <div class="lightbox-canvas">
                <span v-if="state.running" class="lightbox-fps">
                  <span class="lightbox-fps-num">{{ fpsText }}</span>
                  <span class="lightbox-fps-unit">FPS</span>
                </span>
                <img v-if="previewUrl" :key="'zoom' + imageKey" class="lightbox-img" :src="previewUrl" alt="摄像头放大画面"
                  decoding="async" @error="streamFailed">
                <div v-else class="lightbox-empty">{{ previewMessage }}</div>
              </div>
            </div>
          </div>
        </div>
      </Teleport>
      <div class="vision-status" aria-live="polite">
        <div class="status-line">
          <span class="status-key">摄像头</span>
          <span class="status-val" :class="cameraStatusClass">
            {{ cameraStatusText }}
          </span>
        </div>
        <div class="status-line">
          <span class="status-key">识别</span>
          <span class="status-val" :class="state.detector?.ok ? 'is-ok' : 'is-off'">
            {{ state.detector?.ok ? '运行中' : state.detector?.detail || '未启动' }}
          </span>
        </div>

        <div v-if="state.running" class="gauge">
          <div class="gauge-cell">
            <span class="gauge-num">{{ fpsText }}</span>
            <span class="gauge-unit">FPS</span>
            <span class="gauge-cap">实时帧率</span>
          </div>
          <div class="gauge-cell">
            <span class="gauge-num">{{ latencyText }}</span>
            <span class="gauge-unit">ms</span>
            <span class="gauge-cap">p95 延迟</span>
          </div>
          <div class="gauge-cell">
            <span class="gauge-num gauge-chip">{{ state.device === 'cpu' ? 'CPU' : 'GPU' }}</span>
            <span class="gauge-cap">推理设备</span>
          </div>
          <div class="latency-track" :title="`p95 ${latencyText} ms · 健康线 ${latencyBudget} ms`">
            <div class="latency-fill" :class="latencyState" :style="{ width: latencyPercent + '%' }"></div>
            <div class="latency-tick" :style="{ left: latencyBudgetPercent + '%' }"></div>
          </div>
        </div>

        <small class="status-note">主机接收时间，非曝光时间；仅识别显示，不控制车辆。</small>
      </div>
      <div class="camera-controls">
        <input v-model.trim="draftUrl" :disabled="busy || managed" class="url-input camera-url-input" type="text" placeholder="MJPEG 地址，留空自动发现">
        <button class="btn camera-btn camera-btn-secondary" type="button" :disabled="busy || managed" @click="emit('discover')">自动发现</button>
        <select v-model="device" :disabled="busy || managed" aria-label="推理设备"><option value="0">GPU 0</option><option value="cpu">CPU</option></select>
        <button class="btn camera-btn" type="button" :disabled="busy" @click="managed ? stopVision() : startVision()">{{ busy ? '处理中…' : managed ? '停止视觉' : '启动视觉' }}</button>
      </div>
      <div class="camera-controls">
        <select v-model="previewKind" aria-label="画面类型"><option value="raw">原图</option><option value="annotated">YOLO 识别图</option></select>
        <select v-model="classFilter" :disabled="busy" aria-label="类别过滤" @change="changeFilter"><option value="all">全部通用类别</option><option value="traffic">交通灯 / STOP</option></select>
        <button class="btn camera-btn camera-btn-secondary" type="button" @click="reloadStream">刷新画面</button>
      </div>
      <div v-if="error || state.error" class="vision-error" role="alert">{{ error || state.error }}</div>
      <div class="vision-targets">
        <strong>检测目标</strong>
        <p v-if="!state.detector?.ok">识别服务未就绪，当前无有效检测结果。</p>
        <p v-else-if="!state.detections?.length">当前帧未检测到所选类别目标。</p>
        <ul v-else><li v-for="(target, index) in state.detections" :key="`${state.frame?.id}:${index}`">
          {{ target.name }} · {{ (target.score * 100).toFixed(1) }}%<span v-if="target.name === 'traffic light'"> · 灯色未判定</span>
        </li></ul>
        <small>未标定时不提供距离；其他交通标志和中文“停”字不默认支持。</small>
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed, nextTick, onMounted, onUnmounted, ref, watch } from 'vue'
import JoystickControl from './JoystickControl.vue'
import { useCameraHealth } from '../composables/useCameraHealth.js'
const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api/vision`
const emit = defineEmits(['discover', 'velocity-change', 'joystick-enable', 'joystick-estop'])
const props = defineProps({
  defaultUrl: { type: String, default: '' },
  joystickConnected: { type: Boolean, default: false },
  joystickMaxLinear: { type: Number, default: 0.1 },
  joystickMaxAngular: { type: Number, default: 0.5 },
  joystickLockReason: { type: String, default: '' },
  joystickLockHint: { type: String, default: '' },
  joystickManualEnabled: { type: Boolean, default: false },
  joystickCanEnable: { type: Boolean, default: false },
  joystickCanEstop: { type: Boolean, default: false }
})
const draftUrl = ref(props.defaultUrl)
const device = ref('0')
const previewKind = ref('annotated')
const classFilter = ref('all')
const imageKey = ref(0)
const streamLoaded = ref(false)
const busy = ref(false)
const error = ref('')
const zoomOpen = ref(false)
let disposed = false, timer, retryTimer, polling = false
// 相机 health / 双路看门狗 / 离线计时集中在 useCameraHealth，组件只负责取流与 UI。
const {
  state, managed, frameWatch, offlineSeconds,
  cameraStatusClass, cameraStatusText, previewHealthy, observe, resetWatch
} = useCameraHealth({ previewKind })
// All previews use the local gateway, including raw-only viewing. No browser
// opens a second connection to the ESP32 while ROS owns the acquisition stream.
const previewUrl = computed(() => (managed.value && previewHealthy.value
  ? `${API_BASE}/${previewKind.value}.mjpg?v=${imageKey.value}` : ''))
const previewMessage = computed(() => previewUrl.value ? '等待实时画面…' : managed.value ? '等待有效图像，请查看摄像头和识别状态' : '点击“启动视觉”，原图和识别图均由本机网关提供')

// 视觉读数：帧率与 p95 延迟分列呈现，延迟按健康区间着色
const latencyBudget = 200                       // p95 超过 200ms 判为吃紧
const fpsText = computed(() => {
  const v = Number(state.value.fps)
  return Number.isFinite(v) && v > 0 ? v.toFixed(1) : '—'
})
const latencyText = computed(() => {
  const v = state.value.p95Ms
  return v == null || !Number.isFinite(Number(v)) ? '—' : Number(v).toFixed(0)
})
const latencyPercent = computed(() => {
  const v = Number(state.value.p95Ms)
  if (!Number.isFinite(v) || v <= 0) return 0
  return Math.min(100, (v / (latencyBudget * 2)) * 100)   // 满格 = 2 倍健康线
})
const latencyBudgetPercent = computed(() => 50)          // 健康线落在轨道中点
const latencyState = computed(() => {
  const v = Number(state.value.p95Ms)
  if (!Number.isFinite(v) || v <= 0) return 'is-idle'
  if (v <= latencyBudget) return 'is-ok'
  return v <= latencyBudget * 1.5 ? 'is-warn' : 'is-bad'
})
async function request(path, body) {
  const res = await fetch(`${API_BASE}/${path}`, {
    method: body === undefined ? 'GET' : 'POST', signal: AbortSignal.timeout(path === 'start' ? 15000 : 10000),
    ...(body === undefined ? {} : { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
  })
  const result = await res.json()
  if (!res.ok) throw new Error(result.error || `HTTP ${res.status}`)
  return result
}
async function poll() {
  if (polling || disposed) return
  polling = true
  try {
    const current = await request('status')
    if (disposed) return
    state.value = current
    if (current.filter) classFilter.value = current.filter
    if (current.device && !busy.value) device.value = current.device
    // 健康判定/离线计时/双路看门狗统一由 composable 计算；只在"当前这一路卡住"时重建连接。
    if (observe(current)) reloadStream()
  } catch (e) {
    // 卸载后到达的失败响应不得再写已销毁组件：success 分支早有 disposed 守卫，
    // 这里此前漏了（impl-frontend 报告，Lead 确认并补最小守卫）。
    if (!disposed) state.value = { running: false, detections: [], error: `视觉状态连接失败：${e.message}` }
  } finally { polling = false }
}
async function startVision() {
  busy.value = true; error.value = ''; streamLoaded.value = false
  // Unmount any prior preview before asking ROS to acquire the camera.
  state.value = { running: false, detections: [] }
  await nextTick()
  try { state.value = await request('start', { url: draftUrl.value, device: device.value, filter: classFilter.value }); await poll() }
  catch (e) { error.value = e.message }
  finally { busy.value = false }
}
async function stopVision() {
  busy.value = true; error.value = ''
  try { await request('stop', {}); await poll() }
  catch (e) { error.value = e.message }
  finally { busy.value = false }
}
async function changeFilter() {
  if (!managed.value) return
  busy.value = true; error.value = ''
  try { await request('filter', { filter: classFilter.value }); await poll() }
  catch (e) { error.value = e.message }
  finally { busy.value = false }
}
function streamFailed() {
  streamLoaded.value = false
  clearTimeout(retryTimer)
  retryTimer = setTimeout(() => { if (!disposed && previewUrl.value) imageKey.value++ }, 1000)
}
function reloadStream() {
  streamLoaded.value = false
  resetWatch()
  imageKey.value++
}
watch(previewKind, reloadStream)
watch(previewUrl, () => { streamLoaded.value = false })
watch(() => props.defaultUrl, url => { if (!managed.value) draftUrl.value = url })
function onKeydown(e) { if (e.key === 'Escape' && zoomOpen.value) zoomOpen.value = false }
function onVisibilityChange() { if (!document.hidden) reloadStream() }
onMounted(() => { poll(); timer = setInterval(poll, 500); window.addEventListener('keydown', onKeydown); document.addEventListener('visibilitychange', onVisibilityChange) })
onUnmounted(() => { disposed = true; clearInterval(timer); clearTimeout(retryTimer); window.removeEventListener('keydown', onKeydown); document.removeEventListener('visibilitychange', onVisibilityChange) })
</script>

<style scoped>
/* 画面容器：等比居中，不裁剪、不按原始像素溢出 */
.camera-frame {
  flex: none; height: 260px; position: relative;
  display: flex; align-items: center; justify-content: center;
  overflow: hidden; background: #0b0f14;
}
.camera-stream {
  width: 100%; height: 100%; object-fit: contain;
  display: block; cursor: zoom-in;
}
.zoom-btn {
  position: absolute; top: .4rem; right: .4rem; z-index: 2;
  padding: .2rem .5rem; font-size: .78rem; border-radius: 6px; cursor: zoom-in;
  border: 1px solid rgba(255,255,255,.6); background: rgba(15,23,42,.55); color: #fff;
  backdrop-filter: blur(4px);
}
.zoom-btn:hover { background: rgba(15,23,42,.8); }
.camera-lightbox {
  position: fixed; inset: 0; z-index: 9999; display: flex; flex-direction: column;
  background: rgba(2,6,23,.92); padding: 1rem; box-sizing: border-box;
}
.lightbox-bar {
  flex: none; display: flex; align-items: center; justify-content: space-between;
  gap: .75rem; color: #e2e8f0; margin-bottom: .6rem; flex-wrap: wrap;
}
.lightbox-title { font-size: 1rem; font-weight: 600; }
.lightbox-actions { display: flex; gap: .5rem; flex-wrap: wrap; align-items: center; }
.lb-switch {
  padding: .5rem 1rem; font-size: .95rem; font-weight: 700; border-radius: 999px; cursor: pointer;
  border: 2px solid #475569; background: #1e293b; color: #cbd5e1; transition: all .15s ease;
}
.lb-switch:hover { border-color: #64748b; color: #fff; background: #334155; }
.lb-switch.active { background: #2563eb; border-color: #3b82f6; color: #fff; box-shadow: 0 0 0 3px rgba(37,99,235,.35); }
.lb-refresh {
  padding: .5rem 1rem; font-size: .95rem; font-weight: 700; border-radius: 999px; cursor: pointer;
  border: 2px solid #475569; background: #1e293b; color: #cbd5e1; transition: all .15s ease;
}
.lb-refresh:hover { border-color: #64748b; color: #fff; background: #334155; }
.lb-refresh:active { background: #0f172a; }
.lb-close {
  padding: .5rem 1.1rem; font-size: .95rem; font-weight: 700; border-radius: 999px; cursor: pointer;
  border: 2px solid #ef4444; background: #dc2626; color: #fff; transition: all .15s ease;
}
.lb-close:hover { background: #ef4444; box-shadow: 0 0 0 3px rgba(239,68,68,.35); }
.lightbox-stage {
  flex: 1; min-height: 0; display: flex; flex-direction: row; gap: 1rem; align-items: stretch;
}
.lightbox-joy {
  flex: none; display: flex; flex-direction: column; align-items: center; justify-content: center;
  gap: .5rem; width: 260px; max-width: 32vw;
}
.lightbox-joy .joy-title {
  font-size: .9rem; font-weight: 700; color: #cbd5e1; letter-spacing: 1px;
}
.joy-drive-btns { display: flex; gap: .4rem; width: 100%; }
.joy-enable, .joy-estop {
  flex: 1; padding: .5rem .3rem; font-size: .82rem; font-weight: 700; border-radius: 8px;
  cursor: pointer; border: 2px solid transparent; transition: all .15s ease; white-space: nowrap;
}
.joy-enable { background: #16a34a; border-color: #22c55e; color: #fff; }
.joy-enable:hover:not(:disabled) { background: #22c55e; }
.joy-enable:disabled { background: #334155; border-color: #475569; color: #94a3b8; cursor: not-allowed; }
.joy-estop { background: #dc2626; border-color: #ef4444; color: #fff; }
.joy-estop:hover:not(:disabled) { background: #ef4444; }
.joy-estop:disabled { background: #334155; border-color: #475569; color: #94a3b8; cursor: not-allowed; }
/* 摇杆卡片在灯箱深色背景里淡化外框、自适应宽度 */
.lightbox-joy :deep(.card) { background: rgba(30,41,59,.6); border: 1px solid rgba(148,163,184,.25); width: 100%; }
.lightbox-joy :deep(.card-header) { color: #e2e8f0; }
.lightbox-view {
  flex: 1; min-width: 0; position: relative; display: flex; align-items: center; justify-content: center;
}
@media (max-width: 760px) {
  .lightbox-stage { flex-direction: column-reverse; }
  .lightbox-joy { width: 100%; max-width: 100%; }
}
/* 画面包裹层：宽度贴合图片本身，使左上角的 FPS 锚定在"画面"左上角，而不是容器左上角 */
.lightbox-canvas {
  position: relative; display: inline-flex; align-items: center; justify-content: center;
  max-width: 100%; max-height: 100%;
}
.lightbox-fps {
  position: absolute; top: .55rem; left: .55rem; z-index: 2;
  display: inline-flex; align-items: baseline; gap: .3rem;
  padding: .3rem .62rem; color: #e2e8f0; line-height: 1;
  background: rgba(2,6,23,.66); border-radius: var(--radius-sm);
  backdrop-filter: blur(4px);
}
.lightbox-fps-num {
  font-family: var(--font-mono); font-variant-numeric: tabular-nums;
  font-size: 1.2rem; font-weight: 600; line-height: 1;
}
.lightbox-fps-unit {
  font-size: .72rem; font-weight: 600; letter-spacing: .08em;
  text-transform: uppercase; color: #94a3b8;
}
/* 画面尺寸由视口决定，等比铺满可用空间并居中 */
.lightbox-img {
  max-width: calc(100vw - 6rem); max-height: calc(100vh - 6rem);
  width: auto; height: auto; object-fit: contain;
  border-radius: 6px; box-shadow: 0 8px 40px rgba(0,0,0,.5); display: block;
}
.lightbox-empty { color: #94a3b8; font-size: .95rem; line-height: 1.5; }
.vision-status, .vision-targets { margin-top: .65rem; font-size: .85rem; line-height: 1.6; }
.vision-status small, .vision-targets small { color: var(--text-muted); }

/* 状态行：左标签右值的两栏对齐，靠字色区分健康与否 */
.status-line { display: flex; align-items: baseline; gap: .5rem; padding: .1rem 0; }
.status-key { flex: none; width: 3.2em; color: var(--text-muted); font-size: .8rem; }
.status-val { color: var(--text-secondary); }
.status-val.is-ok { color: #0f766e; font-weight: 600; }
.status-val.is-off { color: var(--text-muted); }

/* 仪表读数：三列数字 + 一条延迟刻度，替换原先中点拼接的单行文本 */
.gauge {
  display: grid; grid-template-columns: repeat(3, auto) 1fr;
  align-items: end; gap: 0 1.1rem;
  margin: .55rem 0 .3rem; padding: .6rem .7rem .5rem;
  border: 1px solid var(--border-color); border-radius: var(--radius-md);
  background: var(--bg-secondary);
}
.gauge-cell { display: flex; flex-direction: column; gap: .1rem; }
.gauge-num {
  font-family: var(--font-mono); font-variant-numeric: tabular-nums;
  font-size: 1.45rem; font-weight: 600; line-height: 1; color: var(--text-primary);
  letter-spacing: -.02em;
}
.gauge-chip {
  font-size: .95rem; font-weight: 600; letter-spacing: .04em;
  color: var(--accent-primary); padding: .28rem 0 0;
}
.gauge-unit {
  font-size: .68rem; font-weight: 600; letter-spacing: .06em;
  color: var(--text-muted); text-transform: uppercase;
  align-self: flex-start; margin-top: -.15rem;
}
.gauge-cap { font-size: .7rem; color: var(--text-muted); }

/* 延迟刻度尺：轨道 + 填充 + 健康线刻度 */
.latency-track {
  position: relative; align-self: center; height: 6px; min-width: 5rem;
  border-radius: 3px; background: #dbe3ec; overflow: hidden;
}
.latency-fill { height: 100%; border-radius: 3px; transition: width .4s ease, background .3s ease; }
.latency-fill.is-ok { background: #0f766e; }
.latency-fill.is-warn { background: var(--warning); }
.latency-fill.is-bad { background: var(--danger); }
.latency-fill.is-idle { background: transparent; }
.latency-tick {
  position: absolute; top: -1px; bottom: -1px; width: 2px;
  background: var(--text-muted); opacity: .5;
}
.status-note { display: block; margin-top: .35rem; }
@media (prefers-reduced-motion: reduce) {
  .latency-fill { transition: none; }
}
.vision-error { color: #b42318; padding: .5rem 0; }
.camera-controls { flex-wrap: wrap; gap: .4rem; }
.vision-targets ul { max-height: 12rem; overflow: auto; padding-left: 1.2rem; }
</style>
