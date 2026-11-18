<template>
  <div class="card" v-if="connected">
    <div class="card-header">
      导航控制
    </div>
    <div class="card-body">
      <!-- 地图选择 -->
      <div class="nav-section">
        <label class="nav-label">选择地图</label>
        <div class="map-select-group">
          <select
            class="map-select"
            v-model="selectedMap"
            :disabled="isNavigating || isLoading"
            @change="onMapChange"
          >
            <option value="" disabled>— 请选择地图 —</option>
            <option
              v-for="m in maps"
              :key="m.path"
              :value="m.path"
              :disabled="m.navigable === false"
              :title="m.navigable === false ? (m.qualityError || '不可导航') : ''"
              :style="m.navigable === false ? 'color: #94a3b8;' : ''"
            >
              {{ m.name }} · {{ mapQualityLabel(m) }}
            </option>
          </select>
          <button class="btn-icon" @click="loadMaps" :disabled="isLoading" title="刷新列表">
            <span :class="['refresh-icon', { spinning: isLoading }]">↻</span>
          </button>
          <button
            v-if="selectedMap || mapChoiceTouched"
            class="btn-icon"
            :disabled="isLoading || isNavigating"
            title="清除选择（允许重新自动选图）"
            @click="clearMapChoice"
          >✕</button>
        </div>
        <div v-if="maps.length === 0 && !isLoading" class="nav-hint">
          暂无地图，请先完成建图并保存
        </div>
        <div v-else-if="selectedMapMissing" class="nav-hint" style="color: #dc2626;">
          所选地图已不可用（{{ selectedMapName }}），请重新选择
        </div>
        <div v-else-if="!selectedMap && !hasNavigableMap" class="nav-hint">
          当前没有可导航的地图（门槛：占用≥{{ thresholds.minOccupiedCells }} 且未知≤{{ thresholdUnknownPercent }}%），请重新建图
        </div>
        <div v-else-if="selectedMapItem && selectedMapItem.navigable === false" class="nav-hint" style="color: #dc2626;">
          不可导航：{{ selectedMapItem.qualityError || '未知原因' }}
        </div>
        <!-- 当前加载地图 / 最近保存地图 / 实时建图 三种状态分开显示 -->
        <div class="map-state-list">
          <div class="map-state-row">
            <span class="map-state-label">实时建图</span>
            <span :class="mappingActive ? 'state-live' : 'state-idle'">{{ mappingActive ? '进行中（/map 实时数据）' : '未运行' }}</span>
          </div>
          <div class="map-state-row">
            <span class="map-state-label">最近保存</span>
            <span v-if="lastSavedMap" class="state-saved">{{ lastSavedMap.name }} · {{ savedAtText }}</span>
            <span v-else class="state-idle">本会话尚未保存</span>
          </div>
          <div class="map-state-row">
            <span class="map-state-label">当前加载</span>
            <span v-if="navigationState.mapPath && navigationState.isRunning" class="state-loaded">{{ navigationState.mapPath.split('/').pop() }}（导航中）</span>
            <span v-else class="state-idle">未加载</span>
          </div>
        </div>
      </div>

      <!-- 启动/停止按钮 -->
      <div class="nav-section">
        <button
          v-if="!isNavigating"
          class="btn btn-connect nav-btn"
          @click="startNavigation"
          :disabled="startBtnDisabled"
          :title="startBtnTitle"
        >
          {{ isLoading ? '正在启动...' : '开始导航' }}
        </button>
        <div v-if="!isNavigating && !canControl" class="nav-hint observer-hint">
          观察模式：其他页面持有控制权；本页只读（急停/取消仍可用）
        </div>
        <button
          v-else
          class="btn btn-disconnect nav-btn"
          @click="stopNavigation"
          :disabled="isLoading"
        >
          {{ isLoading ? '正在停止...' : '停止导航' }}
        </button>
      </div>

      <!-- 导航状态 -->
      <div class="nav-status">
        <span>{{ phaseText }}</span>
        <p v-if="goalBusy && navigationState.feedback?.distance_remaining != null">剩余 {{ navigationState.feedback.distance_remaining.toFixed(2) }} 米</p>
      </div>

      <!-- 就绪检查：雷达/里程计/TF/定位/Nav2/控制连接 + 最近更新 + 下一步（P2-1） -->
      <!-- 启动前也要可见：用户需要先看到 blocker 再决定是否开始 -->
      <ReadinessPanel :status="navigationState" class="mt-2" />

      <!-- 初始位置设置 (仅导航运行中显示) -->
      <div v-if="isNavigating" class="nav-section mt-2">
        <button
          class="nav-btn"
          :class="isSettingPose ? 'btn-pose-active' : 'btn-pose'"
          @click="togglePoseMode"
          :disabled="goalBusy || !canControl"
          :title="goalBusy ? '目标执行中：请先取消当前导航' : (!canControl ? '观察模式：控制权在其他页面' : '')"
        >
          {{ isSettingPose ? '正在设置... (按住拖动)' : '设置初始位置' }}
        </button>
      </div>

      <!-- 多点导航 -->
      <div v-if="isNavigating && !isSettingPose" class="nav-section mt-2">
        <button
          class="nav-btn"
          :class="multiPointMode ? 'btn-waypoint-active' : 'btn-waypoint'"
          @click="toggleMultiPointMode"
          :disabled="goalBusy || !canControl"
          :title="goalBusy ? '目标执行中：切换会取消导航，请先手动取消' : (!canControl ? '观察模式：控制权在其他页面' : '')"
        >
          {{ multiPointMode ? '多点导航: 已开启' : '多点导航: 已关闭' }}
        </button>
        <button v-if="multiPointMode" @click="$emit('execute-waypoints')" :disabled="!navigationState.ready || waypointCount === 0 || goalBusy || !canControl">执行航点队列</button>
        <button @click="cancelGoal" :disabled="!goalBusy">取消当前导航</button>
        <div v-if="multiPointMode" class="waypoint-row">
          <span class="waypoint-count">队列点位: {{ waypointCount }}</span>
          <button class="btn-clear-waypoints" @click="clearWaypoints" :disabled="waypointCount === 0">
            清空
          </button>
        </div>
        <!-- 发送前展示每个航点位置与朝向 -->
        <ol v-if="multiPointMode && (waypoints.length || activeWaypoint)" class="waypoint-list">
          <li v-if="activeWaypoint" class="waypoint-active">
            ▶ 执行中 ({{ activeWaypoint.x.toFixed(2) }}, {{ activeWaypoint.y.toFixed(2) }}) · {{ yawDeg(activeWaypoint) }}°
          </li>
          <li v-for="(point, index) in waypoints" :key="index">
            {{ index + 1 }}. ({{ point.x.toFixed(2) }}, {{ point.y.toFixed(2) }}) · {{ yawDeg(point) }}°
          </li>
        </ol>
      </div>

      <!-- 操作提示 -->
      <div v-if="isNavigating && !isSettingPose" class="nav-hint goal-hint" style="margin-top:4px">
        {{ multiPointMode ? '双击地图连续添加航点，添加后点击执行航点队列' : '双击地图设定导航目标点' }}
      </div>
      <div v-if="isSettingPose" class="nav-hint pose-hint">
        🟡 按住地图拖动以设定位置和朝向
      </div>

      <!-- 无位移失败锁存（D）：必须说清"车没动"和"怎么解除" -->
      <div v-if="stuckLatched" class="error-message mt-2">
        无位移失败已锁存：{{ stuckReason }}。请手动挪开车辆或重新设置初始位置，再点「启用遥控」解除。
      </div>
      <!-- 错误信息 -->
      <div v-if="errorMsg" class="error-message mt-2">
        {{ errorMsg }}
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import ReadinessPanel from './ReadinessPanel.vue'
import { useMapCatalog } from '../composables/useMapCatalog.js'

const props = defineProps({
  connected: Boolean,
  controlVersion: { type: Function, default: () => 0 },
  isSettingPose: Boolean,
  sessionId: {
    type: String,
    default: ''
  },
  controlOwner: {
    type: String,
    default: null
  },
  mappingActive: {
    type: Boolean,
    default: false
  },
  lastSavedMap: {
    type: Object,
    default: null
  },
  waypoints: {
    type: Array,
    default: () => []
  },
  activeWaypoint: {
    type: Object,
    default: null
  },
  multiPointMode: {
    type: Boolean,
    default: false
  },
  waypointCount: {
    type: Number,
    default: 0
  }
})

const emit = defineEmits(['update:isNavigating', 'update:isSettingPose', 'update:multiPointMode', 'navigationStarted', 'clear-waypoints', 'execute-waypoints', 'state'])

const isNavigating = ref(false)
const navigationState = ref({})
const goalBusy = computed(() => ['accepted', 'executing', 'canceling'].includes(navigationState.value.goalState))
const phaseLabels = { stopped: '导航未运行', starting: '正在启动', waiting_localization: '请设置初始位置', ready: '导航就绪', accepted: '目标已受理', executing: '正在导航', canceling: '正在取消', canceled: '已取消', succeeded: '已到达', failed: '导航失败' }
const phaseText = computed(() => phaseLabels[navigationState.value.phase] || '等待控制服务')
// 连续无位移失败后的锁存：只在人工接管（启用遥控）时解除。
const stuckLatched = computed(() => Boolean(navigationState.value.stuckPolicy?.latched))
const stuckReason = computed(() => navigationState.value.stuckPolicy?.latchReason
  || '车辆可能贴墙或在膨胀区内')
const isLoading = ref(false)
const errorMsg = ref('')
// 观察模式：其他页面持有控制权时本页只读（急停/取消/停止仍可用）。
const canControl = computed(() => !props.controlOwner || props.controlOwner === props.sessionId)
const savedAtText = computed(() => props.lastSavedMap?.savedAt
  ? new Date(props.lastSavedMap.savedAt).toLocaleString('zh-CN') : '')
function yawDeg(point) { return Math.round(((point?.yaw || 0) * 180 / Math.PI)) }
function newRequestId() {
  return (typeof crypto !== 'undefined' && crypto.randomUUID)
    ? crypto.randomUUID()
    : `req-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
}
// 启动按钮状态门控
const startBtnDisabled = computed(() => {
  if (isLoading.value || !canControl.value) return true
  const m = selectedMapItem.value
  if (!m) return true
  // 兼容老版本后端：无 navigable 视为合格
  return m.navigable === false
})

// 启动按钮禁用原因（必须把 blocker 说出来，不能只置灰）
const startBtnTitle = computed(() => {
  if (!canControl.value) return '观察模式：控制权在其他页面，需先接管'
  if (isLoading.value) return '请求处理中'
  if (!selectedMap.value) return maps.value.length === 0 ? '暂无地图，请先建图并保存' : '请先选择一张地图'
  if (selectedMapMissing.value) return '所选地图已不可用，请重新选择'
  const m = selectedMapItem.value
  if (!m) return '请先选择一张地图'
  if (m.navigable === false) return `不可导航：${m.qualityError || '未知原因'}`
  return ''
})

const isSettingPose = ref(props.isSettingPose)
const multiPointMode = ref(props.multiPointMode)

const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api`
// 地图目录、质量门控与"用户选择不被轮询覆盖"的语义集中在 useMapCatalog。
const {
  maps, selectedMap, mapChoiceTouched, thresholds, thresholdUnknownPercent,
  selectedMapItem, selectedMapName, selectedMapMissing, hasNavigableMap,
  mapQualityLabel, loadMaps, onMapChange, clearMapChoice, adoptServerMapPath
} = useMapCatalog({ apiBase: API_BASE, isLoading, errorMsg })
let statusTimer = null
let checkingStatus = false
// 卸载标记：卸载后到达的响应不得写状态，onMounted 里的 await 之后也不得再建定时器。
let disposed = false
const pollFailed = ref(false)

async function checkStatus() {
  if (checkingStatus) return
  checkingStatus = true
  const controlVersion = props.controlVersion?.() ?? 0
  try {
    const res = await fetch(`${API_BASE}/navigation_status`)
    const data = await res.json()
    if (disposed) return
    if ((props.controlVersion?.() ?? 0) !== controlVersion) return
    isNavigating.value = data.isRunning
    emit('update:isNavigating', data.isRunning)
    if (pollFailed.value && errorMsg.value === '导航状态不可用，控制服务可能已退出') errorMsg.value = ''
    pollFailed.value = false
    navigationState.value = data
    emit('state', data, controlVersion)
    // 服务端报告的当前地图：未手选时跟随，且差图不得被默认选中。
    adoptServerMapPath(data.mapPath)
    if (data.error) errorMsg.value = data.error
  } catch (err) {
    if (disposed) return
    if ((props.controlVersion?.() ?? 0) !== controlVersion) return
    pollFailed.value = true
    errorMsg.value = '导航状态不可用，控制服务可能已退出'
    // 轮询失败只是后端暂时不可达，不能伪造成"已锁定"去解除遥控；由 3s 新鲜度判定控制链路。
    emit('state', { mode: 'unknown', unreachable: true }, controlVersion)
  } finally { checkingStatus = false }
}

async function startNavigation() {
  if (!selectedMap.value) {
    errorMsg.value = '请先选择一张地图'
    return
  }
  isLoading.value = true
  errorMsg.value = ''
  try {
    const res = await fetch(`${API_BASE}/start_navigation`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      // 统一携带控制会话与请求标识；重复 requestId 幂等重放。
      body: JSON.stringify({ mapPath: selectedMap.value, sessionId: props.sessionId, requestId: newRequestId() })
    })
    const data = await res.json()
    if (data.success) {
      isNavigating.value = true
      emit('update:isNavigating', true)
      emit('navigationStarted')
    } else {
      errorMsg.value = data.error || '启动失败'
    }
  } catch (err) {
    errorMsg.value = '无法连接到本地控制服务'
  } finally {
    isLoading.value = false
  }
}

async function stopNavigation() {
  isLoading.value = true
  errorMsg.value = ''
  // 停止导航时也退出初始位置模式
  if (isSettingPose.value) {
    isSettingPose.value = false
    emit('update:isSettingPose', false)
  }
  if (multiPointMode.value) {
    multiPointMode.value = false
    emit('update:multiPointMode', false)
    emit('clear-waypoints')
  }
  try {
    const res = await fetch(`${API_BASE}/stop_navigation`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sessionId: props.sessionId, requestId: newRequestId() })
    })
    const data = await res.json()
    if (data.success) {
      isNavigating.value = false
      emit('update:isNavigating', false)
    } else {
      errorMsg.value = data.error || '停止失败'
    }
  } catch (err) {
    errorMsg.value = '无法连接到本地控制服务'
  } finally {
    isLoading.value = false
  }
}

async function cancelGoal() {
  try {
    // 取消属于安全类操作：任意页面可用。
    const response = await fetch(`${API_BASE}/navigation/cancel`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sessionId: props.sessionId, requestId: newRequestId() })
    })
    const data = await response.json()
    if (!data.success) throw new Error(data.error)
    await checkStatus()
  } catch (error) { errorMsg.value = error.message }
}

function togglePoseMode() {
  isSettingPose.value = !isSettingPose.value
  emit('update:isSettingPose', isSettingPose.value)
}

function toggleMultiPointMode() {
  multiPointMode.value = !multiPointMode.value
  emit('update:multiPointMode', multiPointMode.value)
  if (!multiPointMode.value) {
    emit('clear-waypoints')
  }
}

function clearWaypoints() {
  emit('clear-waypoints')
}

watch(() => props.isSettingPose, value => { isSettingPose.value = value })

watch(() => props.multiPointMode, (value) => {
  multiPointMode.value = value
})

function onMapsUpdated(event) {
  if (event.detail?.path) selectedMap.value = event.detail.path
  loadMaps()
}

onMounted(async () => {
  window.addEventListener('maps-updated', onMapsUpdated)
  await loadMaps()
  await checkStatus()
  // 两次 await 期间组件可能已经卸载：此时再 setInterval 会留下无人清理的轮询。
  if (disposed) return
  statusTimer = setInterval(checkStatus, 500)
})

onUnmounted(() => {
  disposed = true
  window.removeEventListener('maps-updated', onMapsUpdated)
  if (statusTimer) clearInterval(statusTimer)
})
</script>

<style scoped>
.nav-section {
  margin-bottom: 12px;
}

.nav-label {
  display: block;
  font-size: 0.78rem;
  color: var(--text-muted);
  text-transform: uppercase;
    margin-bottom: 6px;
}

.map-select-group {
  display: flex;
  gap: 6px;
  align-items: center;
}

.map-select {
  flex: 1;
  padding: 8px 10px;
  background: #ffffff;
  border: 1px solid var(--border-color);
  border-radius: var(--radius-sm);
  color: var(--text-primary);
  font-family: var(--font-mono);
  font-size: 0.8rem;
  outline: none;
  cursor: pointer;
  transition: var(--transition);
}

.map-select:focus {
  border-color: var(--accent-primary);
  box-shadow: 0 0 0 3px rgba(99, 102, 241, 0.15);
}

.map-select:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.map-select option {
  background: #ffffff;
  color: #0f172a;
}

.btn-icon {
  width: 34px;
  height: 34px;
  background: rgba(99, 102, 241, 0.1);
  border: 1px solid var(--border-color);
  border-radius: var(--radius-sm);
  color: var(--text-secondary);
  font-size: 1rem;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: var(--transition);
  flex-shrink: 0;
}

.btn-icon:hover {
  background: rgba(99, 102, 241, 0.2);
  color: var(--text-primary);
}

.refresh-icon {
  display: inline-block;
  transition: transform 0.5s ease;
}

.refresh-icon.spinning {
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  from { transform: rotate(0deg); }
  to { transform: rotate(360deg); }
}

.nav-btn {
  width: 100%;
  font-weight: 600;
  padding: 12px;
}

.nav-status {
  margin-top: 4px;
  font-size: 0.82rem;
}

.status-nav-active {
  color: #15803d;
}

.status-nav-idle {
  color: #64748b;
}

.nav-hint {
  margin-top: 6px;
  font-size: 0.78rem;
  color: var(--text-muted);
}

.goal-hint {
  color: #2563eb;
  background: rgba(99, 102, 241, 0.08);
  border: 1px solid rgba(99, 102, 241, 0.2);
  border-radius: var(--radius-sm);
  padding: 6px 10px;
}

.pose-hint {
  color: #b45309;
  background: rgba(251, 191, 36, 0.08);
  border: 1px solid rgba(251, 191, 36, 0.25);
  border-radius: var(--radius-sm);
  padding: 6px 10px;
}

.btn-pose {
  width: 100%;
  padding: 10px;
  font-weight: 600;
  font-size: 0.82rem;
  background: rgba(251, 191, 36, 0.1);
  border: 1px solid rgba(251, 191, 36, 0.3);
  color: #b45309;
  border-radius: var(--radius-sm);
  cursor: pointer;
  transition: var(--transition);
}

.btn-pose:hover {
  background: rgba(251, 191, 36, 0.2);
}

.btn-pose-active {
  width: 100%;
  padding: 10px;
  font-weight: 600;
  font-size: 0.82rem;
  background: rgba(251, 191, 36, 0.25);
  border: 1px solid #d97706;
  color: #b45309;
  border-radius: var(--radius-sm);
  cursor: pointer;
  animation: pose-pulse 1.2s ease-in-out infinite;
}

.btn-waypoint {
  width: 100%;
  padding: 10px;
  font-weight: 600;
  font-size: 0.82rem;
  background: rgba(34, 197, 94, 0.12);
  border: 1px solid rgba(34, 197, 94, 0.35);
  color: #15803d;
  border-radius: var(--radius-sm);
  cursor: pointer;
  transition: var(--transition);
}

.btn-waypoint:hover {
  background: rgba(34, 197, 94, 0.2);
}

.btn-waypoint-active {
  width: 100%;
  padding: 10px;
  font-weight: 600;
  font-size: 0.82rem;
  background: rgba(34, 197, 94, 0.25);
  border: 1px solid #16a34a;
  color: #15803d;
  border-radius: var(--radius-sm);
  cursor: pointer;
}

.waypoint-row {
  margin-top: 8px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
}

.waypoint-count {
  font-size: 0.78rem;
  color: var(--text-secondary);
}

.btn-clear-waypoints {
  padding: 4px 10px;
  font-size: 0.75rem;
  border-radius: 6px;
  border: 1px solid rgba(239, 68, 68, 0.35);
  background: rgba(239, 68, 68, 0.15);
  color: #dc2626;
  cursor: pointer;
}

.btn-clear-waypoints:disabled {
  opacity: 0.45;
  cursor: not-allowed;
}

@keyframes pose-pulse {
  0%, 100% { box-shadow: 0 0 0 0 rgba(251, 191, 36, 0.4); }
  50% { box-shadow: 0 0 0 6px rgba(251, 191, 36, 0); }
}

.mt-2 { margin-top: 8px; }

.map-state-list { margin-top: 8px; border-top: 1px dashed var(--border-color); padding-top: 6px; }
.map-state-row { display: flex; justify-content: space-between; gap: 8px; font-size: 0.75rem; padding: 2px 0; }
.map-state-label { color: var(--text-muted); white-space: nowrap; }
.state-live { color: #15803d; font-weight: 600; }
.state-saved { color: #2563eb; }
.state-loaded { color: #7c3aed; font-weight: 600; }
.state-idle { color: var(--text-muted); }
.observer-hint { color: #b45309; background: rgba(251, 191, 36, 0.08); border: 1px solid rgba(251, 191, 36, 0.25); border-radius: var(--radius-sm); padding: 5px 8px; margin-top: 6px; }
.waypoint-list { margin: 8px 0 0; padding-left: 20px; font-size: 0.75rem; font-family: var(--font-mono); color: var(--text-secondary); max-height: 110px; overflow-y: auto; }
.waypoint-list li { padding: 1px 0; }
.waypoint-active { color: #f97316; font-weight: 600; list-style: none; margin-left: -20px; }
</style>
