<template>
  <div class="console-layout">
    <!-- 顶部导航栏 -->
    <header class="console-header">
      <h1>LDC 机器人中控台 <strong v-if="simulation">· 仿真</strong></h1>
      <div class="ownership-indicator">
        <span v-if="isOwner" class="owner-badge owner-self" title="运动租约绑定本页面">控制页</span>
        <span v-else-if="controlOwner" class="owner-badge owner-other" title="其他页面持有运动租约，本页只读；任意页面仍可急停">观察页 · 控制权在其他页面</span>
        <span v-else class="owner-badge owner-free" title="当前无控制页面">无控制页面</span>
        <button v-if="!isOwner" class="btn-takeover" :disabled="!isConnected" @click="takeoverControl">
          {{ controlOwner ? '接管控制权' : '取得控制权' }}
        </button>
        <button v-else class="btn-takeover" :disabled="!isConnected" @click="releaseControl">释放控制权</button>
      </div>
      <div class="status-indicator">
        <span class="status-dot" :class="vehicleConnectionState"></span>
        <span>{{ statusText }}</span>
      </div>
    </header>

    <!-- 全局锁定横幅：急停锁 / 遥控未启用 -->
    <div v-if="lockedBanner" class="locked-banner" :class="`tone-${lockedBanner.tone}`" role="status">
      <span class="locked-dot"></span>
      <span class="locked-text">{{ lockedBanner.text }}</span>
      <button class="locked-action" :disabled="!canControl || !vehicleOnline || !controlLinkOk || !isConnected"
        @click="setEmergencyStop(false)">{{ lockedBanner.action }}</button>
    </div>

    <!-- 主体三栏布局 -->
    <main class="console-body">
      <!-- 左栏: 摇杆 -->
      <section class="left-panel">
        <div class="card drive-card is-collapsible" :class="{ collapsed: driveCardCollapsed }">
          <div class="card-header drive-card-header" @click="driveCardCollapsed = !driveCardCollapsed">
            <span>遥控 · {{ speedProfile.linear.toFixed(2) }} m/s · {{ speedProfile.angular }} rad/s</span>
            <span class="collapse-caret">{{ driveCardCollapsed ? '▸' : '▾' }}</span>
          </div>
          <div class="card-body" v-show="!driveCardCollapsed">
            <div class="drive-selects">
              <label>速度档位
                <select v-model="speedId" aria-label="速度档位">
                  <option v-for="profile in speedProfiles" :key="profile.id" :value="profile.id">{{ profile.label }}</option>
                </select>
              </label>
              <label>操作方式
                <select v-model="driveMode" aria-label="操作方式">
                  <option value="hold">按住连续走，松开停</option>
                  <option value="cruise">点击持续走，停车结束</option>
                </select>
              </label>
            </div>
            <div class="drive-actions">
              <button class="drive-enable" :disabled="!controlLinkOk || !vehicleOnline || !canControl || manualEnabled || !isConnected" @click="setEmergencyStop(false)">{{ manualEnabled ? '遥控已启用' : '启用遥控 / 解除急停' }}</button>
              <button class="drive-estop" @click="setEmergencyStop(true)">急停</button>
            </div>
            <p v-if="controlError" class="error-message">{{ controlError }}</p>
            <p v-if="staleManualSession" class="error-message">刷新前的遥控会话已失效（后端已归零速度）；请重新点击“启用遥控”</p>
            <p v-if="!canControl" class="observer-hint">观察模式：其他页面持有控制权；本页只读，但可随时急停</p>
            <p class="drive-hint">{{ manualEnabled ? (driveMode === 'hold' ? '按住方向按钮或 WASD 持续移动，松开停车' : '点击方向持续行驶；点击停车结束，网页失焦也会停车') : '遥控已锁定，请先启用；切档后需重新启用' }}</p>
            <div class="drive-grid">
              <button v-for="direction in driveDirections" :key="direction.key"
                :disabled="!controlLinkOk || !vehicleOnline || !manualEnabled || !isConnected"
                @pointerdown.prevent="startDirection($event, direction.key)"
                @pointerup="endDirection" @pointercancel="stopKeyboardControl()"
                @lostpointercapture="endDirection"
                @contextmenu.prevent>{{ direction.label }}</button>
              <button @click="setEmergencyStop(true)">停车</button>
            </div>
          </div>
        </div>
        <JoystickControl
          :connected="controlLinkOk && isConnected && manualEnabled"
          :max-linear-speed="speedProfile.linear"
          :max-angular-speed="speedProfile.angular"
          :lock-reason="joystickLockReason"
          :lock-hint="joystickLockHint"
          @velocity-change="onVelocityChange"
        />

        <CameraView
          :default-url="cameraStreamUrl"
          :joystick-connected="isConnected && controlLinkOk && manualEnabled"
          :joystick-max-linear="speedProfile.linear"
          :joystick-max-angular="speedProfile.angular"
          :joystick-lock-reason="joystickLockReason"
          :joystick-lock-hint="manualEnabled ? '' : '点上方“启用遥控 / 解除急停”'"
          :joystick-manual-enabled="manualEnabled"
          :joystick-can-enable="isConnected && controlLinkOk && vehicleOnline && canControl && !manualEnabled"
          :joystick-can-estop="true"
          @velocity-change="onVelocityChange"
          @joystick-enable="setEmergencyStop(false)"
          @joystick-estop="setEmergencyStop(true)"
          @discover="discoverCameraUrl"
        />
      </section>

      <!-- 中栏: 地图 -->
      <section class="center-panel">
        <SlamMap
          ref="slamMapRef"
          :session-id="sessionId"
          :connected="isConnected"
          :mapData="mapData"
          :costmapData="costmapData"
          :scanData="vehicleOnline ? scanData : null"
          :tfData="tfData"
          :planData="planData"
          :layers="mapLayers"
          :isMapping="mappingActive"
          :isNavigating="navigationActive"
          :canSetGoal="canSendGoal"
          v-model:isSettingPose="isSettingPose"
          :multiPointMode="multiPointMode"
          :waypoints="waypointQueue"
          :activeWaypoint="activeWaypoint"
          :pendingGoal="pendingGoal"
          :navFeedback="navFeedback"
          @goal-set="onGoalSet"
          @waypoint-add="onWaypointAdd"
          @initial-pose-set="onInitialPoseSet"
        />
        <!-- 目标预览：发送前展示位置与朝向（P2-4） -->
        <div v-if="pendingGoal" class="card goal-preview">
          <div class="card-header">目标预览 · 发送前确认</div>
          <div class="card-body preview-body">
            <span class="preview-coords">位置 x = {{ pendingGoal.x.toFixed(2) }} m，y = {{ pendingGoal.y.toFixed(2) }} m</span>
            <label class="preview-yaw">朝向 {{ yawDegrees }}°
              <input type="range" min="-180" max="180" step="5" :value="yawDegrees"
                @input="setPendingYaw($event.target.value)" aria-label="目标朝向">
            </label>
            <div class="preview-actions">
              <button class="preview-send" :disabled="!canSendGoal" @click="confirmPendingGoal">发送目标</button>
              <button @click="rotatePending(-90)">-90°</button>
              <button @click="rotatePending(90)">+90°</button>
              <button class="preview-cancel" @click="cancelPendingGoal">取消</button>
            </div>
            <p v-if="!canSendGoal" class="preview-hint">{{ previewBlockReason }}</p>
          </div>
        </div>
      </section>

      <!-- 右栏: 连接 + 电量 + 话题 -->
      <section class="right-panel">
        <ConnectionPanel
          :connectionState="connectionState"
          :vehicle-online="vehicleOnline"
          :vehicle-status-text="statusText"
          :errorMessage="errorMessage"
          @connect="handleConnect"
          @disconnect="handleDisconnect"
        />

        <BatteryStatus
          :percentage="batteryPercentage"
          :voltage="batteryVoltage"
          :connected="isConnected && vehicleOnline"
        />

        <TelemetryPanel
          v-if="isConnected"
          :odomData="vehicleOnline ? odomData : null"
        />

        <MappingControl 
          :connected="isConnected" 
          :session-id="sessionId"
          :can-control="canControl"
          v-model:isMapping="mappingActive"
          @mappingStarted="onMappingStarted"
          @last-saved="lastSavedMap = $event"
        />

        <NavigationControl
          :connected="isConnected"
          :control-version="getControlGeneration"
          :session-id="sessionId"
          :control-owner="controlOwner"
          :mapping-active="mappingActive"
          :last-saved-map="lastSavedMap"
          :waypoints="waypointQueue"
          :active-waypoint="activeWaypoint"
          v-model:isNavigating="navigationActive"
          :canSetGoal="canSendGoal"
          v-model:isSettingPose="isSettingPose"
          v-model:multiPointMode="multiPointMode"
          :waypointCount="waypointQueue.length + (activeWaypoint ? 1 : 0)"
          @clear-waypoints="clearQueue"
          @execute-waypoints="executeWaypoints"
          @state="onNavigationState"
        />

        <EventLog :connected="isConnected" />
        <CalibrationPanel @opened="stopKeyboardControl()" @active-change="calibrationOpen = $event" />

        <div class="card" v-if="isConnected">
          <div class="card-header">
            图层控制
          </div>
          <div class="card-body">
            <div class="layer-controls">
              <div class="layer-item">
                <span>显示地图</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.map">
                  <span class="slider"></span>
                </label>
              </div>
              <div class="layer-item">
                <span>显示代价地图</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.costmap">
                  <span class="slider"></span>
                </label>
              </div>
              <div class="layer-item">
                <span>显示激光</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.scan">
                  <span class="slider"></span>
                </label>
              </div>
              <div class="layer-item">
                <span>显示机器人</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.robot">
                  <span class="slider"></span>
                </label>
              </div>
              <div class="layer-item">
                <span>显示路径</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.path">
                  <span class="slider"></span>
                </label>
              </div>
              <div class="layer-item">
                <span>显示目标点</span>
                <label class="switch">
                  <input type="checkbox" v-model="mapLayers.goal">
                  <span class="slider"></span>
                </label>
              </div>
            </div>
          </div>
        </div>
      </section>
    </main>
  </div>
</template>

<script setup>
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import { useFoxglove } from './composables/useFoxglove.js'
import { batteryDisplay } from './lib/battery.js'
import JoystickControl from './components/JoystickControl.vue'
import BatteryStatus from './components/BatteryStatus.vue'
import SlamMap from './components/SlamMap.vue'
import ConnectionPanel from './components/ConnectionPanel.vue'
import TelemetryPanel from './components/TelemetryPanel.vue'
import MappingControl from './components/MappingControl.vue'
import NavigationControl from './components/NavigationControl.vue'
import EventLog from './components/EventLog.vue'
import CameraView from './components/CameraView.vue'
import CalibrationPanel from './components/CalibrationPanel.vue'

const foxglove = useFoxglove()
const calibrationOpen = ref(false)
const driveCardCollapsed = ref(true)  // 默认折叠遥控卡，把相机卡让进首屏
const slamMapRef = ref(null)
const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api`
const DEFAULT_WS_URL = `ws://${window.location.hostname}:${import.meta.env.VITE_ROBOT_WS_PORT || 8765}`
// 浏览器侧关键状态上报（写入运行日志目录，供离线分析）；由 VITE_CLIENT_LOG=1 开启。
const CLIENT_LOG_ENABLED = import.meta.env.VITE_CLIENT_LOG === '1'
let clientLogSignature = ''
function reportClient(rows) {
  if (!CLIENT_LOG_ENABLED || !rows?.length) return
  fetch(`${API_BASE}/client_log`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(rows), keepalive: true
  }).catch(() => {})
}

let connectRetryTimer = null
let heartbeatWorker = null
let heartbeatTimer = null
let telemetryTimer = null

const TOPIC_DEFAULTS = {
  cmd_vel: '/cmd_vel',
  battery: '/battery_state',
  map: '/map',
  costmap: '/global_costmap/costmap',
  scan: '/scan',
  tf: '/tf',
  tf_static: '/tf_static',
  odom: '/odom',
  plan: '/plan',
  goal_pose: '/goal_pose',
  initial_pose: '/initialpose',
  navigate_feedback: '/navigate_to_pose/_action/feedback',
  imu: '/imu'
}

// ---- 建图状态 (跨组件共享) ----
const mappingActive = ref(false)

// ---- 导航状态 ----
const navigationActive = ref(false)
const isSettingPose = ref(false)
const multiPointMode = ref(false)
const waypointQueue = ref([])
const activeWaypoint = ref(null)
const simulation = ref(false)
const controlError = ref('')
const navigationState = ref({})
let controlId = null
let controlGeneration = 0
let velocityPending = false
let velocitySequence = 0
let leaseReleasePending = false

// ---- 控制会话与归属（P1-3）----
// 会话 ID 存于 sessionStorage：刷新后仍是同一控制页面；新标签页是新会话。
const sessionId = (() => {
  let id = null
  try { id = sessionStorage.getItem('leap-console-session') } catch {}
  if (!id) {
    id = (typeof crypto !== 'undefined' && crypto.randomUUID)
      ? crypto.randomUUID()
      : `sess-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
    try { sessionStorage.setItem('leap-console-session', id) } catch {}
  }
  return id
})()
function newRequestId() {
  return (typeof crypto !== 'undefined' && crypto.randomUUID)
    ? crypto.randomUUID()
    : `req-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
}
const controlOwner = ref(null)
const isOwner = computed(() => controlOwner.value === sessionId)
const canControl = computed(() => !controlOwner.value || controlOwner.value === sessionId)
const staleManualSession = ref(false)
const lastSavedMap = ref(null)

// ---- 目标预览（P2-4：发送前展示位置与朝向）----
const pendingGoal = ref(null)
const goalSubmitting = ref(false)
const goalBusy = computed(() => ['accepted', 'executing', 'canceling'].includes(navigationState.value.goalState))
const canSendGoal = computed(() => isConnected.value && hasUsableMap.value && vehicleOnline.value && navigationState.value.ready && !goalBusy.value && !goalSubmitting.value && canControl.value)
const previewBlockReason = computed(() => {
  if (!isConnected.value) return '桥接未连接'
  if (!hasUsableMap.value) return '地图尚未加载，请等待地图数据'
  if (!vehicleOnline.value) return '车端已断开或数据中断，请先恢复连接'
  if (!navigationState.value.ready) return '导航未就绪，请查看就绪检查'
  if (goalSubmitting.value) return '目标正在提交，请等待受理'
  if (goalBusy.value) return '已有目标执行中，请先取消'
  if (!canControl.value) return '控制权在其他页面，请先接管'
  return ''
})
const yawDegrees = computed(() => Math.round((pendingGoal.value?.yaw || 0) * 180 / Math.PI))
function setPendingYaw(degrees) {
  if (pendingGoal.value) pendingGoal.value = { ...pendingGoal.value, yaw: Number(degrees) * Math.PI / 180 }
}
function rotatePending(delta) {
  if (!pendingGoal.value) return
  let degrees = Math.round((pendingGoal.value.yaw || 0) * 180 / Math.PI) + delta
  while (degrees > 180) degrees -= 360
  while (degrees < -180) degrees += 360
  setPendingYaw(degrees)
}
function confirmPendingGoal() {
  if (!pendingGoal.value || goalSubmitting.value) return
  const preview = pendingGoal.value
  goalSubmitting.value = true
  submitGoals([{ ...preview }])
    .then(ok => { if (ok && pendingGoal.value === preview) pendingGoal.value = null })
    .finally(() => { goalSubmitting.value = false })
}
function cancelPendingGoal() { pendingGoal.value = null }

// ---- 状态 ----
const topics = ref([])
const batteryPercentage = ref(-1)
const batteryVoltage = ref(0)
const mapData = ref(null)
const costmapData = ref(null)
const scanData = ref(null)
const tfData = ref(null)
const odomData = ref(null)
const planData = ref(null)
// 地图交互（目标/初始位姿）必须以真实地图为前提；SlamMap 在没有地图时会退化为虚拟坐标系。
const hasUsableMap = computed(() => Boolean(mapData.value?.info && Array.isArray(mapData.value?.data) && mapData.value.data.length > 0))

// 导航反馈数据
const navFeedback = ref(null)
const imuData = ref(null)
const runtimeTopics = ref({ ...TOPIC_DEFAULTS })
const cameraStreamUrl = ref('')
const manualEnabled = ref(false)
const backendStatusAt = ref(0)
const lastScanAt = ref(0)
const lastOdomAt = ref(0)
const telemetryClock = ref(Date.now())
// 车端在线只以后端 ROS 检查为准：Foxglove 只是画面通道，卡顿/重连不代表车端离线，
// 更不能据此对整车发急停（历史上这是"跑着跑着突然锁车"的主要误触发源）。
const vehicleOnline = computed(() => navigationState.value.checks?.lidar?.ok === true &&
  navigationState.value.checks?.odometry?.ok === true && backendStatusAt.value > 0 &&
  telemetryClock.value - backendStatusAt.value < 3000)

// 控制链路（HTTP 后端）是否可用；与 Foxglove 画面通道解耦，画面断了仍可安全停车/操作。
const controlLinkOk = computed(() => backendStatusAt.value > 0 &&
  telemetryClock.value - backendStatusAt.value < 3000)

// 后端判定车端数据失效时，本地立即同步解除遥控（后端已做保护停车）。
// 关键：这里绝不再向整车发 /emergency_stop —— 由后端按 lockKind 决定是否锁存硬件急停。
watch(vehicleOnline, (online) => {
  if (online) return
  if (manualEnabled.value) {
    manualEnabled.value = false
    stopKeyboardControl(false)
  }
})
// 四档：慢 / 中 / 快 / 最快。上限 0.50 m/s 与固件硬限幅一致
// (firmware/chassis/main/config/motion_config.h kMaxLinearVelocityMmS=500,
//  kMaxAngularVelocityRadS=1.5)；再快必须改固件并重刷。
const speedProfiles = [
  { id: 'slow', label: '慢 · 0.10 m/s', linear: 0.10, angular: 0.5 },
  { id: 'medium', label: '中 · 0.25 m/s', linear: 0.25, angular: 0.8 },
  { id: 'fast', label: '快 · 0.40 m/s', linear: 0.40, angular: 1.2 },
  { id: 'turbo', label: '最快 · 0.50 m/s', linear: 0.50, angular: 1.5 }
]
const speedId = ref('slow')
// 兜底取最慢档（speedProfiles[0]），避免脏 id 回退到较快档位
const speedProfile = computed(() => speedProfiles.find(profile => profile.id === speedId.value) || speedProfiles[0])
const driveMode = ref('hold')
const driveDirections = [
  { key: 'w', label: '↑ 前进' }, { key: 'a', label: '← 左转' },
  { key: 's', label: '↓ 后退' }, { key: 'd', label: '→ 右转' }
]
watch([speedId, driveMode], () => {
  // 切档只做软停车并要求重新启用，绝不向车端发 /emergency_stop。
  softStop()
  manualEnabled.value = false
  controlId = null
})

// ---- 图层控制 ----
const mapLayers = ref({
  map: true,
  costmap: true,
  scan: true,
  robot: true,
  path: true,
  goal: true
})

// ================= Foxglove 频道 ID ==================

// 订阅 ID
let batterySubId = null
let mapSubId = null
let costmapSubId = null
let scanSubId = null
let tfStaticSubId = null
let tfSubId = null
let odomSubId = null
let planSubId = null
let imuSubId = null

// ---- 计算属性 ----
const connectionState = computed(() => foxglove.connectionState.value)
const errorMessage = computed(() => foxglove.errorMessage.value)
const isConnected = computed(() => connectionState.value === 'connected')
const vehicleConnectionState = computed(() => isConnected.value && !vehicleOnline.value
  ? 'disconnected' : connectionState.value)
watch(connectionState, (state) => {
  if (state === 'connected') return
  // 画面通道断开即视为本页失去操作条件：先软停车，再清本地遥控会话并通知后端释放租约。
  // 心跳不再继续，后端会按租约超时完成最终锁定；恢复连接后必须重新启用遥控。
  softStop()
  releaseLeaseOnDisconnect()
  manualEnabled.value = false
  controlId = null
  stopKeyboardControl(false)
  batterySubId = mapSubId = costmapSubId = scanSubId = tfSubId = tfStaticSubId = null
  odomSubId = planSubId = imuSubId = null
  clearAllData()
})

const statusText = computed(() => {
  switch (connectionState.value) {
    case 'connected': return vehicleOnline.value ? '小车已连接 · 数据正常'
      : (lastScanAt.value || lastOdomAt.value) ? '车端已断开或数据无效、中断 · 桥接在线' : '等待小车数据 · 桥接在线'
    case 'connecting': return '连接中...'
    case 'error': return '连接错误'
    default: return '未连接'
  }
})

// 摇杆不可用时的原因（按优先级给出最关键的阻塞项）
const joystickLockReason = computed(() => {
  if (!controlLinkOk.value) return '控制链路未就绪'
  if (!isConnected.value) return '数据桥接未连接'
  if (!vehicleOnline.value) return '等待车端数据'
  if (!canControl.value) return '控制权在其他页面'
  if (!manualEnabled.value) return '遥控未启用'
  return ''
})

const joystickLockHint = computed(() => {
  if (!controlLinkOk.value) return '检查后端服务是否运行（:3000）'
  if (!isConnected.value) return '检查 foxglove_bridge 与 WS 8765'
  if (!vehicleOnline.value) return '确认小车已上电、Wi-Fi 已连接'
  if (!canControl.value) return '点右上角“取得控制权”或“接管控制权”'
  if (!manualEnabled.value) return '点左侧“启用遥控 / 解除急停”'
  return ''
})

// 全局锁定横幅：急停锁 / 遥控未启用时提示，并可一键解除
const lockedBanner = computed(() => {
  if (!controlLinkOk.value) return null
  const state = navigationState.value || {}
  if (state.mode === 'locked') {
    const last = state.lastStop || {}
    const kind = state.lockKind || 'idle'
    if (kind === 'estop') {
      return { text: `车辆已急停：${last.reason || '人工急停'}`, action: '解除并启用', tone: 'estop' }
    }
    if (kind === 'protective') {
      const reason = last.reason || '保护性停车'
      return {
        text: vehicleOnline.value
          ? `车辆已保护停车：${reason}（数据已恢复，可继续）`
          : `车辆已保护停车：${reason}`,
        action: vehicleOnline.value ? '继续遥控' : '等待数据恢复',
        tone: 'protective'
      }
    }
    // 上电初始状态 / 主动停止：只是在等操作者启用，不是故障也不是急停。
    return vehicleOnline.value
      ? { text: '遥控未启用，摇杆与方向键不可用', action: '启用遥控', tone: 'idle' }
      : { text: '等待车端数据（雷达 / 里程计）', action: '等待数据恢复', tone: 'idle' }
  }
  if (vehicleOnline.value && !manualEnabled.value) {
    return { text: '遥控未启用，摇杆与方向键不可用', action: '启用遥控', tone: 'idle' }
  }
  return null
})

// ---- 事件处理 ----
function handleConnect(url = DEFAULT_WS_URL) {
  if (connectionState.value === 'connected' || connectionState.value === 'connecting') return
  foxglove.connect(url)
}

function onMappingStarted() {
  // 建图开始时自动重置地图视图
  slamMapRef.value?.resetView()
}

// 清空所有订阅数据
function clearAllData() {
  batteryPercentage.value = -1
  batteryVoltage.value = 0
  mapData.value = null
  costmapData.value = null
  scanData.value = null
  lastScanAt.value = 0
  lastOdomAt.value = 0
  tfData.value = null
  odomData.value = null
  planData.value = null
  navFeedback.value = null // 清除导航反馈数据
  imuData.value = null
  waypointQueue.value = []
  activeWaypoint.value = null
  topics.value = []
}

function handleDisconnect() {
  softStop()
  releaseLeaseOnDisconnect()
  manualEnabled.value = false
  controlId = null
  foxglove.disconnect()
  batterySubId = null
  mapSubId = null
  costmapSubId = null
  scanSubId = null
  tfSubId = tfStaticSubId = null
  odomSubId = null
  planSubId = null
  imuSubId = null
  clearAllData()
}

async function discoverCameraUrl() {
  try {
    const res = await fetch(`${API_BASE}/camera/discover`, { method: 'POST' })
    const data = await res.json()
    if (data.success && data.stream_url) {
      cameraStreamUrl.value = data.stream_url
    }
  } catch (err) {
    console.warn('摄像头自动发现失败:', err)
  }
}

async function loadRuntimeConfig() {
  try {
    const res = await fetch(`${API_BASE}/runtime_config`)
    const data = await res.json()
    runtimeTopics.value = { ...TOPIC_DEFAULTS, ...(data.topics || {}) }
    cameraStreamUrl.value = data.camera?.stream_url || ''
    simulation.value = !!data.simulation
  } catch (err) {
    console.warn('加载运行时配置失败，使用默认话题:', err)
    runtimeTopics.value = { ...TOPIC_DEFAULTS }
  }
}

/**
 * 当服务器广播可用频道后，自动:
 * 运动指令由后端统一处理，Foxglove 只接收遥测。
 * 2. Subscribe /battery_state (电量)
 * 3. Subscribe /map (SLAM 地图)
 */
function onChannelsReady(event) {
  const channels = event.detail
  topics.value = channels

  // 2. 固件发布 BatteryState；仅在显式选择旧话题时兼容 UInt16。
  if (!batterySubId) {
    batterySubId = foxglove.subscribe(runtimeTopics.value.battery, (data) => {
      const display = batteryDisplay(data)
      batteryVoltage.value = display.voltage
      batteryPercentage.value = display.percentage
    })
  }

  // 3. 订阅 /map
  if (!mapSubId) {
    mapSubId = foxglove.subscribe(runtimeTopics.value.map, (data) => {
      mapData.value = data
    })
  }

  // 4. 订阅 /scan
  if (!costmapSubId) {
    costmapSubId = foxglove.subscribe(runtimeTopics.value.costmap, (data) => {
      costmapData.value = navigationActive.value && !mappingActive.value ? data : null
    })
  }

  // 5. 订阅 /scan
  if (!scanSubId) {
    scanSubId = foxglove.subscribe(runtimeTopics.value.scan, (data) => {
      scanData.value = data
      lastScanAt.value = Date.now()
    })
  }

  // 6. 订阅 /tf
  if (!tfSubId) {
    tfSubId = foxglove.subscribe(runtimeTopics.value.tf, (data) => {
      tfData.value = data
    })
  }

  // 7. 订阅 /odom
  if (!odomSubId) {
    odomSubId = foxglove.subscribe(runtimeTopics.value.odom, (data) => {
      odomData.value = data
      lastOdomAt.value = Date.now()
    })
  }

  if (!tfStaticSubId) {
    tfStaticSubId = foxglove.subscribe(runtimeTopics.value.tf_static, data => { tfData.value = { ...data, static: true } })
  }

  // 9. 订阅 /plan (导航路径)
  if (!planSubId) {
    planSubId = foxglove.subscribe(runtimeTopics.value.plan, (data) => {
      planData.value = navigationActive.value && !mappingActive.value ? data : null
    })
  }

  // 12. 订阅 IMU（可配置，当前仅缓存数据）
  if (!imuSubId) {
    imuSubId = foxglove.subscribe(runtimeTopics.value.imu, (data) => {
      imuData.value = data
    })
  }
}

// 监听频道就绪事件
window.addEventListener('foxglove:channels', onChannelsReady)
window.addEventListener('foxglove:channels_update', onChannelsReady)
onUnmounted(() => {
  window.removeEventListener('foxglove:channels', onChannelsReady)
  window.removeEventListener('foxglove:channels_update', onChannelsReady)
  handleDisconnect()
})

async function controlRequest(endpoint, data = {}) {
  const response = await fetch(`${API_BASE}/${endpoint}`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(data)
  })
  const result = await response.json()
  if (!response.ok || !result.success) throw new Error(result.error || '控制服务请求失败')
  return result
}
// 无控制页面时，明确的运动操作即取得控制权；他人持有时必须显式接管。
async function ensureOwnership() {
  if (controlOwner.value === sessionId) return
  if (!controlOwner.value) {
    const result = await controlRequest('control/claim', { sessionId, requestId: newRequestId() })
    controlOwner.value = result.owner || sessionId
    return
  }
  throw new Error('控制权由其他页面持有；如需控制请先显式接管（接管会先取消当前运动）')
}
async function takeoverControl() {
  const owner = controlOwner.value
  if (owner && owner !== sessionId) {
    const confirmed = (typeof window !== 'undefined' && typeof window.confirm === 'function')
      ? window.confirm('接管将先取消当前运动并等待停止确认，原控制页面将失去控制权。确定接管？')
      : true
    if (!confirmed) return
  }
  try {
    const result = await controlRequest('control/takeover', { sessionId, requestId: newRequestId() })
    controlOwner.value = result.owner || sessionId
    manualEnabled.value = false
    controlId = null
    staleManualSession.value = false
    controlError.value = ''
  } catch (error) { controlError.value = error.message }
}
function releaseControl() {
  controlRequest('control/release', { sessionId, requestId: newRequestId() })
    .then(() => { controlOwner.value = null; manualEnabled.value = false; controlId = null })
    .catch(error => { controlError.value = error.message })
}
// 桥接断开/主动断开时尽力释放租约；失败也不阻塞本地清理，后端会按心跳超时释放。
function releaseLeaseOnDisconnect() {
  if (leaseReleasePending) return
  if (!manualEnabled.value && controlOwner.value !== sessionId) return
  leaseReleasePending = true
  controlRequest('control/release', { sessionId, requestId: newRequestId() })
    .then(() => { controlOwner.value = null })
    .catch(() => {})
    .finally(() => { leaseReleasePending = false })
}
async function sendVelocity(linearX, angularZ) {
  if (!manualEnabled.value || velocityPending) return
  velocityPending = true
  try { await controlRequest('navigation/velocity', { sessionId, controlId, sequence: ++velocitySequence, linearX, angularZ }) }
  catch (error) { manualEnabled.value = false; controlError.value = error.message }
  finally { velocityPending = false }
}
function onVelocityChange({ linearX, angularZ }) {
  if (!manualEnabled.value || !isConnected.value || !controlLinkOk.value) return
  sendVelocity(Math.max(-speedProfile.value.linear, Math.min(speedProfile.value.linear, linearX)),
    Math.max(-speedProfile.value.angular, Math.min(speedProfile.value.angular, angularZ)))
}
async function setEmergencyStop(state) {
  const generation = ++controlGeneration
  manualEnabled.value = false
  stopKeyboardControl(false)
  controlId = null
  try {
    if (!state && (!controlLinkOk.value || !vehicleOnline.value)) return
    if (!state) await ensureOwnership()
    // 急停属于安全操作：任意页面可用，不校验归属。
    const result = await controlRequest(state ? 'navigation/estop' : 'control/manual_enable',
      state ? { requestId: newRequestId() } : { sessionId, requestId: newRequestId() })
    if (generation !== controlGeneration) {
      if (!state) await controlRequest('navigation/estop', { requestId: newRequestId() })
      return
    }
    // 丢弃启用前及取消阶段发出的状态查询，避免旧 locked 响应撤销本次启用。
    ++controlGeneration
    manualEnabled.value = !state
    controlId = result.controlId
    staleManualSession.value = false
    controlError.value = ''
  } catch (error) { controlError.value = error.message }
}
// 提交中闸门：ACK 返回前禁止再次提交。
// 此前没有闸门，而按钮只靠 500ms 轮询的 goalBusy 禁用 —— ack 前快速双击会发出
// 两次带不同 requestId 的 navigation/goals，后端幂等键不同，等于给车下发两份目标。
let goalsSubmitting = false
async function submitGoals(points) {
  if (goalsSubmitting) return false
  goalsSubmitting = true
  manualEnabled.value = false
  controlId = null
  stopKeyboardControl(false)
  try {
    await ensureOwnership()
    await controlRequest('navigation/goals', { points, sessionId, requestId: newRequestId() })
    controlError.value = ''
    return true
  } catch (error) { controlError.value = error.message; return false }
  finally { goalsSubmitting = false }
}
function onGoalSet({ x, y, yaw = 0 }) {
  // 单点模式：先预览位置与朝向，确认后才发送（P2-4）。
  // 地图未加载时 SlamMap 会退化为虚拟坐标系，必须拒绝，避免提交错误世界坐标。
  if (!hasUsableMap.value) { controlError.value = '地图尚未加载，暂不能设置目标'; return }
  if (!multiPointMode.value) pendingGoal.value = { x, y, yaw }
}
function onWaypointAdd({ x, y, yaw = 0 }) {
  if (!hasUsableMap.value) { controlError.value = '地图尚未加载，暂不能添加航点'; return }
  if (multiPointMode.value && !goalBusy.value) {
    waypointQueue.value.push({ x, y, yaw })
  }
}
function executeWaypoints() {
  if (waypointQueue.value.length) submitGoals([...waypointQueue.value])
}
// 只清"待执行"航点：绝不取消正在执行的目标
// （此前清空航点会顺带发 navigation/cancel，把用户正在跑的目标悄悄取消）。
function clearQueue() {
  waypointQueue.value = []
  activeWaypoint.value = null
}
// 显式取消：只在用户明确要求取消导航时调用。
async function cancelGoal() {
  clearQueue()
  try {
    await controlRequest('navigation/cancel', { sessionId, requestId: newRequestId() })
    controlError.value = ''
  } catch (error) {
    controlError.value = error.message
  }
}
function getControlGeneration() { return controlGeneration }
function onNavigationState(state, version = controlGeneration) {
  if (version !== controlGeneration) return
  // 后端暂时轮询失败：保持上一次已知状态，不伪造锁定、不解除遥控。
  if (state.unreachable) return
  const wasBusy = goalBusy.value
  // 页面归属状态始终反映后端实际状态。
  if ('controlOwner' in state) controlOwner.value = state.controlOwner || null
  navigationState.value = state
  backendStatusAt.value = state.checks ? Date.now() : 0
  navFeedback.value = { ...state.feedback, goalId: state.goalId, goalState: state.goalState }
  if (goalBusy.value) {
    manualEnabled.value = false
    pendingGoal.value = null
    activeWaypoint.value = state.current
    waypointQueue.value = state.queue || []
  } else if (wasBusy) {
    activeWaypoint.value = null
    waypointQueue.value = []
  }
  // 后端仍是 manual 但本页没有速度会话（例如刷新后）：不自动恢复，提示重新启用。
  staleManualSession.value = state.mode === 'manual' && !controlId && isOwner.value
  if (state.mode !== 'manual' || !controlId) manualEnabled.value = false
}
let poseSubmitting = false
async function onInitialPoseSet({ x, y, yaw }) {
  if (poseSubmitting) return
  if (!hasUsableMap.value) { controlError.value = '地图尚未加载，暂不能设置初始位置'; return }
  poseSubmitting = true
  try {
    await ensureOwnership()
    // 必须等到后端 ack 才退出设点模式；失败时保留设点模式，方便原地重试。
    await controlRequest('navigation/initial_pose', { x, y, yaw, sessionId, requestId: newRequestId() })
    isSettingPose.value = false
    controlError.value = ''
  } catch (error) {
    controlError.value = `初始位置未生效：${error.message}（可在地图上重新点击设置）`
  } finally {
    poseSubmitting = false
  }
}

// 关闭多点模式只丢弃"待执行"航点；正在执行的目标保持不动（无隐式取消）。
watch(multiPointMode, (enabled) => {
  if (!enabled) clearQueue()
})

watch([navigationActive, mappingActive], () => {
  // 切换建图/导航只丢弃上一模式的代价图与路径；地图本身保留，避免刷新/切换后
  // 因 mapData 被清空而退化到虚拟坐标系，或等待 transient-local 地图重发。
  costmapData.value = null
  planData.value = null
})

watch(navigationActive, (running) => {
  if (running) {
    slamMapRef.value?.resetView()
  } else {
    // 导航已经停了，这里只需要丢掉待执行航点，不再补发 cancel。
    clearQueue()
  }
})

// ======================== WASD 键盘控制 ========================
const keyboardState = {
  w: false,
  a: false,
  s: false,
  d: false
}
let keyboardInterval = null

function startDirection(event, key) {
  if (!manualEnabled.value || !isConnected.value || !controlLinkOk.value) return
  const wasActive = keyboardState[key]
  stopKeyboardControl()
  if (driveMode.value === 'cruise' && wasActive) return
  event.currentTarget.setPointerCapture(event.pointerId)
  keyboardState[key] = true
  updateKeyboardControl()
  keyboardInterval = setInterval(updateKeyboardControl, 50)
}

function endDirection() {
  if (driveMode.value === 'hold') stopKeyboardControl()
}

function updateKeyboardControl() {
  if (!isConnected.value || !controlLinkOk.value || !manualEnabled.value) return

  let linearX = 0
  let angularZ = 0

  if (keyboardState.w) linearX += speedProfile.value.linear
  if (keyboardState.s) linearX -= speedProfile.value.linear
  if (keyboardState.a) angularZ += speedProfile.value.angular
  if (keyboardState.d) angularZ -= speedProfile.value.angular

  sendVelocity(linearX, angularZ)
}

function stopKeyboardControl(sendStop = true) {
  keyboardState.w = false
  keyboardState.a = false
  keyboardState.s = false
  keyboardState.d = false

  if (keyboardInterval) {
    clearInterval(keyboardInterval)
    keyboardInterval = null
  }

  if (sendStop && manualEnabled.value) {
    controlRequest('navigation/velocity', { sessionId, controlId, sequence: ++velocitySequence, linearX: 0, angularZ: 0 }).catch(error => { controlError.value = error.message })
  }
}

function isTypingTarget(target) {
  if (!target) return false
  const tag = target.tagName
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || target.isContentEditable === true
}

// 弹窗/灯箱优先处理 Escape；只有主工作区才把 Escape 当作急停。
function hasOpenOverlay() {
  if (calibrationOpen.value) return true
  if (typeof document === 'undefined' || typeof document.querySelector !== 'function') return false
  try { return Boolean(document.querySelector('dialog[open], .camera-lightbox, .modal-overlay')) } catch { return false }
}

// 摄像头灯箱本身就是操作界面（内含摇杆与启用/急停按钮），打开时仍须允许 WASD 行驶；
// 标定向导与其他模态继续拦截，避免误动。
function hasBlockingOverlay() {
  if (calibrationOpen.value) return true
  if (typeof document === 'undefined' || typeof document.querySelector !== 'function') return false
  try { return Boolean(document.querySelector('dialog[open], .modal-overlay')) } catch { return false }
}

function handleKeyDown(e) {
  const typing = isTypingTarget(e.target)
  if (e.key === 'Escape') {
    if (typing || hasOpenOverlay()) return
    setEmergencyStop(true); return
  }
  // 标定向导、确认弹窗打开时不穿透；摄像头灯箱例外（它本身是遥控操作界面）。
  if (hasBlockingOverlay()) return
  if (typing) return
  if (e.code === 'Space') { stopKeyboardControl(); return }
  if (!manualEnabled.value) return

  const key = e.key.toLowerCase()
  if (['w', 'a', 's', 'd'].includes(key)) {
    e.preventDefault()
    if (!keyboardState[key]) {
      keyboardState[key] = true
      
      // 按第一下键时，开启 20Hz 的控制循环
      if (!keyboardInterval) {
        updateKeyboardControl() // 立即发一次
        keyboardInterval = setInterval(updateKeyboardControl, 50)
      }
    }
  }
}

function handleKeyUp(e) {
  const key = e.key.toLowerCase()
  if (['w', 'a', 's', 'd'].includes(key)) {
    keyboardState[key] = false
    
    // 如果所有键都松开了，停止循环并发送一次 0 速度
    if (!keyboardState.w && !keyboardState.a && !keyboardState.s && !keyboardState.d) {
      stopKeyboardControl()
    }
  }
}

function handleWindowBlur() {
  // 失焦只做软停车：固件看门狗会在 300 ms 内停住，不再锁存硬件急停。
  softStop()
}

// 状态签名变化时上报一条，避免轮询噪音。
watch([
  () => connectionState.value,
  () => navigationState.value?.mode,
  () => navigationState.value?.lockKind,
  () => navigationState.value?.lastStop?.reason,
  () => manualEnabled.value,
  () => vehicleOnline.value,
  () => isOwner.value,
  () => controlError.value
], () => {
  const signature = JSON.stringify([connectionState.value, navigationState.value?.mode,
    navigationState.value?.lockKind, navigationState.value?.lastStop?.reason,
    manualEnabled.value, vehicleOnline.value, isOwner.value, controlError.value])
  if (signature === clientLogSignature) return
  clientLogSignature = signature
  reportClient([{
    type: 'state', ws: connectionState.value, vehicleOnline: vehicleOnline.value,
    mode: navigationState.value?.mode ?? null, lockKind: navigationState.value?.lockKind ?? null,
    lockReason: navigationState.value?.lastStop?.reason ?? null,
    manualEnabled: manualEnabled.value, owner: isOwner.value, error: controlError.value || null,
    lidarOk: navigationState.value?.checks?.lidar?.ok ?? null,
    odomOk: navigationState.value?.checks?.odometry?.ok ?? null
  }])
}, { flush: 'post' })

/**
 * 软停车：停发速度并由固件 300 ms 看门狗停车，但不锁存车端硬件急停。
 * 用于切档、窗口失焦、主动断开等"人不在操作"的场景。
 */
function softStop() {
  stopKeyboardControl(false)
  if (manualEnabled.value && controlId) {
    controlRequest('navigation/velocity', { sessionId, controlId, sequence: ++velocitySequence,
      linearX: 0, angularZ: 0 }).catch(() => {})
  }
}

/**
 * 运动租约心跳：必须与 Foxglove 画面通道、页面可见性解耦。
 * 用 Web Worker 计时，避开后台标签页对 setInterval 的强节流。
 */
function startHeartbeat() {
  const source = `setInterval(() => postMessage(1), 1000)`
  try {
    heartbeatWorker = new Worker(URL.createObjectURL(new Blob([source], { type: 'text/javascript' })))
    heartbeatWorker.onmessage = sendHeartbeat
  } catch (error) {
    heartbeatTimer = setInterval(sendHeartbeat, 1000)
  }
}

function sendHeartbeat() {
  // 画面桥接断开后不再由本页续租：本地已清遥控会话，租约交给后端按超时释放；
  // 恢复连接后必须重新启用遥控。
  if (!isConnected.value) return
  // 只有持有（或曾取得）控制权的页面才需要续租约。
  if (!manualEnabled.value && controlOwner.value !== sessionId) return
  controlRequest('navigation/heartbeat', { sessionId })
    .then(result => { if ('owner' in result) controlOwner.value = result.owner || null })
    .catch(() => { manualEnabled.value = false })
}

function handleVisibility() { if (document.hidden) softStop(); else sendHeartbeat() }

onMounted(() => {
  telemetryTimer = setInterval(() => { telemetryClock.value = Date.now() }, 500)
  startHeartbeat()
  document.addEventListener('visibilitychange', handleVisibility)
  loadRuntimeConfig()
  handleConnect()
  connectRetryTimer = setInterval(() => {
    if (connectionState.value !== 'connected') {
      handleConnect()
    }
  }, 5000)
  if (CLIENT_LOG_ENABLED) {
    window.addEventListener('error', (event) => reportClient([{ type: 'window-error',
      message: event.message, source: event.filename, line: event.lineno }]))
    window.addEventListener('unhandledrejection', (event) => reportClient([{ type: 'unhandled-rejection',
      message: String(event.reason?.message || event.reason) }]))
    document.addEventListener('visibilitychange', () => reportClient([{ type: 'visibility',
      hidden: document.hidden, manualEnabled: manualEnabled.value }]))
  }
  window.addEventListener('keydown', handleKeyDown)
  window.addEventListener('keyup', handleKeyUp)
  window.addEventListener('blur', handleWindowBlur)
  document.addEventListener('visibilitychange', handleVisibility)
})

onUnmounted(() => {
  if (heartbeatWorker) { heartbeatWorker.terminate(); heartbeatWorker = null }
  if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null }
  if (telemetryTimer) clearInterval(telemetryTimer)
  if (connectRetryTimer) {
    clearInterval(connectRetryTimer)
    connectRetryTimer = null
  }
  window.removeEventListener('keydown', handleKeyDown)
  window.removeEventListener('keyup', handleKeyUp)
  window.removeEventListener('blur', handleWindowBlur)
  document.removeEventListener('visibilitychange', handleVisibility)
  stopKeyboardControl(false)
})

</script>

<style scoped>
.left-panel .card-body > label { display: block; margin-bottom: 10px; }
.left-panel select { width: 100%; padding: 8px; margin-top: 5px; }
.drive-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
.drive-grid button { padding: 14px 8px; touch-action: none; user-select: none; }

.ownership-indicator { display: flex; align-items: center; gap: 8px; }
.owner-badge { font-size: 0.75rem; padding: 4px 10px; border-radius: 999px; font-weight: 600; }
.owner-self { background: rgba(34, 197, 94, 0.15); border: 1px solid rgba(34, 197, 94, 0.4); color: #15803d; }
.owner-other { background: rgba(251, 191, 36, 0.12); border: 1px solid rgba(251, 191, 36, 0.4); color: #b45309; }
.owner-free { background: rgba(100, 116, 139, 0.12); border: 1px solid rgba(100, 116, 139, 0.35); color: #475569; }
.btn-takeover { font-size: 0.75rem; padding: 4px 10px; border-radius: 6px; border: 1px solid rgba(99, 102, 241, 0.45); background: rgba(99, 102, 241, 0.12); color: #4f46e5; cursor: pointer; }
.btn-takeover:hover:not(:disabled) { background: rgba(99, 102, 241, 0.22); }
.btn-takeover:disabled { opacity: 0.5; cursor: not-allowed; }
.observer-hint { color: #b45309; font-size: 0.8rem; }

.center-panel { position: relative; }
.goal-preview { margin-top: 10px; border: 1px solid rgba(34, 197, 94, 0.45); }
.preview-body { display: flex; flex-wrap: wrap; align-items: center; gap: 10px; }
.preview-coords { font-family: var(--font-mono); font-size: 0.82rem; }
.preview-yaw { display: flex; align-items: center; gap: 6px; font-size: 0.8rem; }
.preview-yaw input[type="range"] { width: 160px; }
.preview-actions { display: flex; gap: 6px; }
.preview-actions button { padding: 6px 12px; font-size: 0.8rem; }
.preview-send { background: rgba(34, 197, 94, 0.15); border: 1px solid rgba(34, 197, 94, 0.5); color: #15803d; font-weight: 600; }
.preview-send:disabled { opacity: 0.5; cursor: not-allowed; }
.preview-cancel { border-color: rgba(239, 68, 68, 0.4); color: #dc2626; }
.preview-hint { width: 100%; margin: 0; font-size: 0.75rem; color: #b45309; }
</style>
