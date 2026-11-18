<template>
  <div class="card map-card">
    <div class="card-header">
      <div class="header-left">
        {{ isMapping ? '实时建图' : isNavigating ? '已保存地图 · 定位导航' : '雷达与地图' }}
      </div>
      <div class="header-right">
        <!-- 导航特有控件 -->
        <template v-if="isNavigating">
          <span class="nav-status-mini">{{ navFeedback?.goalState === 'executing' ? '导航中' : '定位导航模式' }}</span>
          <button class="btn-mini btn-mini-pose" :class="{ 'active': isSettingPose }" @click="togglePoseMode">
            设置初始位置
          </button>
        </template>

        <button v-if="isMapping" class="btn-mini btn-mini-save" @click="openSaveDialog" :disabled="isSaving">
          {{ isSaving ? '保存中...' : '保存地图' }}
        </button>
        <button v-if="hasMap" class="btn-mini" @click="resetView">重置视图</button>
      </div>
    </div>
    <div class="card-body">
      <div 
        v-if="shouldShowCanvas" 
        class="map-canvas-container"
        ref="containerRef"
        @wheel.prevent="onWheel"
        @mousedown="onMouseDown"
        @mousemove="onMouseMove"
        @mouseup="onMouseUp"
        @mouseleave="cancelGesture"
        @touchstart.passive="onTouchStart"
        @touchmove.prevent="onTouchMove"
        @touchend="onMouseUp"
        @dblclick="onDblClick"
        :style="{ cursor: isSettingPose ? 'crosshair' : isNavigating ? 'crosshair' : (isDragging ? 'grabbing' : 'grab') }"
      >
        <canvas ref="mapCanvasRef" aria-label="导航地图" />
        <div class="map-info">
          <span v-if="mapInfo">{{ mapInfo.width }} × {{ mapInfo.height }} | 缩放: {{ (scale * 100).toFixed(0) }}%</span>
          <span v-else>无地图模式 | 缩放: {{ (scale * 100).toFixed(0) }}%</span>
        </div>
        
        <!-- 悬浮操作提示 -->
        <div v-if="isNavigating && !isSettingPose" class="map-overlay-hint hint-goal">
          {{ !hasMap ? '地图尚未加载，目标/初始位姿暂不可用' : !canSetGoal ? '等待定位就绪或当前任务结束' : multiPointMode ? '双击添加航点；Shift+拖动指定朝向' : '双击预览目标；Shift+拖动指定朝向' }}
        </div>
        <div v-if="isSettingPose" class="map-overlay-hint hint-pose">
          按住地图拖动以设定初始位置和朝向
        </div>

        <!-- 导航实时反馈面板 -->
        <div v-if="isNavigating && ['accepted', 'executing', 'canceling'].includes(navFeedback?.goalState)" class="map-overlay-feedback">
          <div class="feedback-item">
            <span class="fb-label">剩余距离</span>
            <span class="fb-value">{{ formattedDistance }}</span>
          </div>
          <div class="feedback-item">
            <span class="fb-label">预计时间</span>
            <span class="fb-value">{{ formattedETA }}</span>
          </div>
          <div v-if="!hasNavFeedback" class="fb-waiting">等待导航反馈...</div>
        </div>
      </div>
      <div v-else class="map-placeholder">
        <p v-if="!connected">请先连接机器人</p>
        <p v-else>等待地图或雷达数据...</p>
      </div>

      <!-- 保存状态提示 -->
      <div v-if="saveMsg" class="save-toast" :class="saveSuccess ? 'save-toast-ok' : 'save-toast-err'">
        {{ saveMsg }}
      </div>
      
      <!-- 初始位置设置成功提示 -->
      <div v-if="settingMsg" class="save-toast save-toast-ok">
        {{ settingMsg }}
      </div>
      <div v-if="mapHintMsg" class="save-toast save-toast-err">
        {{ mapHintMsg }}
      </div>

      <!-- 单点导航到达提示 -->
      <div v-if="arriveMsg" class="save-toast save-toast-ok">
        {{ arriveMsg }}
      </div>
    </div>

    <!-- 保存地图对话框 -->
    <Teleport to="body">
      <div v-if="showSaveDialog" class="modal-overlay" role="dialog" aria-modal="true" aria-label="保存地图"
        @keydown.esc.stop="showSaveDialog = false" @click.self="showSaveDialog = false">
        <div class="modal-box">
          <h3 class="modal-title">保存地图</h3>
          <p class="modal-desc">请输入要保存的地图名称：</p>
          <input
            ref="mapNameInputRef"
            v-model="mapNameInput"
            class="modal-input"
            type="text"
            placeholder="例如: my_room_map"
            @keydown.enter="confirmSave"
            @keydown.esc="showSaveDialog = false"
            maxlength="64"
          />
          <div class="modal-actions">
            <button class="btn-modal-cancel" @click="showSaveDialog = false">取消</button>
            <button class="btn-modal-confirm" @click="confirmSave" :disabled="!mapNameInput.trim()">确认保存</button>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script setup>
import { worldToMap, mapToWorld, yawOf, headingToGoal, gridTransform } from '../lib/mapCoordinates.js'
import { cacheTransforms, lookupTransform, transformPoint, frameName } from '../lib/tfTransforms.js'
import { ref, computed, watch, nextTick, onMounted, onUnmounted } from 'vue'

const props = defineProps({
  sessionId: { type: String, default: '' },
  canSetGoal: Boolean,
  connected: Boolean,
  mapData: Object,
  costmapData: Object,
  scanData: Object,
  tfData: Object,
  planData: Object,
  isMapping: Boolean,
  isNavigating: Boolean,
  isSettingPose: Boolean,
  multiPointMode: Boolean,
  waypoints: {
    type: Array,
    default: () => []
  },
  activeWaypoint: {
    type: Object,
    default: null
  },
  pendingGoal: {
    type: Object,
    default: null
  },
  navFeedback: Object,
  layers: {
    type: Object,
    default: () => ({ map: true, costmap: true, scan: true, robot: true, path: true, goal: true })
  }
})

// === 保存地图 ===
const showSaveDialog = ref(false)
const mapNameInput = ref('')
const mapNameInputRef = ref(null)
const isSaving = ref(false)
const saveMsg = ref('')
const saveSuccess = ref(false)
const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api`
function newRequestId() {
  return (typeof crypto !== 'undefined' && crypto.randomUUID)
    ? crypto.randomUUID()
    : `req-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
}

// === 导航 ===
const goalPoint = ref(null)  // { x, y } in world (map) coords
const navPathPoints = ref([])  // Array of { x, y }

// === 初始位置 ===
const poseAnchor = ref(null)  // { x, y } 按下位置 (世界坐标)
const poseDrag = ref(null)    // { x, y } 拖动尾端 (世界坐标)
const settingMsg = ref('')
const mapHintMsg = ref('')
const arriveMsg = ref('')

const emit = defineEmits(['goal-set', 'waypoint-add', 'initial-pose-set', 'update:isSettingPose'])

// === 导航反馈计算 ===
const formattedETA = computed(() => {
  if (!props.navFeedback || !props.navFeedback.estimated_time_remaining) return '-- s'
  const time = props.navFeedback.estimated_time_remaining
  const totalSeconds = time.sec + time.nanosec / 1e9
  if (totalSeconds < 0 || totalSeconds > 3600) return '-- s' // 过滤异常值
  return totalSeconds.toFixed(1) + ' s'
})

const formattedDistance = computed(() => {
  const feedbackDistance = props.navFeedback?.distance_remaining

  let poseDistance = null
  const target = props.multiPointMode ? props.activeWaypoint : goalPoint.value
  if (robotPose.value && target) {
    const dx = robotPose.value.x - target.x
    const dy = robotPose.value.y - target.y
    poseDistance = Math.sqrt(dx * dx + dy * dy)
  }

  const candidates = []
  if (typeof feedbackDistance === 'number' && Number.isFinite(feedbackDistance)) {
    candidates.push(feedbackDistance)
  }
  if (typeof poseDistance === 'number' && Number.isFinite(poseDistance)) {
    candidates.push(poseDistance)
  }
  if (candidates.length === 0) return '-- m'

  // 非常接近目标时显示 0，避免浮点抖动
  const minDistance = Math.min(...candidates)
  if (minDistance <= 0.03) return '0.00 m'
  return minDistance.toFixed(2) + ' m'
})

const hasNavFeedback = computed(() => {
  if (typeof props.navFeedback?.distance_remaining === 'number') return true
  const target = props.multiPointMode ? props.activeWaypoint : goalPoint.value
  return !!(robotPose.value && target)
})

function togglePoseMode() {
  emit('update:isSettingPose', !props.isSettingPose)
}

async function openSaveDialog() {
  mapNameInput.value = ''
  saveMsg.value = ''
  showSaveDialog.value = true
  await nextTick()
  mapNameInputRef.value?.focus()
}

async function confirmSave() {
  const name = mapNameInput.value.trim()
  if (!name) return
  if (!props.sessionId) {
    saveSuccess.value = false
    saveMsg.value = '缺少控制会话标识，请刷新页面后重试'
    return
  }
  showSaveDialog.value = false
  isSaving.value = true
  saveMsg.value = ''
  try {
    const res = await fetch(`${API_BASE}/save_map`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mapName: name, sessionId: props.sessionId, requestId: newRequestId() })
    })
    const data = await res.json()
    saveSuccess.value = data.success
    saveMsg.value = data.success ? `地图已保存: ${name}` : `${data.error}`
    if (data.success) window.dispatchEvent(new CustomEvent('maps-updated', { detail: data }))
  } catch (err) {
    saveSuccess.value = false
    saveMsg.value = '无法连接到本地控制服务'
  } finally {
    isSaving.value = false
    setTimeout(() => { saveMsg.value = '' }, 5000)
  }
}

const containerRef = ref(null)
const mapCanvasRef = ref(null)
const mapInfo = ref(null)

// 离屏 Canvas，用于存储渲染好的地图图像
let offscreenCanvas = null
let costmapCanvas = null
let costmapInfo = null
const showNavigationCostmap = computed(() => props.isNavigating && !props.isMapping)

// 视图变换状态
let viewAdjusted = false
let resizeObserver = null
const scale = ref(1.0)
const offsetX = ref(0)
const offsetY = ref(0)

// 机器人与雷达状态
const robotPose = ref(null) // { x, y, yaw } in map frame
const laserPoints = ref([]) // Array of { x, y } in map frame
let lastScanReceived = 0
const goalAnchor = ref(null)
const goalDrag = ref(null)

// 交互状态
let isDragging = false
let lastMouseX = 0
let lastMouseY = 0

const shouldShowCanvas = computed(() => {
  return props.connected && (props.mapData || props.scanData || props.tfData)
})

const hasMap = computed(() => {
  return props.mapData && props.mapData.info && props.mapData.data
})

/**
 * 获取当前的坐标系信息。如果有地图用地图的，没有则用虚拟的。
 */
function getEffectiveMapInfo() {
  if (props.mapData && props.mapData.info) {
    return props.mapData.info
  }
  // 虚拟坐标系：500x500 像素，0.05米分辨率 (25m x 25m 区域)
  // 原点设在中心 (12.5m, 12.5m)
  return {
    width: 500,
    height: 500,
    resolution: 0.05,
    origin: {
      position: { x: -12.5, y: -12.5, z: 0 }
    }
  }
}

/**
 * 将 OccupancyGrid 原始数据生成图像到离屏 Canvas
 */
function updateOffscreenCanvas(occupancyGrid) {
  const info = occupancyGrid.info
  const data = occupancyGrid.data
  const width = info.width
  const height = info.height

  if (!width || !height || !data || data.length === 0) return

  const geometryChanged = !offscreenCanvas || offscreenCanvas.width !== width || offscreenCanvas.height !== height
  if (!offscreenCanvas) {
    offscreenCanvas = document.createElement('canvas')
  }
  offscreenCanvas.width = width
  offscreenCanvas.height = height

  const ctx = offscreenCanvas.getContext('2d')
  const imageData = ctx.createImageData(width, height)
  const pixels = imageData.data

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const rosIdx = y * width + x
      const val = data[rosIdx]
      const canvasIdx = ((height - 1 - y) * width + x) * 4
      
      let r, g, b
      if (val === -1) { r = 60; g = 60; b = 65 }
      else if (val === 0) { r = 240; g = 240; b = 245 }
      else if (val === 100) { r = 30; g = 30; b = 40 }
      else {
        const intensity = 255 - Math.round((val / 100) * 180)
        r = g = b = intensity
      }
      pixels[canvasIdx] = r
      pixels[canvasIdx + 1] = g
      pixels[canvasIdx + 2] = b
      pixels[canvasIdx + 3] = 255
    }
  }
  ctx.putImageData(imageData, 0, 0)
  
  mapInfo.value = { width, height, resolution: info.resolution }
  
  if (geometryChanged && !viewAdjusted) {
    autoFit()
  } else {
    requestDraw()
  }
}

function updateCostmapCanvas(occupancyGrid) {
  const info = occupancyGrid.info
  const data = occupancyGrid.data
  const width = info.width
  const height = info.height

  if (!width || !height || !data || data.length === 0) return

  if (!costmapCanvas) {
    costmapCanvas = document.createElement('canvas')
  }
  costmapCanvas.width = width
  costmapCanvas.height = height

  const ctx = costmapCanvas.getContext('2d')
  const imageData = ctx.createImageData(width, height)
  const pixels = imageData.data

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const rosIdx = y * width + x
      const val = data[rosIdx]
      const canvasIdx = ((height - 1 - y) * width + x) * 4

      if (val < 1) {
        pixels[canvasIdx + 3] = 0
        continue
      }

      const clamped = Math.max(0, Math.min(100, val))
      const t = clamped / 100
      pixels[canvasIdx] = Math.round(96 - t * 56)
      pixels[canvasIdx + 1] = Math.round(165 - t * 95)
      pixels[canvasIdx + 2] = Math.round(250 - t * 45)
      pixels[canvasIdx + 3] = Math.round(55 + t * 150)
    }
  }

  ctx.putImageData(imageData, 0, 0)
  costmapInfo = info
  requestDraw()
}

/**
 * 转换世界坐标 (米) 到地图图像像素坐标

 */
function worldToPixel(wx, wy) {
  return worldToMap(getEffectiveMapInfo(), wx, wy)
}

let isDrawPending = false
function requestDraw() {
  if (!isDrawPending) {
    isDrawPending = true
    requestAnimationFrame(() => {
      try { draw() } finally { isDrawPending = false }
    })
  }
}

function drawGoalMarker(ctx, waypoint, index, color) {
  const gp = worldToPixel(waypoint.x, waypoint.y)
  const r = 8 / scale.value
  ctx.strokeStyle = color
  ctx.lineWidth = 2 / scale.value
  ctx.beginPath()
  ctx.arc(gp.x, gp.y, r, 0, Math.PI * 2)
  ctx.stroke()

  ctx.fillStyle = color
  ctx.beginPath()
  ctx.arc(gp.x, gp.y, r * 0.45, 0, Math.PI * 2)
  ctx.fill()

  const label = String(index + 1)
  ctx.font = `${12 / scale.value}px var(--font-mono)`
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillStyle = '#ffffff'
  ctx.fillText(label, gp.x, gp.y)
}

/**
 * 绘图逻辑
 */
function draw() {
  const canvas = mapCanvasRef.value
  const container = containerRef.value
  if (!canvas || !container) return

  canvas.width = container.clientWidth
  canvas.height = container.clientHeight
  const ctx = canvas.getContext('2d')
  ctx.clearRect(0, 0, canvas.width, canvas.height)

  const isMapVisible = props.layers.map && offscreenCanvas
  const isCostmapVisible = showNavigationCostmap.value && props.layers.costmap && costmapCanvas &&
    frameName(props.costmapData?.header?.frame_id) === frameName(props.mapData?.header?.frame_id)
  const isScanVisible = props.layers.scan && laserPoints.value.length > 0
  const isRobotVisible = props.layers.robot && robotPose.value

  ctx.save()
  ctx.translate(offsetX.value, offsetY.value)
  ctx.scale(scale.value, scale.value)
  ctx.imageSmoothingEnabled = false 
  
  // 1. 画地图 (如果有)
  if (isMapVisible) {
    ctx.drawImage(offscreenCanvas, 0, 0)
  } else if (!offscreenCanvas) {
    // 画一个淡紫色的十字准星表示虚拟原点
    ctx.strokeStyle = 'rgba(99, 102, 241, 0.2)'
    ctx.lineWidth = 1 / scale.value
    const center = worldToPixel(0, 0)
    ctx.beginPath()
    ctx.moveTo(center.x - 20/scale.value, center.y)
    ctx.lineTo(center.x + 20/scale.value, center.y)
    ctx.moveTo(center.x, center.y - 20/scale.value)
    ctx.lineTo(center.x, center.y + 20/scale.value)
    ctx.stroke()
  }

  if (isCostmapVisible) {
    ctx.save()
    ctx.transform(...gridTransform(getEffectiveMapInfo(), costmapInfo))
    ctx.drawImage(costmapCanvas, 0, 0)
    ctx.restore()
  }

  // 2. 画雷达点 (红色)
  if (isScanVisible) {
    ctx.fillStyle = '#b91c1c'
    for (const p of laserPoints.value) {
      const pix = worldToPixel(p.x, p.y)
      ctx.beginPath()
      ctx.arc(pix.x, pix.y, 2.1 / scale.value, 0, Math.PI * 2)
      ctx.fill()
    }
  }

  // 3. 画机器人 (蓝色箭头)
  if (isRobotVisible) {
    const pix = worldToPixel(robotPose.value.x, robotPose.value.y)
    ctx.save()
    ctx.translate(pix.x, pix.y)
    ctx.rotate(-(robotPose.value.yaw - yawOf(getEffectiveMapInfo().origin.orientation)))
    
    ctx.fillStyle = '#6366f1'
    ctx.beginPath()
    ctx.moveTo(8 / scale.value, 0)
    ctx.lineTo(-6 / scale.value, -6 / scale.value)
    ctx.lineTo(-6 / scale.value, 6 / scale.value)
    ctx.closePath()
    ctx.fill()
    
    ctx.strokeStyle = '#fff'
    ctx.lineWidth = 1 / scale.value
    ctx.stroke()
    ctx.restore()
  }

  // 4. 画导航路径 (绿色折线)
  if (props.isNavigating && !props.isMapping && props.layers.path !== false && navPathPoints.value.length > 1) {
    ctx.strokeStyle = 'rgba(52, 211, 153, 0.8)'
    ctx.lineWidth = 2 / scale.value
    ctx.setLineDash([6 / scale.value, 3 / scale.value])
    ctx.beginPath()
    const first = worldToPixel(navPathPoints.value[0].x, navPathPoints.value[0].y)
    ctx.moveTo(first.x, first.y)
    for (let i = 1; i < navPathPoints.value.length; i++) {
      const pt = worldToPixel(navPathPoints.value[i].x, navPathPoints.value[i].y)
      ctx.lineTo(pt.x, pt.y)
    }
    ctx.stroke()
    ctx.setLineDash([])
  }

  // 5. 画目标点 / 航点
  if (props.layers.goal !== false) {
    // 发送前预览：位置 + 朝向（P2-4），确认后才提交
    if (props.pendingGoal) {
      const pp = worldToPixel(props.pendingGoal.x, props.pendingGoal.y)
      const r = 10 / scale.value
      ctx.save()
      ctx.strokeStyle = '#22c55e'
      ctx.lineWidth = 2 / scale.value
      ctx.setLineDash([4 / scale.value, 3 / scale.value])
      ctx.beginPath()
      ctx.arc(pp.x, pp.y, r, 0, Math.PI * 2)
      ctx.stroke()
      ctx.setLineDash([])
      const yaw = props.pendingGoal.yaw || 0
      ctx.translate(pp.x, pp.y)
      ctx.rotate(-(yaw - yawOf(getEffectiveMapInfo().origin.orientation)))
      ctx.fillStyle = '#22c55e'
      ctx.beginPath()
      ctx.moveTo(r * 2.2, 0)
      ctx.lineTo(r * 0.8, -r * 0.7)
      ctx.lineTo(r * 0.8, r * 0.7)
      ctx.closePath()
      ctx.fill()
      ctx.restore()
      ctx.font = `${11 / scale.value}px var(--font-mono)`
      ctx.textAlign = 'center'
      ctx.fillStyle = '#22c55e'
      ctx.fillText('待发送', pp.x, pp.y - r * 1.6)
    }
    if (props.multiPointMode) {
      if (props.activeWaypoint) {
        drawGoalMarker(ctx, props.activeWaypoint, 0, '#f97316')
      }
      for (let i = 0; i < props.waypoints.length; i++) {
        drawGoalMarker(ctx, props.waypoints[i], i + (props.activeWaypoint ? 1 : 0), '#a78bfa')
      }
    } else if (goalPoint.value) {
      const gp = worldToPixel(goalPoint.value.x, goalPoint.value.y)
      const r = 8 / scale.value
      ctx.strokeStyle = '#a78bfa'
      ctx.lineWidth = 2 / scale.value
      ctx.beginPath()
      ctx.arc(gp.x, gp.y, r, 0, Math.PI * 2)
      ctx.stroke()
      ctx.fillStyle = 'rgba(167, 139, 250, 0.4)'
      ctx.beginPath()
      ctx.arc(gp.x, gp.y, r * 0.5, 0, Math.PI * 2)
      ctx.fill()
      ctx.strokeStyle = '#a78bfa'
      ctx.lineWidth = 1.5 / scale.value
      const cross = 14 / scale.value
      ctx.beginPath()
      ctx.moveTo(gp.x - cross, gp.y)
      ctx.lineTo(gp.x + cross, gp.y)
      ctx.moveTo(gp.x, gp.y - cross)
      ctx.lineTo(gp.x, gp.y + cross)
      ctx.stroke()
    }
  }

  if (goalAnchor.value) {
    const start = worldToPixel(goalAnchor.value.x, goalAnchor.value.y)
    const end = goalDrag.value ? worldToPixel(goalDrag.value.x, goalDrag.value.y) : start
    ctx.strokeStyle = '#22c55e'
    ctx.lineWidth = 3 / scale.value
    ctx.beginPath(); ctx.moveTo(start.x, start.y); ctx.lineTo(end.x, end.y); ctx.stroke()
  }

  // 6. 画初始位置箭头 (黄色)
  if (poseAnchor.value) {
    const ap = worldToPixel(poseAnchor.value.x, poseAnchor.value.y)
    const r = 8 / scale.value
    // 圆圈
    ctx.strokeStyle = '#fbbf24'
    ctx.fillStyle = 'rgba(251, 191, 36, 0.3)'
    ctx.lineWidth = 2 / scale.value
    ctx.beginPath()
    ctx.arc(ap.x, ap.y, r, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
    // 方向箭头
    if (poseDrag.value) {
      const dp = worldToPixel(poseDrag.value.x, poseDrag.value.y)
      const dx = dp.x - ap.x
      const dy = dp.y - ap.y
      const len = Math.sqrt(dx * dx + dy * dy)
      if (len > 2) {
        const ux = dx / len
        const uy = dy / len
        const arrowLen = Math.max(len, 20 / scale.value)
        const ex = ap.x + ux * arrowLen
        const ey = ap.y + uy * arrowLen
        // 主轴线
        ctx.strokeStyle = '#fbbf24'
        ctx.lineWidth = 2.5 / scale.value
        ctx.beginPath()
        ctx.moveTo(ap.x, ap.y)
        ctx.lineTo(ex, ey)
        ctx.stroke()
        // 箭头头部
        const hw = 6 / scale.value
        const hl = 10 / scale.value
        ctx.fillStyle = '#fbbf24'
        ctx.beginPath()
        ctx.moveTo(ex, ey)
        ctx.lineTo(ex - ux * hl - uy * hw, ey - uy * hl + ux * hw)
        ctx.lineTo(ex - ux * hl + uy * hw, ey - uy * hl - ux * hw)
        ctx.closePath()
        ctx.fill()
      }
    }
  }

  ctx.restore()
}

// 监听图层配置变化
watch(() => props.layers, () => {
  requestDraw()
}, { deep: true })

// 监听导航路径变化
watch(() => props.planData, (newData) => {
  if (!newData || !newData.poses) {
    navPathPoints.value = []
    requestDraw()
    return
  }
  navPathPoints.value = newData.poses.map(p => ({ x: p.position.x, y: p.position.y }))
  requestDraw()
}, { deep: true })

// ==================== TF 树管理 ====================
// ROS 2 的 TF 树通常是: map -> odom -> base_link -> base_scan
// /tf 话题分别发布各段变换，我们需要缓存并组合它们

const tfTree = {}  // { 'parent|child': { translation, rotation } }

watch(() => props.connected, (connected) => {
  if (connected) return
  for (const key of Object.keys(tfTree)) delete tfTree[key]
  robotPose.value = null
  laserPoints.value = []
  offscreenCanvas = costmapCanvas = null
  mapInfo.value = null
  goalPoint.value = null
  navPathPoints.value = []
  viewAdjusted = false
  scale.value = 1
  offsetX.value = offsetY.value = 0
})

// Never draw odometry coordinates on a map when map→odom is missing.
function updateScene() {
  const frame = props.mapData ? frameName(props.mapData.header?.frame_id || 'map') : 'odom'
  const base = lookupTransform(tfTree, frame, 'base_link') || lookupTransform(tfTree, frame, 'base_footprint')
  robotPose.value = base ? { ...base.translation, yaw: yawOf(base.rotation) } : null
  const scan = props.scanData
  const laser = lookupTransform(tfTree, frame, scan?.header?.frame_id)
  const points = []
  if (scan?.ranges && laser && Date.now() - lastScanReceived < 1500) {
    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i]
      if (!Number.isFinite(r) || r < scan.range_min || r > scan.range_max) continue
      const angle = scan.angle_min + i * scan.angle_increment
      points.push(transformPoint(laser, { x: r * Math.cos(angle), y: r * Math.sin(angle), z: 0 }))
    }
  }
  laserPoints.value = points
  requestDraw()
}
watch(() => props.tfData, data => { cacheTransforms(tfTree, data); updateScene() }, { deep: true })
watch(() => props.scanData, () => { lastScanReceived = Date.now(); updateScene() }, { deep: true })
watch(() => props.mapData, updateScene)

function autoFit() {
  if (!offscreenCanvas || !containerRef.value) return
  const container = containerRef.value
  const cw = container.clientWidth
  const ch = container.clientHeight
  const mw = offscreenCanvas.width
  const mh = offscreenCanvas.height

  if (cw === 0 || ch === 0) return

  // 计算缩放：使图像完整放入容器，且充满其中一个维度
  const s = Math.min(cw / mw, ch / mh)
  scale.value = s
  offsetX.value = (cw - mw * s) / 2
  offsetY.value = (ch - mh * s) / 2
  requestDraw()
}

function resetView() {
  viewAdjusted = false
  autoFit()
}

// ---- 事件处理 ----
function onWheel(e) {
  viewAdjusted = true
  const zoomSpeed = 0.1
  const oldScale = scale.value
  const delta = -Math.sign(e.deltaY)
  const newScale = Math.max(0.1, Math.min(20, oldScale * (1 + delta * zoomSpeed)))
  
  // 获取鼠标在 Canvas 上的相对坐标
  const rect = containerRef.value.getBoundingClientRect()
  const mouseX = e.clientX - rect.left
  const mouseY = e.clientY - rect.top

  // 以鼠标位置为中心缩放
  offsetX.value = mouseX - (mouseX - offsetX.value) * (newScale / oldScale)
  offsetY.value = mouseY - (mouseY - offsetY.value) * (newScale / oldScale)
  
  scale.value = newScale
  requestDraw()
}

function onMouseDown(e) {
  if (e.shiftKey && props.isNavigating && props.canSetGoal && !props.isSettingPose && hasMap.value) {
    const rect = containerRef.value.getBoundingClientRect()
    goalAnchor.value = mapToWorld(getEffectiveMapInfo(),
      (e.clientX - rect.left - offsetX.value) / scale.value,
      (e.clientY - rect.top - offsetY.value) / scale.value)
    goalDrag.value = null
    return
  }
  if (props.isSettingPose) {
    if (!hasMap.value) { showMapHint('地图尚未加载，暂不能设置初始位姿'); return }
    // 初始位置模式: 记录按下点，禁止地图拖动
    const rect = containerRef.value.getBoundingClientRect()
    const cx = (e.clientX - rect.left - offsetX.value) / scale.value
    const cy = (e.clientY - rect.top - offsetY.value) / scale.value
    const info = getEffectiveMapInfo()
    poseAnchor.value = mapToWorld(info, cx, cy)
    poseDrag.value = null
    requestDraw()
    return
  }
  isDragging = true
  lastMouseX = e.clientX
  lastMouseY = e.clientY
}

function onMouseMove(e) {
  if (goalAnchor.value) {
    const rect = containerRef.value.getBoundingClientRect()
    goalDrag.value = mapToWorld(getEffectiveMapInfo(),
      (e.clientX - rect.left - offsetX.value) / scale.value,
      (e.clientY - rect.top - offsetY.value) / scale.value)
    requestDraw()
    return
  }
  if (props.isSettingPose && poseAnchor.value) {
    const rect = containerRef.value.getBoundingClientRect()
    const cx = (e.clientX - rect.left - offsetX.value) / scale.value
    const cy = (e.clientY - rect.top - offsetY.value) / scale.value
    const info = getEffectiveMapInfo()
    poseDrag.value = mapToWorld(info, cx, cy)
    draw()
    return
  }
  if (!isDragging) return
  viewAdjusted = true
  const dx = e.clientX - lastMouseX
  const dy = e.clientY - lastMouseY
  offsetX.value += dx
  offsetY.value += dy
  lastMouseX = e.clientX
  lastMouseY = e.clientY
  draw()
}

function showMapHint(text) {
  mapHintMsg.value = text
  setTimeout(() => { if (mapHintMsg.value === text) mapHintMsg.value = '' }, 3000)
}
function submitMapGoal(point, yaw) {
  if (!hasMap.value) { showMapHint('地图尚未加载，暂不能设置目标'); return }
  if (props.multiPointMode) emit('waypoint-add', { ...point, yaw })
  else {
    goalPoint.value = { ...point, yaw }
    emit('goal-set', { ...point, yaw })
  }
  requestDraw()
}
function cancelGesture() {
  poseAnchor.value = poseDrag.value = goalAnchor.value = goalDrag.value = null
  isDragging = false
  requestDraw()
}
function onMouseUp(e) {
  if (goalAnchor.value) {
    const point = goalAnchor.value
    const yaw = goalDrag.value ? headingToGoal({ ...point, yaw: robotPose.value?.yaw || 0 }, goalDrag.value)
      : headingToGoal(robotPose.value, point)
    if (props.isNavigating && props.canSetGoal) submitMapGoal(point, yaw)
    cancelGesture()
    return
  }
  if (props.isSettingPose && poseAnchor.value) {
    if (!hasMap.value) {
      showMapHint('地图尚未加载，暂不能设置初始位姿')
      poseAnchor.value = poseDrag.value = null
      draw()
      return
    }
    let yaw = robotPose.value?.yaw || 0
    if (poseDrag.value) {
      const dx = poseDrag.value.x - poseAnchor.value.x
      const dy = poseDrag.value.y - poseAnchor.value.y
      if (Math.abs(dx) > 0.01 || Math.abs(dy) > 0.01) {
        yaw = Math.atan2(dy, dx)
      }
    }
    // 只提交，不自行退出设点模式：是否退出由 App 依据后端 ack 决定。
    // 此前这里紧跟 emit('update:isSettingPose', false)，配合 App 的 v-model 会直接改父级
    // 状态，使 App 里"等 ack 再关"形同虚设 —— 请求失败时用户已退出设点模式、无法原地重试。
    emit('initial-pose-set', { x: poseAnchor.value.x, y: poseAnchor.value.y, yaw })

    // 显示等待提示（保持设点模式，直到 App 收到 ack）
    settingMsg.value = '已提交初始位置，等待 AMCL 确认'
    setTimeout(() => { settingMsg.value = '' }, 3000)

    // 清除预览
    poseAnchor.value = null
    poseDrag.value = null
    draw()
    return
  }
  isDragging = false
}

// 双击设定导航目标点
function onDblClick(e) {
  if (!props.isNavigating || !props.canSetGoal) return
  if (props.isSettingPose) return
  if (!hasMap.value) { showMapHint('地图尚未加载，暂不能设置目标'); return }
  const rect = containerRef.value.getBoundingClientRect()
  const canvasX = (e.clientX - rect.left - offsetX.value) / scale.value
  const canvasY = (e.clientY - rect.top - offsetY.value) / scale.value
  // 将 Canvas 像素坐标转换回世界坐标
  const info = getEffectiveMapInfo()
  const { x: wx, y: wy } = mapToWorld(info, canvasX, canvasY)
  submitMapGoal({ x: wx, y: wy }, headingToGoal(robotPose.value, { x: wx, y: wy }))
}

// 触摸支持
let initialPinchDist = 0
let lastPinchCenter = { x: 0, y: 0 }

function getPinchDist(e) {
  const dx = e.touches[0].clientX - e.touches[1].clientX
  const dy = e.touches[0].clientY - e.touches[1].clientY
  return Math.sqrt(dx * dx + dy * dy)
}

function getPinchCenter(e) {
  return {
    x: (e.touches[0].clientX + e.touches[1].clientX) / 2,
    y: (e.touches[0].clientY + e.touches[1].clientY) / 2
  }
}

function onTouchStart(e) {
  if (e.touches.length === 1) {
    onMouseDown(e.touches[0])
  } else if (e.touches.length === 2) {
    isDragging = false // 缩放时停止拖动
    initialPinchDist = getPinchDist(e)
    lastPinchCenter = getPinchCenter(e)
  }
}

function onTouchMove(e) {
  if (e.touches.length === 1) {
    onMouseMove(e.touches[0])
  } else if (e.touches.length === 2) {
    const dist = getPinchDist(e)
    const center = getPinchCenter(e)
    const rect = containerRef.value.getBoundingClientRect()
    
    viewAdjusted = true
    const zoomFactor = dist / initialPinchDist
    const oldScale = scale.value
    const newScale = Math.max(0.1, Math.min(20, oldScale * zoomFactor))
    
    // 以双指中心缩放
    const cx = center.x - rect.left
    const cy = center.y - rect.top
    
    offsetX.value = cx - (cx - offsetX.value) * (newScale / oldScale)
    offsetY.value = cy - (cy - offsetY.value) * (newScale / oldScale)
    
    scale.value = newScale
    initialPinchDist = dist
    draw()
  }
}

watch(containerRef, (container) => {
  resizeObserver?.disconnect()
  if (!container) return
  resizeObserver = new ResizeObserver(() => { if (!viewAdjusted) autoFit(); else requestDraw() })
  resizeObserver.observe(container)
})
onUnmounted(() => resizeObserver?.disconnect())

watch(() => props.mapData, (newData) => {
  if (newData && newData.info && newData.data) {
    updateOffscreenCanvas(newData)
  } else {
    offscreenCanvas = null
    mapInfo.value = null
    requestDraw()
  }
}, { deep: true })

// Navigation overlays must never survive into a different mapping/navigation session.
watch(() => [props.isNavigating, props.isMapping], () => {
  for (const key of Object.keys(tfTree)) if (key.startsWith('map|')) delete tfTree[key]
  navPathPoints.value = []
  cancelGesture()
  updateScene()
  costmapCanvas = null
  requestDraw()
})

watch(() => props.costmapData, (newData) => {
  if (showNavigationCostmap.value && newData && newData.info && newData.data) {
    updateCostmapCanvas(newData)
  } else {
    costmapCanvas = null
    requestDraw()
  }
}, { deep: true })

watch(() => props.multiPointMode, (enabled) => {
  if (enabled) {
    goalPoint.value = null
  }
  requestDraw()
})

watch(() => props.pendingGoal, (pending) => {
  // 预览被取消且没有执行中的目标时，清除双击留下的内部标记，避免误读为已发送。
  if (!pending && !props.multiPointMode &&
      !['accepted', 'executing', 'canceling'].includes(props.navFeedback?.goalState)) {
    goalPoint.value = null
  }
  requestDraw()
})

watch(() => [props.waypoints, props.activeWaypoint, props.multiPointMode], () => {
  if (!props.multiPointMode) goalPoint.value = props.activeWaypoint
  requestDraw()
}, { deep: true })



watch(() => props.navFeedback?.goalState, (state) => {
  if (state === 'succeeded') {
    goalPoint.value = null
    requestDraw()
    arriveMsg.value = 'Nav2 已确认到达'
    setTimeout(() => { arriveMsg.value = '' }, 2500)
  }
})

function onWindowKeydown(event) {
  if (event.key !== 'Escape' || !showSaveDialog.value) return
  event.preventDefault()
  event.stopPropagation()
  showSaveDialog.value = false
}

onMounted(() => {
  if (typeof window !== 'undefined' && typeof window.addEventListener === 'function') {
    window.addEventListener('keydown', onWindowKeydown, true)
  }
  if (props.mapData) updateOffscreenCanvas(props.mapData)
  if (showNavigationCostmap.value && props.costmapData) updateCostmapCanvas(props.costmapData)
})

onUnmounted(() => {
  if (typeof window !== 'undefined' && typeof window.removeEventListener === 'function') {
    window.removeEventListener('keydown', onWindowKeydown, true)
  }
})

defineExpose({
  resetView,
  clearGoal: () => { goalPoint.value = null; navPathPoints.value = []; draw() },
  clearPose: () => { poseAnchor.value = null; poseDrag.value = null; draw() }
})

</script>

<style scoped>
.header-left {
  display: flex;
  align-items: center;
}
.header-right {
  display: flex;
  align-items: center;
  gap: 6px;
}
.btn-mini {
  background: #f8fafc;
  border: 1px solid #cfd6df;
  color: #334155;
  padding: 2px 8px;
  border-radius: 4px;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
}
.btn-mini:hover:not(:disabled) {
  background: #e2e8f0;
}
.btn-mini:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
.btn-mini-save {
  background: rgba(52, 211, 153, 0.15);
  border-color: rgba(52, 211, 153, 0.4);
  color: #15803d;
}
.btn-mini-save:hover:not(:disabled) {
  background: rgba(52, 211, 153, 0.3);
}
.btn-mini-pose {
  background: rgba(251, 191, 36, 0.15);
  border-color: rgba(251, 191, 36, 0.4);
  color: #b45309;
}
.btn-mini-pose:hover {
  background: rgba(251, 191, 36, 0.3);
}
.btn-mini-pose.active {
  background: rgba(251, 191, 36, 0.3);
  border-color: #b45309;
  animation: pose-pulse 1.2s ease-in-out infinite;
}
@keyframes pose-pulse {
  0%, 100% { box-shadow: 0 0 0 0 rgba(251, 191, 36, 0.4); }
  50% { box-shadow: 0 0 0 4px rgba(251, 191, 36, 0); }
}
.nav-status-mini {
  color: #15803d;
  font-size: 11px;
  font-weight: bold;
  margin-right: 4px;
}
.map-card :deep(.card-body) {
  padding: 0;
  display: flex;
  flex-direction: column;
  position: relative;
}
.map-canvas-container {
  width: 100%;
  flex: 1;
  min-height: 0;
  position: relative;
  overflow: hidden;
  background: #f3f4f6;
  cursor: grab;
}
.map-canvas-container:active {
  cursor: grabbing;
}
canvas {
  display: block;
}

.map-info {
  position: absolute;
  top: 12px;
  left: 12px;
  background: rgba(255, 255, 255, 0.92);
  padding: 4px 10px;
  border-radius: var(--radius-sm);
  font-family: var(--font-mono);
  font-size: 0.7rem;
  color: var(--text-secondary);
  pointer-events: none;
}
.map-overlay-hint {
  position: absolute;
  top: 12px;
  left: 50%;
  transform: translateX(-50%);
  padding: 6px 14px;
  border-radius: 20px;
  font-size: 0.8rem;
  font-weight: 500;
  pointer-events: none;
  animation: fadein-down 0.3s ease;
  backdrop-filter: blur(4px);
}
.hint-goal {
  color: #2563eb;
  background: rgba(99, 102, 241, 0.25);
  border: 1px solid rgba(99, 102, 241, 0.4);
}
.hint-pose {
  color: #b45309;
  background: rgba(251, 191, 36, 0.25);
  border: 1px solid rgba(251, 191, 36, 0.4);
}

.map-overlay-feedback {
  position: absolute;
  top: 56px;
  right: 12px;
  display: flex;
  flex-direction: column;
  gap: 6px;
  min-width: 150px;
  padding: 10px 12px;
  border-radius: 10px;
  border: 1px solid rgba(99, 102, 241, 0.35);
  background: rgba(255, 255, 255, 0.94);
  backdrop-filter: blur(4px);
  z-index: 5;
}

.feedback-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
}

.fb-label {
  font-size: 0.72rem;
  color: var(--text-secondary);
}

.fb-value {
  font-size: 0.78rem;
  font-family: var(--font-mono);
  color: var(--text-primary);
}

.fb-waiting {
  font-size: 0.72rem;
  color: var(--text-muted);
  text-align: right;
}
@keyframes fadein-down {
  from { opacity: 0; transform: translate(-50%, -10px); }
  to   { opacity: 1; transform: translate(-50%, 0); }
}

/* 保存提示 */
.save-toast {
  position: absolute;
  bottom: 12px;
  left: 50%;
  transform: translateX(-50%);
  padding: 7px 18px;
  border-radius: 20px;
  font-size: 13px;
  font-weight: 500;
  pointer-events: none;
  animation: fadein 0.2s ease;
  white-space: nowrap;
  z-index: 10;
}
.save-toast-ok  { background: rgba(16,185,129,0.85); color: #fff; }
.save-toast-err { background: rgba(239,68,68,0.85);  color: #fff; }
@keyframes fadein { from { opacity:0; transform:translateX(-50%) translateY(6px); } to { opacity:1; transform:translateX(-50%) translateY(0); } }

/* Modal */
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(15, 23, 42, 0.24);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  backdrop-filter: blur(4px);
}
.modal-box {
  background: #ffffff;
  border: 1px solid #cfd6df;
  border-radius: 14px;
  padding: 28px 32px;
  min-width: 340px;
  box-shadow: 0 18px 48px rgba(15, 23, 42, 0.18);
  animation: modal-in 0.18s ease;
}
@keyframes modal-in {
  from { transform: scale(0.92); opacity: 0; }
  to   { transform: scale(1);    opacity: 1; }
}
.modal-title { font-size: 1.1rem; font-weight: 700; color: var(--text-primary); margin: 0 0 8px; }
.modal-desc  { font-size: 0.85rem; color: var(--text-muted); margin: 0 0 14px; }
.modal-input {
  width: 100%; box-sizing: border-box;
  background: #f8fafc;
  border: 1px solid #cfd6df;
  border-radius: 8px; padding: 10px 12px;
  color: var(--text-primary); font-size: 0.95rem; outline: none;
  transition: border-color 0.2s;
}
.modal-input:focus { border-color: #6366f1; }
.modal-actions { display: flex; gap: 10px; margin-top: 16px; justify-content: flex-end; }
.btn-modal-cancel {
  background: #f8fafc;
  border: 1px solid #cfd6df;
  color: var(--text-muted); padding: 8px 18px;
  border-radius: 8px; cursor: pointer; font-size: 0.9rem;
  transition: all 0.2s;
}
.btn-modal-cancel:hover { background: #e2e8f0; }
.btn-modal-confirm {
  background: #2563eb;
  border: none; color: #fff; padding: 8px 20px;
  border-radius: 8px; cursor: pointer;
  font-size: 0.9rem; font-weight: 600;
  transition: all 0.2s;
}
.btn-modal-confirm:hover:not(:disabled) { background: #1d4ed8; }
.btn-modal-confirm:disabled { opacity: 0.4; cursor: not-allowed; }
</style>
