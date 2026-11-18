<template>
  <div class="card" v-if="connected">
    <div class="card-header event-header">
      <span>任务事件</span>
      <button class="btn-export" @click="exportDiagnostics" title="下载诊断包（版本、运行参数、节点状态、事件与有限日志；不含密钥）">
        导出诊断包
      </button>
    </div>
    <div class="card-body">
      <div v-if="!events.length" class="empty">暂无事件；启动、定位、目标、取消与失败都会记录在这里</div>
      <ul class="event-list">
        <li v-for="event in events" :key="event.seq" :class="levelClass(event)">
          <span class="event-time">{{ formatTime(event.at) }}</span>
          <span class="event-type">{{ typeLabel(event.type) }}</span>
          <span v-if="event.reason" class="event-reason">{{ event.reason }}</span>
          <span class="event-ids">
            <template v-if="event.runId">运行#{{ event.runId }}</template>
            <template v-if="event.goalId"> · 目标{{ shortId(event.goalId) }}</template>
          </span>
        </li>
      </ul>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'

const props = defineProps({ connected: Boolean })

const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api`
const events = ref([])
let cursor = 0
let timer = null
let polling = false

const TYPE_LABELS = {
  startup_begin: '导航启动', startup_done: '启动完成', startup_failed: '启动失败',
  localization_requested: '请求定位', localization_confirmed: '定位已确认',
  goal_submitted: '提交目标', goal_dispatched: '派发目标', goal_accepted: '目标受理',
  goal_succeeded: '目标完成', goal_failed: '目标失败', goal_canceled: '目标取消',
  queue_completed: '队列完成', cancel_requested: '请求取消', cancel_confirmed: '取消已确认',
  cancel_timeout: '取消超时', estop: '急停', manual_enabled: '启用遥控',
  lease_expired: '心跳失联', sensor_dropout: '传感器断流', sensor_recovered: '传感器恢复',
  node_inactive: '节点失活', node_active: '节点激活', navigation_fault: '导航失效锁定',
  navigation_stop: '停止导航', control_claimed: '取得控制权', control_takeover: '接管控制权',
  control_released: '释放控制权', request_denied: '越权操作被拒', heartbeat_denied: '非控制页心跳被拒',
  bridge_shutdown: '控制服务退出'
}
const ERROR_TYPES = new Set(['startup_failed', 'goal_failed', 'cancel_timeout', 'estop', 'lease_expired',
  'sensor_dropout', 'node_inactive', 'navigation_fault', 'bridge_shutdown', 'request_denied', 'heartbeat_denied'])

function typeLabel(type) { return TYPE_LABELS[type] || type }
function shortId(id) { return String(id).slice(0, 6) }
function levelClass(event) { return ERROR_TYPES.has(event.type) ? 'event-error' : '' }
function formatTime(at) {
  const date = new Date(at * 1000)
  return date.toLocaleTimeString('zh-CN', { hour12: false })
}

async function pollEvents() {
  if (polling) return
  polling = true
  try {
    const res = await fetch(`${API_BASE}/navigation/events?since=${cursor}`)
    const data = await res.json()
    if (data.success && Array.isArray(data.events) && data.events.length) {
      events.value = [...events.value, ...data.events].slice(-60)
      cursor = data.events.at(-1).seq
    }
  } catch { /* 后端不可用时保持现有事件，不清空 */ }
  finally { polling = false }
}

function exportDiagnostics() {
  // Content-Disposition: attachment → 浏览器直接下载 JSON 诊断包。
  const link = document.createElement('a')
  link.href = `${API_BASE}/diagnostics/bundle`
  link.rel = 'noopener'
  document.body.appendChild(link)
  link.click()
  link.remove()
}

watch(() => props.connected, (connected) => {
  if (connected) pollEvents()
})

onMounted(() => {
  if (props.connected) pollEvents()
  timer = setInterval(pollEvents, 2000)
})
onUnmounted(() => { if (timer) clearInterval(timer) })
</script>

<style scoped>
.event-header { display: flex; justify-content: space-between; align-items: center; }
.btn-export { font-size: 0.72rem; padding: 3px 8px; border-radius: 6px; border: 1px solid rgba(99, 102, 241, 0.4); background: rgba(99, 102, 241, 0.1); color: #4f46e5; cursor: pointer; }
.btn-export:hover { background: rgba(99, 102, 241, 0.2); }
.empty { font-size: 0.75rem; color: var(--text-muted); }
.event-list { list-style: none; margin: 0; padding: 0; max-height: 220px; overflow-y: auto; font-size: 0.74rem; }
.event-list li { padding: 3px 2px; border-bottom: 1px dashed var(--border-color); display: flex; flex-wrap: wrap; gap: 4px; }
.event-time { color: var(--text-muted); font-family: var(--font-mono); }
.event-type { font-weight: 600; color: var(--text-secondary); }
.event-reason { color: var(--text-primary); flex-basis: 100%; }
.event-ids { color: var(--text-muted); flex-basis: 100%; font-family: var(--font-mono); }
.event-error .event-type { color: #dc2626; }
.event-error .event-reason { color: #dc2626; }
</style>
