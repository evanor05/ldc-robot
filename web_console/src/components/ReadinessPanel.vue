<template>
  <div class="readiness-panel">
    <!-- 等待用户定位与系统启动失败分开显示 -->
    <div v-if="phase === 'waiting_localization'" class="banner banner-wait">
      ⏳ 等待用户定位：请在地图上设置初始位置和朝向（不限时）
    </div>
    <div v-else-if="phase === 'failed'" class="banner banner-fail">
      ⛔ 导航不可用：{{ status.error || status.missing?.join('；') || '请查看下方检查项' }}
    </div>
    <table class="check-table">
      <tbody>
        <tr v-for="item in rows" :key="item.key" :class="{ 'check-bad': !item.ok }">
          <td class="dot" :class="item.ok ? 'dot-ok' : 'dot-bad'">{{ item.ok ? '●' : '○' }}</td>
          <td class="name">{{ item.label }}</td>
          <td class="detail" :title="item.detail">{{ item.detail }}</td>
          <td class="age">{{ item.age }}</td>
        </tr>
      </tbody>
    </table>
    <p v-if="status.nextAction" class="next-action">下一步：{{ status.nextAction }}</p>
    <p v-if="status.lastStop" class="stop-reason">最近停车：{{ status.lastStop.reason }}</p>
    <details v-if="status.runtimeProfile">
      <summary>{{ status.isRunning ? '实际算法与定位质量' : '上次运行配置与定位记录' }}</summary>
      <p>地图：{{ status.mapPath || '未加载' }}</p>
      <p>规划：{{ plugins('planner_server', 'planner_plugins') }}</p>
      <p>控制：{{ plugins('controller_server', 'controller_plugins') }}</p>
      <p>关键运行参数指纹：{{ status.runtimeProfile.sha256?.slice(0, 12) || '等待读取' }}{{ status.runtimeProfile.fresh ? '' : '（未更新）' }}</p>
      <p v-if="!status.runtimeProfile.complete">运行参数尚未完整读取：{{ status.runtimeProfile.observedNodes?.length || 0 }} / {{ status.runtimeProfile.expectedNodes?.length || 6 }} 个节点。</p>
      <p v-for="file in status.configuration?.parameterFiles || []" :key="file.path">启动参数文件：{{ file.sha256?.slice(0, 12) || '未读取' }}</p>
      <p v-if="status.checks?.localization">定位：{{ status.checks.localization.code }}；σ位置 {{ uncertainty(status.checks.localization.sigmaXY) }}m，σ朝向 {{ uncertainty(status.checks.localization.sigmaYaw) }}rad</p>
      <p>车端固件版本：尚未实车核对</p>
    </details>
  </div>
</template>

<script setup>
import { computed } from 'vue'

const props = defineProps({
  status: { type: Object, default: () => ({}) }
})

const phase = computed(() => props.status.phase)
function uncertainty(value) { return Number.isFinite(value) ? value.toFixed(3) : '--' }
function plugins(node, field) {
  const params = props.status.runtimeProfile?.parameters?.[node] || {}
  const aliases = Array.isArray(params[field]) ? params[field] : []
  return aliases.map(alias => params[`${alias}.plugin`] || `${alias}（读取中）`).join('、') || '等待 ROS 参数服务'
}

function ageText(seconds) {
  if (seconds == null || !Number.isFinite(seconds)) return '--'
  if (seconds < 60) return `${seconds.toFixed(1)} 秒前`
  return `${Math.floor(seconds / 60)} 分钟前`
}

const rows = computed(() => {
  const checks = props.status.checks || {}
  const defs = [
    ['lidar', '雷达 /scan'],
    ['odometry', '里程计 /odom'],
    ['tf', 'TF 变换'],
    ['localization', '定位 (AMCL)'],
    ['nav2', 'Nav2 节点'],
    ['control', '控制连接']
  ]
  return defs.map(([key, label]) => {
    const item = checks[key] || {}
    let age = '--'
    if (key === 'control') age = item.leaseAgeSeconds != null ? `租约 ${ageText(item.leaseAgeSeconds)}` : (item.owner ? '在线' : '无控制页')
    else age = ageText(item.ageSeconds)
    return { key, label, ok: !!item.ok, detail: item.detail || (props.status.isRunning ? '未知' : '导航未启动'), age }
  })
})
</script>

<style scoped>
.readiness-panel { font-size: 0.78rem; }
.banner { padding: 6px 10px; border-radius: var(--radius-sm); margin-bottom: 6px; font-weight: 600; }
.banner-wait { background: rgba(251, 191, 36, 0.12); border: 1px solid rgba(251, 191, 36, 0.35); color: #b45309; }
.banner-fail { background: rgba(239, 68, 68, 0.12); border: 1px solid rgba(239, 68, 68, 0.35); color: #dc2626; }
.check-table { width: 100%; border-collapse: collapse; }
.check-table td { padding: 3px 4px; vertical-align: top; }
.dot { width: 14px; }
.dot-ok { color: #22c55e; }
.dot-bad { color: #ef4444; }
.name { white-space: nowrap; color: var(--text-secondary); }
.detail { color: var(--text-primary); max-width: 150px; overflow-wrap: anywhere; }
.age { white-space: nowrap; color: var(--text-muted); text-align: right; }
.check-bad .detail { color: #dc2626; }
.next-action { margin-top: 6px; color: #2563eb; background: rgba(99, 102, 241, 0.08); border-radius: var(--radius-sm); padding: 5px 8px; }
.stop-reason { color: #b45309; padding: 6px 0; overflow-wrap: anywhere; }
details { margin-top: 8px; overflow-wrap: anywhere; } details p { margin: 5px 0; } summary { cursor: pointer; }
</style>
