<template>
  <div class="card" v-if="connected">
    <div class="card-header">
      建图控制
    </div>
    <div class="card-body">
      <div v-if="!isMapping" class="mapping-mode" role="radiogroup" aria-label="建图模式">
        <label class="mode-option" :class="{ active: selectedMode === 'manual' }">
          <input type="radio" value="manual" v-model="selectedMode" :disabled="isLoading || !canControl">
          <span>手动驾驶</span>
          <small>摇杆/键盘移动建图</small>
        </label>
        <label class="mode-option" :class="{ active: selectedMode === 'auto' }">
          <input type="radio" value="auto" v-model="selectedMode" :disabled="isLoading || !canControl">
          <span>自动探索</span>
          <small>机器人自主建图</small>
        </label>
      </div>
      <p v-if="selectedMode === 'auto'" class="mapping-status mt-2 auto-warning">
        {{ isMapping ? '自动探索中；请确认急停可用/场地安全' : '自动探索模式：启动前请确认急停可用、场地安全' }}
      </p>
      <div class="mapping-controls">
        <button 
          v-if="!isMapping"
          class="btn btn-connect mapping-btn" 
          @click="startMapping"
          :disabled="isLoading || !canControl"
          :title="canControl ? '' : '观察模式：控制权在其他页面'"
        >
          {{ isLoading ? '正在启动...' : selectedMode === 'auto' ? '开始自动建图' : '开始建图' }}
        </button>
        <button 
          v-else
          class="btn btn-disconnect mapping-btn" 
          @click="stopMapping"
          :disabled="isLoading"
        >
          {{ isLoading && !aborting ? '正在保存并停止...' : '保存并停止建图' }}
        </button>
        <!-- 停止但不保存：差图/保存失败时也必须能停车（P0-B） -->
        <button
          v-if="isMapping"
          class="btn mapping-btn mapping-btn-abort"
          :class="{ 'mapping-btn-abort-urgent': saveFailed }"
          @click="abortMapping"
          :disabled="isLoading"
          title="跳过保存，立即终止建图进程并补发零速"
        >
          {{ aborting ? '正在停止建图…' : '停止但不保存' }}
        </button>
      </div>
      
      <div class="mapping-status mt-2 text-sm text-gray-400">
        <span v-if="isMapping" class="text-green-400">● {{ activeMode === 'auto' ? '自动探索建图' : '手动建图' }}进行中（地图来自 /map 实时数据）</span>
        <span v-else>○ 实时建图未运行</span>
      </div>
      
      <div v-if="errorMsg && !saveFailed" class="error-message mt-2">
        {{ errorMsg }}
      </div>
      <div v-if="saveFailed" class="error-message mt-2">
        保存失败：{{ saveErrorText }}<template v-if="isMapping">，可点「停止但不保存」直接停车</template><template v-else>；建图已停止</template>
      </div>
      <div v-if="abortNotice" class="mapping-status mt-2 text-sm text-green-400">
        {{ abortNotice }}
      </div>
      <div v-if="savedMapName" class="mapping-status mt-2 text-sm text-green-400">
        最近保存地图：{{ savedMapName }}<template v-if="savedAtText"> · {{ savedAtText }}</template>，可在导航地图列表中选择。
      </div>
      <div v-if="!canControl && !isMapping" class="mapping-status mt-2 text-sm text-gray-400">
        观察模式：开始建图需先取得/接管控制权
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, onUnmounted } from 'vue'
import { useMappingTask } from '../composables/useMappingTask.js'

const props = defineProps({
  connected: Boolean,
  sessionId: { type: String, default: '' },
  canControl: { type: Boolean, default: true }
})

const emit = defineEmits(['mappingStarted', 'update:isMapping', 'last-saved'])

// 任务状态机（开始/保存并停止/停止但不保存 + 轮询）在 composable 里，
// 组件只负责展示与事件转发；行为契约由 tests/mappingAbortUi.test.js 与
// tests/mappingMode.test.js 盯着。
const API_BASE = `http://${window.location.hostname}:${import.meta.env.VITE_ROBOT_API_PORT || 3000}/api`
const {
  isMapping, isLoading, errorMsg, savedMapName, savedAtText,
  saveFailed, saveErrorText, aborting, abortNotice,
  selectedMode, activeMode,
  checkStatus, startMapping, stopMapping, abortMapping,
  startPolling, stopPolling
} = useMappingTask({ apiBase: API_BASE, sessionId: () => props.sessionId, emit })

onMounted(startPolling)
onUnmounted(stopPolling)
</script>

<style scoped>
.mapping-mode {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
  margin-bottom: 10px;
}
.mode-option {
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 8px 10px;
  border: 1px solid var(--border-color);
  border-radius: var(--radius-sm);
  background: #f8fafc;
  cursor: pointer;
  font-size: 0.8rem;
  color: var(--text-secondary);
}
.mode-option.active {
  border-color: var(--accent-primary);
  background: rgba(99, 102, 241, 0.12);
  color: var(--text-primary);
}
.mode-option input { margin: 0 0 4px; accent-color: var(--accent-primary); }
.mode-option small { font-size: 0.7rem; color: var(--text-muted); }
.mode-option:has(input:disabled) { opacity: 0.55; cursor: not-allowed; }
.auto-warning { color: #b45309; background: rgba(251, 191, 36, 0.12); border: 1px solid rgba(251, 191, 36, 0.3); border-radius: var(--radius-sm); padding: 5px 8px; }
.mapping-controls {
  display: flex;
  flex-direction: column;
  gap: 8px;
}
.mapping-btn {
  width: 100%;
  font-weight: 600;
  padding: 12px;
}
/* 停止但不保存：次要危险操作，与「保存并停止」明确区分 */
.mapping-btn-abort {
  padding: 9px;
  font-size: 0.85rem;
  background: transparent;
  border: 1px dashed #d97706;
  color: #b45309;
  border-radius: var(--radius-sm);
  cursor: pointer;
}
.mapping-btn-abort:hover:not(:disabled) {
  background: rgba(251, 191, 36, 0.14);
}
.mapping-btn-abort-urgent {
  border-style: solid;
  border-color: #dc2626;
  color: #b91c1c;
  background: rgba(220, 38, 38, 0.08);
}
.mapping-btn-abort:disabled { opacity: 0.6; cursor: not-allowed; }
.mt-2 { margin-top: 8px; }
.text-sm { font-size: 0.85rem; }
.text-gray-400 { color: #9ca3af; }
.text-green-400 { color: #34d399; }
</style>
