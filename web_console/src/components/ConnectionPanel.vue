<template>
  <div class="card">
    <div class="card-header">
      连接
    </div>
    <div class="card-body connection-controls">
      <button
        v-if="!isConnected"
        class="btn btn-connect"
        @click="$emit('connect')"
        :disabled="connectionState === 'connecting'"
      >
        {{ connectionState === 'connecting' ? '连接中...' : '连接' }}
      </button>

      <button
        v-else
        class="btn btn-disconnect"
        @click="$emit('disconnect')"
      >
        断开数据桥接
      </button>

      <!-- 连接成功提示 -->
      <div class="status-indicator" v-if="isConnected">
        <span class="status-dot connected"></span>
        <span>数据桥接已连接</span>
      </div>
      <div class="status-indicator" v-if="isConnected" role="status" aria-live="polite">
        <span class="status-dot" :class="vehicleOnline ? 'connected' : 'disconnected'"></span>
        <span>{{ vehicleStatusText || '等待小车数据' }}</span>
      </div>

      <!-- 错误信息 -->
      <div class="error-message" v-if="errorMessage">
        {{ errorMessage }}
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed } from 'vue'

const props = defineProps({
  connectionState: String,
  vehicleOnline: Boolean,
  vehicleStatusText: String,
  errorMessage: String
})

const emit = defineEmits(['connect', 'disconnect'])

const isConnected = computed(() => props.connectionState === 'connected')
</script>
