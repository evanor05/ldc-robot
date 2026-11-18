/**
 * 建图任务状态机（从 MappingControl.vue 抽出）。
 *
 * 保持的行为契约（都有测试盯着）：
 * - start / stop(保存并停止) / abort(停止但不保存) 三条路径互不混淆；
 * - 保存失败绝不阻断"已停止"的事实：后端返回 stopped=true 时面板必须退出"建图中"，
 *   并把原因显示出来，同时保留「停止但不保存」可点；
 * - 中止（abort）前必须二次确认，成功后不触发保存；
 * - 状态轮询以服务端 mode 为准并回填用户选择；停止后保留用户选择。
 */
import { ref } from 'vue'

export function newRequestId() {
    return (typeof crypto !== 'undefined' && crypto.randomUUID)
        ? crypto.randomUUID()
        : `req-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
}

export function useMappingTask({ apiBase, sessionId, emit = () => {} }) {
    const isMapping = ref(false)
    const isLoading = ref(false)
    const errorMsg = ref('')
    const savedMapName = ref('')
    const savedAtText = ref('')
    // 保存失败与中止建图是两条独立路径：保存失败绝不能让面板卡在"建图中"。
    const saveFailed = ref(false)
    const saveErrorText = ref('')
    const aborting = ref(false)
    const abortNotice = ref('')
    // 用户本次选择；停止后保留；建图中以服务端 mode 为准并回填。
    const selectedMode = ref('manual')
    const activeMode = ref('')
    let statusTimer = null

    const currentSession = () => (typeof sessionId === 'function' ? sessionId() : sessionId)

    async function checkStatus() {
        try {
            const res = await fetch(`${apiBase}/mapping_status`)
            const data = await res.json()
            isMapping.value = data.isRunning
            emit('update:isMapping', data.isRunning)
            if (data.isRunning) {
                // 建图中以服务端实际模式为准（可能由其他页面启动），并回填选择。
                activeMode.value = data.mode === 'auto' ? 'auto' : 'manual'
                selectedMode.value = activeMode.value
            } else {
                // 停止后保留用户本次选择，不强制回到手动。
                activeMode.value = ''
            }
            if (data.lastSavedMap) {
                savedMapName.value = data.lastSavedMap.name
                savedAtText.value = data.lastSavedMap.savedAt
                    ? new Date(data.lastSavedMap.savedAt).toLocaleString('zh-CN') : ''
                emit('last-saved', data.lastSavedMap)
            }
        } catch (err) {
            console.error('获取建图状态失败:', err)
        }
    }

    async function startMapping() {
        isLoading.value = true
        errorMsg.value = ''
        try {
            // 变更类任务统一携带控制会话与请求标识；无控制页面时该操作即取得控制权。
            const res = await fetch(`${apiBase}/start_mapping`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sessionId: currentSession(), requestId: newRequestId(),
                    mode: selectedMode.value })
            })
            const data = await res.json()
            if (data.success) {
                activeMode.value = selectedMode.value
                isMapping.value = true
                emit('update:isMapping', true)
                emit('mappingStarted')
            } else {
                errorMsg.value = data.error || '启动失败'
            }
        } catch (err) {
            errorMsg.value = '无法连接到本地控制服务'
        } finally {
            isLoading.value = false
        }
    }

    async function stopMapping() {
        isLoading.value = true
        errorMsg.value = ''
        saveFailed.value = false
        abortNotice.value = ''
        try {
            const res = await fetch(`${apiBase}/stop_mapping`, { method: 'POST' })
            const data = await res.json()
            if (data.success) {
                isMapping.value = false
                activeMode.value = ''
                emit('update:isMapping', false)
                if (data.savedMap) {
                    savedMapName.value = data.savedMap.name
                    savedAtText.value = new Date(data.savedMap.savedAt || Date.now()).toLocaleString('zh-CN')
                    emit('last-saved', { ...data.savedMap, savedAt: data.savedMap.savedAt || Date.now() })
                    if (typeof window !== 'undefined' && window.dispatchEvent) {
                        window.dispatchEvent(new CustomEvent('maps-updated', { detail: data.savedMap }))
                    }
                }
            } else if (data.stopped) {
                // 后端已停止建图但保存失败：状态必须跟随真实进程状态，不能停留在"建图中"。
                isMapping.value = false
                activeMode.value = ''
                emit('update:isMapping', false)
                saveFailed.value = true
                saveErrorText.value = data.error || '未知原因'
            } else {
                // 未确认停止：保留"建图中"，并让「停止但不保存」保持可点。
                errorMsg.value = data.error || '停止失败'
            }
        } catch (err) {
            errorMsg.value = '无法连接到本地控制服务'
        } finally {
            isLoading.value = false
        }
    }

    // 停止建图但不保存：跳过 saveMap，直接终止进程组并补发零速。
    async function abortMapping() {
        const confirmed = typeof window !== 'undefined' && typeof window.confirm === 'function'
            ? window.confirm('停止建图且不保存？当前未保存的地图会丢失，机器人将立即停车。')
            : true
        if (!confirmed) return
        isLoading.value = true
        aborting.value = true
        errorMsg.value = ''
        abortNotice.value = ''
        try {
            const res = await fetch(`${apiBase}/abort_mapping`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sessionId: currentSession(), requestId: newRequestId() })
            })
            const data = await res.json()
            if (data.success) {
                isMapping.value = false
                activeMode.value = ''
                saveFailed.value = false
                saveErrorText.value = ''
                emit('update:isMapping', false)
                abortNotice.value = data.message || '建图已中止（未保存）'
            } else {
                errorMsg.value = data.error || '中止建图失败'
            }
        } catch (err) {
            errorMsg.value = '无法连接到本地控制服务'
        } finally {
            isLoading.value = false
            aborting.value = false
        }
    }

    function startPolling(intervalMs = 3000) {
        checkStatus()
        statusTimer = setInterval(checkStatus, intervalMs)
    }

    function stopPolling() {
        if (statusTimer) clearInterval(statusTimer)
        statusTimer = null
    }

    return {
        isMapping, isLoading, errorMsg, savedMapName, savedAtText,
        saveFailed, saveErrorText, aborting, abortNotice,
        selectedMode, activeMode,
        checkStatus, startMapping, stopMapping, abortMapping,
        startPolling, stopPolling
    }
}
