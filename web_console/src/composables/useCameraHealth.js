/**
 * 相机健康状态机（从 CameraView.vue 抽出，纯状态、无 DOM）。
 *
 * 解决的真实问题：
 * - running（ROS 视觉进程在跑）与 healthy（相机真的有画面）必须分开，
 *   否则 fps=0 时界面仍显示"健康"；
 * - raw 与 annotated 必须各有独立的帧身份做看门狗，共用一条身份时
 *   raw 卡死会被 annotated 的帧推进掩盖；
 * - 相机离线要有可读的重连倒计时，而不是只显示上一个正常状态。
 */
import { computed, ref } from 'vue'

export const FRAME_STALL_MS = 3000
export const AUTO_RELOAD_MIN_MS = 5000

export function useCameraHealth({ previewKind }) {
    const state = ref({ running: false, starting: false, detections: [] })
    // 每种预览各自的帧身份（网关 previewFrames.<kind> + 检测帧 id / 完成数）
    const frameWatch = { raw: { token: '', at: Date.now() }, annotated: { token: '', at: Date.now() } }
    const offlineSeconds = ref(0)
    let offlineSince = 0
    let lastAutoReloadAt = 0

    const managed = computed(() => state.value.running || state.value.starting)
    const cameraHealthy = computed(() => Boolean(state.value.camera?.ok))
    const cameraStatusClass = computed(() => (cameraHealthy.value ? 'is-ok' : 'is-off'))
    const cameraStatusText = computed(() => {
        if (cameraHealthy.value) return '实时画面正常'
        if (managed.value) return `摄像头离线（已 ${offlineSeconds.value}s，重连中）`
        return state.value.camera?.detail || '未连接'
    })
    // 当前预览这一路是否真的有帧来源：raw 看相机，annotated 还要识别服务就绪。
    const previewHealthy = computed(() => (previewKind.value === 'raw'
        ? cameraHealthy.value
        : cameraHealthy.value && Boolean(state.value.detector?.ok)))

    function resetWatch() {
        frameWatch.raw.at = frameWatch.annotated.at = Date.now()
    }

    /**
     * 每次轮询调用一次：更新离线计时与两条看门狗。
     * @returns {boolean} 当前预览这一路是否卡住（调用方据此重建 MJPEG 连接）
     */
    function observe(current) {
        const running = Boolean(current.running || current.starting)
        const cameraOk = Boolean(current.camera?.ok)
        const detectorOk = Boolean(current.detector?.ok)
        const now = Date.now()
        if (running && !cameraOk) {
            if (!offlineSince) offlineSince = now
            offlineSeconds.value = Math.max(1, Math.floor((now - offlineSince) / 1000))
        } else {
            offlineSince = 0
            offlineSeconds.value = 0
        }
        const tokens = {
            raw: `pf:${current.previewFrames?.raw ?? ''}`,
            annotated: `pf:${current.previewFrames?.annotated ?? ''}`
                + `|c:${current.completed ?? ''}|f:${current.frame?.id ?? ''}`
        }
        const healthy = { raw: cameraOk, annotated: cameraOk && detectorOk }
        let reload = false
        for (const kind of ['raw', 'annotated']) {
            const watch = frameWatch[kind]
            if (!running || !healthy[kind]) {
                watch.at = now
                continue
            }
            if (tokens[kind] !== watch.token) {
                watch.token = tokens[kind]
                watch.at = now
                continue
            }
            if (kind === previewKind.value && now - watch.at > FRAME_STALL_MS
                && now - lastAutoReloadAt > AUTO_RELOAD_MIN_MS) {
                // 该路卡住但 HTTP 没报错：允许调用方重建这一路 MJPEG 连接（有冷却窗口）。
                lastAutoReloadAt = now
                watch.at = now
                reload = true
            }
        }
        return reload
    }

    return {
        state, managed, frameWatch, offlineSeconds,
        cameraHealthy, cameraStatusClass, cameraStatusText, previewHealthy,
        observe, resetWatch
    }
}
