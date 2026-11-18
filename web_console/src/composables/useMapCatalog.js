/**
 * 地图目录与质量门控（从 NavigationControl.vue 抽出）。
 *
 * 这里承载的是 P0-A 的验收语义，必须保持：
 * - 只有 navigable !== false 的图能当默认选择；差图不可选、要标出原因；
 * - 用户手选后（mapChoiceTouched）轮询/刷新都不得改选；
 * - 刷新后按 name 重新绑定（path 可能随目录迁移变化），找不到就保留并提示，
 *   绝不静默改选；
 * - 质量缺失时文案不得出现 NaN 或伪造的 0 栅格。
 */
import { computed, ref } from 'vue'

export function useMapCatalog({ apiBase, isLoading, errorMsg }) {
    const maps = ref([])
    const selectedMap = ref('')
    // 用户手动选图后，状态轮询不能把选择覆盖回当前导航地图（否则新地图看起来「选不中」）。
    const mapChoiceTouched = ref(false)
    // 后端返回的门槛用于文案展示；拿不到时按生产门槛兜底，避免出现两套数字。
    const thresholds = ref({ minOccupiedCells: 2500, maxUnknownRatio: 0.85 })

    const selectedMapItem = computed(() => maps.value.find(m => m.path === selectedMap.value))
    const thresholdUnknownPercent = computed(() => (thresholds.value.maxUnknownRatio * 100).toFixed(0))
    const selectedMapName = computed(() => {
        if (selectedMapItem.value) return selectedMapItem.value.name
        return selectedMap.value ? selectedMap.value.split('/').pop() : ''
    })
    // 选过图但列表里已经没有它：不允许静默改选，必须提示用户重选。
    const selectedMapMissing = computed(() => Boolean(
        mapChoiceTouched.value && selectedMap.value && !selectedMapItem.value))
    const hasNavigableMap = computed(() => maps.value.some(m => m.navigable !== false))

    // 单张地图的质量文案：质量缺失时绝不显示 NaN 或伪造的 0 栅格。
    function mapQualityLabel(map) {
        if (!map) return '质量未知'
        if (map.navigable === false) {
            return `不可导航：${String(map.qualityError || '原因未知').slice(0, 60)}`
        }
        const quality = map.quality
        if (!quality || !Number.isFinite(quality.occupied) || !Number.isFinite(quality.unknownRatio)) {
            return '质量未知'
        }
        return `${quality.occupied} 栅格 · 未知 ${(quality.unknownRatio * 100).toFixed(1)}%`
    }

    async function loadMaps() {
        isLoading.value = true
        try {
            const res = await fetch(`${apiBase}/list_maps`)
            const data = await res.json()
            // 刷新前记住用户选择的 name：path 可能随目录迁移变化，不能用旧 path 判断存在性。
            const previousName = selectedMap.value ? selectedMap.value.split('/').pop() : ''
            maps.value = data.maps || []
            if (data.thresholds) thresholds.value = data.thresholds
            if (selectedMap.value) {
                // 已选过图：只按 name 重新绑定；找不到保留原选择，交给提示去说，绝不自动改选。
                const rebound = maps.value.find(m => m.name === previousName)
                if (rebound) selectedMap.value = rebound.path
                return
            }
            if (!mapChoiceTouched.value) {
                // 默认只选第一张合格图；一张合格图都没有时保持空选择并给出提示。
                const firstNavigable = maps.value.find(m => m.navigable !== false)
                if (firstNavigable) selectedMap.value = firstNavigable.path
            }
        } catch (err) {
            console.error('加载地图列表失败:', err)
            if (errorMsg) errorMsg.value = '无法加载地图列表'
        } finally {
            isLoading.value = false
        }
    }

    function onMapChange() { mapChoiceTouched.value = true }

    // 唯一允许把 mapChoiceTouched 复位成 false 的用户操作。
    function clearMapChoice() {
        selectedMap.value = ''
        mapChoiceTouched.value = false
        if (errorMsg) errorMsg.value = ''
    }

    /**
     * 服务端报告"当前加载的是这张图"时同步选择：
     * 用户没手选过才跟随；而且只接受合格图（差图不得被默认选中）。
     */
    function adoptServerMapPath(mapPath) {
        if (!mapPath || mapChoiceTouched.value) return false
        const map = maps.value.find(m => m.path === mapPath)
        if (map && map.navigable === false) return false
        selectedMap.value = mapPath
        return true
    }

    return {
        maps, selectedMap, mapChoiceTouched, thresholds, thresholdUnknownPercent,
        selectedMapItem, selectedMapName, selectedMapMissing, hasNavigableMap,
        mapQualityLabel, loadMaps, onMapChange, clearMapChoice, adoptServerMapPath
    }
}
