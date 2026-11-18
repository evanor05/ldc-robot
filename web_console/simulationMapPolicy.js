import { qualityOptions } from './mapStorage.js';

// The synthetic room has a known perimeter and a large unknown canvas. Its
// explicit fixture policy must never silently replace real-robot thresholds.
export function simulationMapPolicy(settings, env = process.env) {
    if (!env.ROBOT_SIM_MAP_POLICY) return settings;
    if (env.ROBOT_SIMULATION !== '1' || env.ROS_DOMAIN_ID !== '197') {
        throw new Error('仿真地图策略仅允许用于隔离 Domain 197');
    }
    const gate = qualityOptions(JSON.parse(env.ROBOT_SIM_MAP_POLICY));
    return { ...settings, min_occupied_cells: gate.minOccupiedCells,
        max_unknown_ratio: gate.maxUnknownRatio, expected_resolution: gate.expectedResolution };
}
