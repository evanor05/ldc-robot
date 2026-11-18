import assert from 'node:assert/strict';
import test from 'node:test';
import { simulationMapPolicy } from '../simulationMapPolicy.js';

const settings = { command: 'ros2', min_occupied_cells: 2500, max_unknown_ratio: .85 };
const fixture = JSON.stringify({ minOccupiedCells: 300, maxUnknownRatio: .95, expectedResolution: .05 });

test('real policy and simulation without an explicit fixture keep configured values', () => {
    assert.equal(simulationMapPolicy(settings, {}), settings);
    assert.equal(simulationMapPolicy(settings, { ROBOT_SIMULATION: '1', ROS_DOMAIN_ID: '197' }), settings);
});

test('explicit fixture policy is isolated and uses the canonical quality validation', () => {
    const env = { ROBOT_SIMULATION: '1', ROS_DOMAIN_ID: '197', ROBOT_SIM_MAP_POLICY: fixture };
    assert.deepEqual(simulationMapPolicy(settings, env), {
        command: 'ros2', min_occupied_cells: 300, max_unknown_ratio: .95, expected_resolution: .05 });
    assert.equal(settings.min_occupied_cells, 2500, 'never mutate the real config object');
    for (const override of [{ ROBOT_SIMULATION: '0' }, { ROS_DOMAIN_ID: '0' }, { ROS_DOMAIN_ID: '198' }]) {
        assert.throws(() => simulationMapPolicy(settings, { ...env, ...override }), /隔离 Domain 197/);
    }
    assert.throws(() => simulationMapPolicy(settings, { ...env, ROBOT_SIM_MAP_POLICY: '{invalid' }));
    assert.throws(() => simulationMapPolicy(settings, { ...env, ROBOT_SIM_MAP_POLICY: '{"minOccupiedCells":0}' }), /非法/);
});
