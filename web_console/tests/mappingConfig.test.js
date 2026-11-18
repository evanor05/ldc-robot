import fs from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';

const config = JSON.parse(fs.readFileSync(new URL('../ros_config.json', import.meta.url), 'utf8'));
const script = fs.readFileSync(new URL('../../scripts/start_mapping_task.sh', import.meta.url), 'utf8');

test('mapping task delegates to the mode-aware launcher', () => {
    const task = config.tasks.mapping[0];
    assert.equal(task.command, 'bash');
    assert.deepEqual(task.args, ['../scripts/start_mapping_task.sh']);
});

test('mapping launcher keeps manual GMapping as default and auto uses exploration', () => {
    assert.match(script, /ldc_environment --real/);
    assert.match(script, /MODE=manual/);
    assert.match(script, /ros2 launch ldc_exploration auto_mapping\.launch\.py rviz:=false/);
    assert.match(script, /ros2 launch slam_gmapping slam_gmapping\.launch\.py/);
});
