import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { CdrReader } from '../src/composables/useFoxglove.js'
import { decodeBatteryState, batteryDisplay } from '../src/lib/battery.js'

test('decode actual ROS Lyrical serialized BatteryState, with aligned arrays and NaNs', () => {
  const fixture = readFileSync(new URL('./fixtures/battery-state.cdr', import.meta.url))
  const buffer = fixture.buffer.slice(fixture.byteOffset, fixture.byteOffset + fixture.byteLength)
  const reader = new CdrReader(buffer, 4)
  const data = decodeBatteryState(reader)
  assert.deepEqual(data.header, { stamp: { sec: 123, nanosec: 456 }, frame_id: 'battery' })
  assert.ok(Math.abs(data.voltage - 7.8) < 1e-5)
  assert.ok(Number.isNaN(data.temperature))
  assert.equal(data.current, -0.25)
  assert.equal(data.percentage, 0.5)
  assert.equal(data.present, true)
  assert.equal(data.cell_voltage.length, 2)
  assert.ok(Number.isNaN(data.cell_temperature[0]))
  assert.equal(data.location, 'main')
  assert.equal(data.serial_number, 'fixture')
  assert.equal(reader.pos, buffer.byteLength)
  assert.equal(batteryDisplay(data).percentage, 50)
})
test('unknown state-of-charge stays unknown, voltage uses volts', () => {
  assert.deepEqual(batteryDisplay({ voltage: 7.8, percentage: NaN, present: true }), { voltage: 7.8, percentage: -1 })
  assert.deepEqual(batteryDisplay({ voltage: 7.8, percentage: 0.5, present: false }), { voltage: 0, percentage: -1 })
  assert.deepEqual(batteryDisplay({ data: 78 }), { voltage: 7.8, percentage: -1 })
})
