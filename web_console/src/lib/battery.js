// sensor_msgs/msg/BatteryState, in ROS IDL field order. Voltage is in volts;
// percentage is a fraction in [0, 1], with NaN denoting unknown.
export function decodeBatteryState(reader) {
  const header = {
    stamp: { sec: reader.readInt32(), nanosec: reader.readUint32() },
    frame_id: reader.readString()
  }
  const result = { header }
  for (const name of ['voltage', 'temperature', 'current', 'charge', 'capacity', 'design_capacity', 'percentage']) {
    result[name] = reader.readFloat32()
  }
  for (const name of ['power_supply_status', 'power_supply_health', 'power_supply_technology']) {
    result[name] = reader.readUint8()
  }
  result.present = reader.readUint8() !== 0
  result.cell_voltage = reader.readFloat32Array()
  result.cell_temperature = reader.readFloat32Array()
  result.location = reader.readString()
  result.serial_number = reader.readString()
  return result
}

export function batteryDisplay(data) {
  if (typeof data.voltage === 'number') {
    return {
      voltage: Number.isFinite(data.voltage) && data.present !== false ? data.voltage : 0,
      percentage: Number.isFinite(data.percentage) && data.percentage >= 0 && data.percentage <= 1 && data.present !== false
        ? Math.round(data.percentage * 100) : -1
    }
  }
  // Old UInt16 devices provide decivolts but no state-of-charge measurement.
  return { voltage: Number.isFinite(data.data) ? data.data / 10 : 0, percentage: -1 }
}
