export function yawOf(q = {}) {
  return Math.atan2(2 * ((q.w ?? 1) * (q.z ?? 0) + (q.x ?? 0) * (q.y ?? 0)),
    1 - 2 * ((q.y ?? 0) ** 2 + (q.z ?? 0) ** 2))
}
export function worldToMap(info, x, y) {
  const yaw = yawOf(info.origin.orientation), c = Math.cos(yaw), s = Math.sin(yaw)
  const dx = x - info.origin.position.x, dy = y - info.origin.position.y
  return { x: (c * dx + s * dy) / info.resolution,
    y: info.height - 1 - (-s * dx + c * dy) / info.resolution }
}
export function mapToWorld(info, x, y) {
  const yaw = yawOf(info.origin.orientation), c = Math.cos(yaw), s = Math.sin(yaw)
  const dx = x * info.resolution, dy = (info.height - 1 - y) * info.resolution
  return { x: info.origin.position.x + c * dx - s * dy,
    y: info.origin.position.y + s * dx + c * dy }
}
export function screenToWorld(info, x, y, view) {
  return mapToWorld(info, (x - view.offsetX) / view.scale, (y - view.offsetY) / view.scale)
}

export function headingToGoal(pose, target) {
  if (!pose) return 0
  const dx = target.x - pose.x, dy = target.y - pose.y
  return Math.hypot(dx, dy) < 0.01 ? pose.yaw : Math.atan2(dy, dx)
}

// Canvas coordinates have a downward Y axis; image origins may also be rotated.
export function gridTransform(map, grid) {
  const at = (x, y) => {
    const world = mapToWorld(grid, x, y)
    return worldToMap(map, world.x, world.y)
  }
  const o = at(0, 0), x = at(1, 0), y = at(0, 1)
  return [x.x-o.x, x.y-o.y, y.x-o.x, y.y-o.y, o.x, o.y]
}
