// ROS TF transforms map child coordinates into the parent frame.
export const frameName = name => (name || '').replace(/^\/+/, '');
const identity = () => ({ translation: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } });

function quaternion(q = {}) {
  const values = [q.x ?? 0, q.y ?? 0, q.z ?? 0, q.w ?? 1];
  const n = Math.hypot(...values);
  if (!Number.isFinite(n) || n < 1e-12) return null;
  return { x: values[0] / n, y: values[1] / n, z: values[2] / n, w: values[3] / n };
}
function multiply(a, b) {
  return { x: a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
    y: a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
    z: a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
    w: a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z };
}
function rotate(q, p) {
  const r = multiply(multiply(q, { x: p.x, y: p.y, z: p.z ?? 0, w: 0 }),
    { x: -q.x, y: -q.y, z: -q.z, w: q.w });
  return { x: r.x, y: r.y, z: r.z };
}
export function transformPoint(tf, p) {
  const r = rotate(tf.rotation, p);
  return { x: r.x + tf.translation.x, y: r.y + tf.translation.y, z: r.z + tf.translation.z };
}
function compose(a, b) {
  return { translation: transformPoint(a, b.translation), rotation: multiply(a.rotation, b.rotation) };
}
function inverse(tf) {
  const q = tf.rotation, rotation = { x: -q.x, y: -q.y, z: -q.z, w: q.w };
  return { rotation, translation: rotate(rotation,
    { x: -tf.translation.x, y: -tf.translation.y, z: -tf.translation.z }) };
}
export function cacheTransforms(tree, message, receivedAt = Date.now()) {
  for (const item of message?.transforms || []) {
    const parent = frameName(item.header?.frame_id), child = frameName(item.child_frame_id);
    const q = quaternion(item.transform?.rotation), p = item.transform?.translation;
    if (!parent || !child || parent === child || !q || !p ||
        ![p.x, p.y, p.z ?? 0].every(Number.isFinite)) continue;
    tree[`${parent}|${child}`] = { parent, child, receivedAt, static: message.static === true,
      transform: { rotation: q, translation: { x: p.x, y: p.y, z: p.z ?? 0 } } };
  }
}
export function lookupTransform(tree, parent, child, now = Date.now()) {
  parent = frameName(parent); child = frameName(child);
  if (!parent || !child) return null;
  const pending = [{ frame: parent, transform: identity() }], seen = new Set([parent]);
  for (let i = 0; i < pending.length; i++) {
    const current = pending[i];
    if (current.frame === child) return current.transform;
    for (const edge of Object.values(tree)) {
      if (!edge.static && now - edge.receivedAt > 1500) continue;
      const next = edge.parent === current.frame ? edge.child : edge.child === current.frame ? edge.parent : null;
      if (!next || seen.has(next)) continue;
      seen.add(next);
      pending.push({ frame: next, transform: compose(current.transform,
        edge.parent === current.frame ? edge.transform : inverse(edge.transform)) });
    }
  }
  return null;
}
