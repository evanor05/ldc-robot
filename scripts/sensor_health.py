"""Measurement validation independent of ROS transport and node lifecycle."""
import math

MAX_AGE = 1.5
MAX_FUTURE = .25


def finite(values):
    return all(math.isfinite(v) for v in values)


def pose_valid(pose):
    p, q = pose.position, pose.orientation
    return (finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
            abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) <= .05)


def odom_error(msg):
    if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_link', 'base_footprint'):
        return '里程计坐标系不匹配'
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    if not pose_valid(msg.pose.pose) or not finite((v.x, v.y, v.z, w.x, w.y, w.z)):
        return '里程计数值或四元数无效'
    return ''


def scan_error(msg):
    if msg.header.frame_id != 'laser_frame':
        return '雷达坐标系不匹配'
    if (not msg.ranges or not finite((msg.angle_min, msg.angle_max, msg.angle_increment,
                                    msg.range_min, msg.range_max, msg.scan_time, msg.time_increment)) or
            msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max or
            msg.scan_time < 0 or msg.time_increment < 0):
        return '雷达扫描元数据无效'
    expected = msg.angle_min + (len(msg.ranges)-1)*msg.angle_increment
    if abs(expected-msg.angle_max) > abs(msg.angle_increment)*.5 + 1e-5:
        return '雷达角度与数组长度不一致'
    # +inf is a legitimate no-return measurement. NaNs may occur in individual
    # beams, but a frame containing no usable/no-return beams is not evidence.
    if not any(v == math.inf or (math.isfinite(v) and msg.range_min <= v <= msg.range_max)
               for v in msg.ranges):
        return '雷达扫描没有有效束或无回波束'
    return ''


def automatic_velocity_valid(msg):
    v, w = msg.linear, msg.angular
    return (finite((v.x, v.y, v.z, w.x, w.y, w.z)) and
            abs(v.x) <= .4 and abs(w.z) <= 1.2 and
            max(abs(v.y), abs(v.z), abs(w.x), abs(w.y)) <= 1e-6)


class MeasurementHealth:
    def __init__(self, validator, max_age=MAX_AGE):
        self.validator = validator
        # 每种测量的容忍窗口可独立设置：2D 雷达在固件端按整圈发布，
        # 偶发整帧丢失时用更宽的窗口，避免把短暂抖动误判为断流。
        self.max_age = max_age
        self.received_at = 0.
        self.accepted_at = 0.
        self.stamp_ns = 0
        self.invalid = '尚未收到有效样本'

    def observe(self, msg, ros_ns, monotonic):
        self.received_at = monotonic
        stamp = msg.header.stamp
        stamp_ns = stamp.sec*1_000_000_000 + stamp.nanosec
        error = self.validator(msg)
        age = (ros_ns-stamp_ns)/1e9
        if stamp_ns <= 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            error = '样本时间戳无效'
        elif not -MAX_FUTURE <= age <= MAX_AGE:
            error = '样本时间戳过期或超前'
        if error:
            self.invalid = error
            return False
        if stamp_ns <= self.stamp_ns:
            # Duplicates/reordering never renew validity. A source/clock reset
            # can recover only after the old measurement lease has expired.
            if monotonic-self.accepted_at <= self.max_age or stamp_ns == self.stamp_ns:
                return False
        self.stamp_ns = stamp_ns
        self.accepted_at = monotonic
        self.invalid = ''
        return True

    def check(self, ros_ns, monotonic):
        receipt_age = monotonic-self.received_at if self.received_at else None
        sample_age = (ros_ns-self.stamp_ns)/1e9 if self.stamp_ns else None
        valid_age = monotonic-self.accepted_at if self.accepted_at else None
        ok = (not self.invalid and sample_age is not None and valid_age is not None and
              -MAX_FUTURE <= sample_age <= self.max_age and 0 <= valid_age <= self.max_age)
        detail = '有效样本持续更新' if ok else self.invalid or '有效样本已过期或时间基准变化'
        return dict(ok=bool(ok), ageSeconds=round(valid_age, 2) if valid_age is not None else None,
                    receiptAgeSeconds=round(receipt_age, 2) if receipt_age is not None else None,
                    sampleAgeSeconds=round(sample_age, 2) if sample_age is not None else None,
                    detail=detail)
