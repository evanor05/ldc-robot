"""AMCL quality and motion-aware freshness, independent of the ROS executor.

An idle AMCL need not publish periodically. Once odometry exceeds AMCL's
update distance/angle, a new, time-aligned localization is required. A large
map correction is latched until an explicit initial-pose request.
"""
from collections import deque
import math
import numpy as np


def stamp_ns(msg):
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def planar_pose(pose):
    p, q = pose.position, pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def pose_error(msg):
    if msg.header.frame_id != 'map':
        return 'frame', 'AMCL 坐标系必须为 map'
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        return 'nonfinite', 'AMCL 位姿包含非有限数'
    if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .05:
        return 'quaternion', 'AMCL 四元数未归一化'
    cov = np.asarray(msg.pose.covariance, dtype=float)
    if cov.size != 36 or not np.isfinite(cov).all():
        return 'covariance', 'AMCL 协方差无效'
    cov = cov.reshape(6, 6)
    if not np.allclose(cov, cov.T, atol=1e-8, rtol=1e-6) or np.linalg.eigvalsh(cov).min() < -1e-8:
        return 'covariance', 'AMCL 协方差必须对称且半正定'
    # Zero covariance carries no useful uncertainty estimate for planar pose.
    if min(cov[0, 0], cov[1, 1], cov[5, 5]) <= 0:
        return 'covariance', 'AMCL 平面协方差缺失或为零'
    return '', ''


class LocalizationHealth:
    def __init__(self, update_distance=.25, update_angle=.2, update_timeout=1.5,
                 max_sigma_xy=.75, max_sigma_yaw=.75):
        self.update_distance = update_distance
        self.update_angle = update_angle
        self.update_timeout = update_timeout
        self.max_sigma_xy, self.max_sigma_yaw = max_sigma_xy, max_sigma_yaw
        self.odometry = deque(maxlen=500)
        self.reset()

    def reset(self, requested_ns=None):
        self.requested_ns = requested_ns
        self.accepted_ns = 0
        self.accepted_at = 0.
        self.last_pose = self.anchor = None
        self.pending_motion_at = None
        self.sigma_xy = self.sigma_yaw = None
        self.code = 'waiting_initial_pose' if requested_ns is None else 'waiting_update'
        self.detail = '请设置初始位置' if requested_ns is None else '等待本次初始位置之后的有效 AMCL 更新'
        self.latched = False
        self.rejected = 0

    def observe_odometry(self, msg, now):
        pose = planar_pose(msg.pose.pose)
        self.odometry.append((stamp_ns(msg), pose))
        if self.anchor is not None and self.motion_requires_update(pose, self.anchor):
            if self.pending_motion_at is None:
                self.pending_motion_at = now

    def motion_requires_update(self, pose, reference):
        # Match AMCL shouldUpdateFilter(): strict thresholds on each odom axis.
        return (abs(pose[0]-reference[0]) > self.update_distance or
                abs(pose[1]-reference[1]) > self.update_distance or
                abs(angle(pose[2]-reference[2])) > self.update_angle)

    def aligned_odometry(self, stamp):
        if not self.odometry:
            return None
        sample = min(self.odometry, key=lambda item: abs(item[0]-stamp))
        return sample[1] if abs(sample[0]-stamp) <= 250_000_000 else None

    def reject(self, code, detail, latch=False):
        self.rejected += 1
        self.code, self.detail = code, detail
        self.latched = self.latched or latch
        return False

    def observe(self, msg, ros_ns, now):
        if self.requested_ns is None or self.latched:
            return False
        code, detail = pose_error(msg)
        if code:
            return self.reject(code, detail)
        stamp = stamp_ns(msg)
        if stamp <= 0 or not -.25 <= (ros_ns-stamp)/1e9 <= 1.5:
            return self.reject('timestamp', 'AMCL 时间戳过期、超前或无效')
        if stamp < self.requested_ns or stamp <= self.accepted_ns:
            # Delayed DDS messages cannot confirm a new initialization or renew health.
            self.rejected += 1
            return False
        covariance = msg.pose.covariance
        sigma_xy = math.sqrt(max(covariance[0], covariance[7]))
        sigma_yaw = math.sqrt(covariance[35])
        if sigma_xy > self.max_sigma_xy or sigma_yaw > self.max_sigma_yaw:
            return self.reject('uncertainty', 'AMCL 不确定度超过诊断阈值，请重新定位')
        odom = self.aligned_odometry(stamp)
        if odom is None:
            return self.reject('unaligned', 'AMCL 与原始里程计时间无法配对（超过 0.25 秒）')
        pose = planar_pose(msg.pose.pose)
        if self.last_pose is not None and self.anchor is not None:
            yaw_offset = self.last_pose[2]-self.anchor[2]
            dx, dy = odom[0]-self.anchor[0], odom[1]-self.anchor[1]
            expected_x = self.last_pose[0]+math.cos(yaw_offset)*dx-math.sin(yaw_offset)*dy
            expected_y = self.last_pose[1]+math.sin(yaw_offset)*dx+math.cos(yaw_offset)*dy
            residual = math.hypot(pose[0]-expected_x, pose[1]-expected_y)
            yaw_residual = abs(angle(pose[2]-self.last_pose[2]-angle(odom[2]-self.anchor[2])))
            if residual > .75 or yaw_residual > math.pi/3:
                return self.reject('jump', 'AMCL 相对轮式运动发生大幅跳变，需重新设置初始位置', True)
        self.accepted_ns, self.accepted_at = stamp, now
        self.last_pose, self.anchor = pose, odom
        self.sigma_xy, self.sigma_yaw = sigma_xy, sigma_yaw
        self.pending_motion_at = None
        self.code, self.detail = 'valid', 'AMCL 位姿、协方差与来源时间有效'
        # The newest odom may already be newer than this AMCL scan.
        if self.odometry:
            newest = self.odometry[-1][1]
            if self.motion_requires_update(newest, odom):
                self.pending_motion_at = now
        return True

    def check(self, ros_ns, now):
        code, detail = self.code, self.detail
        if code == 'valid':
            if ros_ns < self.accepted_ns-250_000_000:
                code, detail = 'clock_reset', 'ROS 时间回拨，请重新设置初始位置'
                self.latched = True
                self.code, self.detail = code, detail
            elif self.pending_motion_at is not None and now-self.pending_motion_at > self.update_timeout:
                code, detail = 'motion_stale', '车辆已移动，但 AMCL 未及时更新'
            elif self.pending_motion_at is None:
                code, detail = 'stationary', '定位有效；未达到 AMCL 运动更新阈值'
        return dict(ok=code in ('valid', 'stationary'), code=code, detail=detail,
                    # 请求/接受时间戳外露：失败时页面与 run manifest 要能报出"用哪条
                    # 初始位姿请求、最后接受的 AMCL 位姿是哪条"，而不是只说"启动超时"。
                    requestedStamp=self.requested_ns or None,
                    acceptedStamp=self.accepted_ns or None,
                    waitingUser=self.requested_ns is None or not self.accepted_ns,
                    ageSeconds=round(now-self.accepted_at, 3) if self.accepted_at else None,
                    sampleAgeSeconds=round((ros_ns-self.accepted_ns)/1e9, 3) if self.accepted_ns else None,
                    sigmaXY=self.sigma_xy, sigmaYaw=self.sigma_yaw,
                    motionPendingSeconds=round(now-self.pending_motion_at, 3) if self.pending_motion_at is not None else None,
                    requiresInitialPose=self.latched, rejected=self.rejected,
                    limits=dict(updateDistance=self.update_distance, updateAngle=self.update_angle,
                                updateTimeout=self.update_timeout, maxSigmaXY=self.max_sigma_xy,
                                maxSigmaYaw=self.max_sigma_yaw))
