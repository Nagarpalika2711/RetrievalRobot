#!/usr/bin/env python3
"""Known-map cylinder localization with a planar EKF.

The cylinder detector publishes visualization MarkerArray poses in base_link.
Marker.id and Marker.color are NOT physical landmark IDs or measured RGB colors.
Association uses predicted positions, unique nearest-neighbor, and Mahalanobis gating.
The initial robot map pose must be approximately known; not global localization.
"""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def rotation(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class LandmarkLocalizer(Node):
    def __init__(self):
        super().__init__('landmark_localizer')
        defaults = {
            'landmarks_json': '', 'initial_x': 0.0, 'initial_y': 0.0,
            'initial_yaw': 0.0, 'initial_std_xy': 0.25,
            'initial_std_yaw': 0.25, 'measurement_std_m': 0.12,
            'process_std_xy_per_m': 0.08, 'process_std_yaw_per_rad': 0.10,
            'association_gate_m': 0.75, 'mahalanobis_gate': 9.21,
            'max_measurement_age_s': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        def param(n): return self.get_parameter(n).value
        with open(str(param('landmarks_json')), encoding='utf8') as f:
            self.landmarks = json.load(f)['landmarks']
        self.X = np.array([param('initial_x'), param('initial_y'), param('initial_yaw')], dtype=float)
        self.P = np.diag([param('initial_std_xy')**2, param('initial_std_xy')**2,
                          param('initial_std_yaw')**2]).astype(float)
        self.last_odom = None
        self.current_odom = None
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 30)
        self.mark_sub = self.create_subscription(MarkerArray, '/viz/detections', self.on_detections, 10)
        self.timer = self.create_timer(0.05, self.publish_estimate)
        self.get_logger().info('EKF localizer started; geometry-only association, known initial map pose.')

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        cur = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw(msg.pose.pose.orientation)], dtype=float)
        if self.last_odom is None or t <= self.last_odom[0]:
            if self.last_odom and t < self.last_odom[0]:
                self.get_logger().warn('Simulation clock reset: new odometry baseline')
            self.last_odom = (t, cur)
            self.current_odom = cur
            return
        old = self.last_odom[1]
        # Incremental movement expressed in the OLD robot frame.
        dxy_body = rotation(-old[2]) @ (cur[:2] - old[:2])
        dtheta = wrap(cur[2] - old[2])
        old_yaw = self.X[2]
        global_delta = rotation(old_yaw) @ dxy_body
        self.X[:2] += global_delta
        self.X[2] = wrap(old_yaw + dtheta)
        # Jacobian of pose update with respect to old state.
        F = np.eye(3)
        F[0, 2] = -global_delta[1]
        F[1, 2] = global_delta[0]
        travel = float(np.linalg.norm(dxy_body))
        sigma_xy = 0.003 + float(self.get_parameter('process_std_xy_per_m').value)*travel
        sigma_theta = 0.002 + float(self.get_parameter('process_std_yaw_per_rad').value)*abs(dtheta)
        Q = np.diag([sigma_xy**2, sigma_xy**2, sigma_theta**2])
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5*(self.P + self.P.T)
        self.last_odom = (t, cur)
        self.current_odom = cur

    def on_detections(self, msg):
        if self.current_odom is None:
            return
        observations = []
        for mark in msg.markers:
            if mark.action != Marker.ADD or mark.type != Marker.CYLINDER:
                continue
            if mark.header.frame_id != 'base_link':
                # No frame conversion in this implementation. Detector must target base_link.
                continue
            observations.append(np.array([mark.pose.position.x, mark.pose.position.y]))
        if not observations:
            return
        # Use the newest detector timestamp to reject out-of-date measurements.
        stamp = max((m.header.stamp.sec + 1e-9*m.header.stamp.nanosec
                     for m in msg.markers if m.action == Marker.ADD and m.type == Marker.CYLINDER), default=0.)
        now = self.get_clock().now().nanoseconds*1e-9
        if stamp and (stamp > now + 0.1 or now-stamp > float(self.get_parameter('max_measurement_age_s').value)):
            return
        Rnoise = np.eye(2)*float(self.get_parameter('measurement_std_m').value)**2
        gate = float(self.get_parameter('association_gate_m').value)
        chi2_gate = float(self.get_parameter('mahalanobis_gate').value)
        # Candidate associations based on predicted local positions. Every physical
        # landmark is used at most once per detection array.
        candidates = []
        for i, obs in enumerate(observations):
            for j, lm in enumerate(self.landmarks):
                delta = np.array([lm['x']-self.X[0], lm['y']-self.X[1]])
                expected = rotation(-self.X[2]) @ delta
                residual = obs - expected
                dist = float(np.linalg.norm(residual))
                if dist <= gate:
                    candidates.append((dist, i, j))
        candidates.sort()
        used_obs, used_land = set(), set()
        matches = []
        for dist, i, j in candidates:
            if i in used_obs or j in used_land:
                continue
            used_obs.add(i)
            used_land.add(j)
            matches.append((i, j))
        matched = 0
        for i, j in matches:
            landmark = self.landmarks[j]
            dx, dy = landmark['x']-self.X[0], landmark['y']-self.X[1]
            c, s = math.cos(self.X[2]), math.sin(self.X[2])
            expected = np.array([c*dx+s*dy, -s*dx+c*dy])
            H = np.array([[-c, -s, expected[1]], [s, -c, -expected[0]]])
            residual = observations[i] - expected
            S = H @ self.P @ H.T + Rnoise
            try:
                mahal = float(residual.T @ np.linalg.solve(S, residual))
                if mahal > chi2_gate:
                    continue
                K = np.linalg.solve(S, H @ self.P).T
            except np.linalg.LinAlgError:
                continue
            self.X = self.X + K @ residual
            self.X[2] = wrap(self.X[2])
            I_KH = np.eye(3) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ Rnoise @ K.T
            matched += 1
        if matched:
            self.get_logger().info(f'EKF corrected using {matched} cylinders', throttle_duration_sec=3.0)

    def publish_estimate(self):
        if self.current_odom is None:
            return
        stamp = self.get_clock().now().to_msg()
        # map->odom = (map->base) composed with inverse(odom->base)
        map_odom_yaw = wrap(self.X[2]-self.current_odom[2])
        map_odom_xy = self.X[:2] - rotation(map_odom_yaw) @ self.current_odom[:2]
        tf = TransformStamped()
        tf.header.frame_id = 'map'; tf.header.stamp = stamp; tf.child_frame_id = 'odom'
        tf.transform.translation.x = float(map_odom_xy[0]); tf.transform.translation.y = float(map_odom_xy[1])
        tf.transform.rotation.z = math.sin(map_odom_yaw/2)
        tf.transform.rotation.w = math.cos(map_odom_yaw/2)
        self.broadcaster.sendTransform(tf)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'; pose.header.stamp = stamp
        pose.pose.pose.position.x = float(self.X[0]); pose.pose.pose.position.y = float(self.X[1])
        pose.pose.pose.orientation.z = math.sin(self.X[2]/2)
        pose.pose.pose.orientation.w = math.cos(self.X[2]/2)
        for i, index in enumerate((0, 1, 5)):
            for j, jndex in enumerate((0, 1, 5)):
                pose.pose.covariance[index*6+jndex] = float(self.P[i, j])
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = LandmarkLocalizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
