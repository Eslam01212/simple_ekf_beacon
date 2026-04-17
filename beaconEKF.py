#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class BeaconEKF(Node):
    def __init__(self):
        super().__init__('beacon_ekf')

        # 4 cylinders at corners of a 3 m square, centered at origin
        self.beacons = np.array([
            [-1.8/2, 1.5],
            [ 1.8/2, 1.5],
            [ 1.8/2, -.4],
            [-1.8/2, -.4],
        ], dtype=float)

        # cylinder / scan settings
        self.cyl_radius = 0.1
        self.cluster_gap = 0.15
        self.min_cluster_pts = 3
        self.max_scan_range = 3.5

        # added odom noise for robustness test
        self.noise_d = 0.3
        self.noise_yaw = math.radians(3.0)
        
        self.odom_shift_x = 0.0
        self.odom_shift_y = -0.0

        # EKF state [x, y, yaw]
        self.x = np.array([0.0, 0.0, 0.0], dtype=float)
        self.noisy = np.array([0.0, 0.0, 0.0], dtype=float)

        self.P = np.diag([0.05, 0.05, math.radians(5.0)])**2
        self.Q = np.diag([0.01, 0.01, math.radians(0.8)]) ** 2   
        self.Q = np.diag([0.03, 0.03, math.radians(2.0)]) ** 2   

        self.R = np.diag([0.30, math.radians(12.0)])**2        

        self.prev_odom = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #self.create_timer(0.1, self.move_square)
        self.create_timer(0.1, self.move_circle)
        self.create_timer(0.5, self.print_pose)

        self.get_logger().info('Beacon EKF started.')

    def safe_stop(self):
        if not hasattr(self, 'scan') or self.scan is None or len(self.scan) == 0:
            return False

        if np.any(np.asarray(self.scan, dtype=np.float32) < 0.20):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.destroy_node()
            rclpy.shutdown()
            raise SystemExit

        return False

    def move_square(self):
        if self.safe_stop():
            return
        if not hasattr(self, 'x'):
            return

        msg = Twist()

        if not hasattr(self, 'mode'):
            self.mode = 'go'
            self.x0, self.y0, self.yaw0 = self.x[0], self.x[1], self.x[2]

        if self.mode == 'go':
            if math.hypot(self.x[0] - self.x0, self.x[1] - self.y0) < 1:
                msg.linear.x = 0.1
            else:
                self.mode = 'turn'
                self.yaw0 = self.x[2]
        else:
            if abs(wrap(self.x[2] - self.yaw0)) < math.pi / 2:
                msg.angular.z = 0.2
            else:
                self.mode = 'go'
                self.x0, self.y0 = self.x[0], self.x[1]
        self.cmd_pub.publish(msg)

    def move_circle(self):
        if self.safe_stop():
            return
        if not hasattr(self, 'cmd_pub'):
            return
    
        msg = Twist()
        msg.linear.x = 0.08      # forward speed
        msg.angular.z = 0.5     # turning speed
        self.cmd_pub.publish(msg)
    
    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y 
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.prev_odom is None:
            self.prev_odom = (px, py, yaw)
            self.odom_x, self.odom_y, self.odom_yaw = px + self.odom_shift_x, py + self.odom_shift_y, yaw
            self.x[:] = [self.odom_x, self.odom_y, yaw]
            self.noisy[:] = [self.odom_x, self.odom_y, yaw]
            return

        ppx, ppy, pyaw = self.prev_odom
        dx = px - ppx
        dy = py - ppy
        dyaw = wrap(yaw - pyaw)

        d = math.hypot(dx, dy)
        if msg.twist.twist.linear.x < 0.0:
            d = -d

        # add fake odom noise
        d += np.random.normal(0.0, self.noise_d)
        dyaw += np.random.normal(0.0, self.noise_yaw)

        # noisy odom-only pose
        thn = self.noisy[2] + 0.5 * dyaw
        self.noisy[0] += d * math.cos(thn)
        self.noisy[1] += d * math.sin(thn)
        self.noisy[2] = wrap(self.noisy[2] + dyaw)

        # EKF prediction uses the same noisy odom
        th_mid = self.x[2] + 0.5 * dyaw
        self.x[0] += d * math.cos(th_mid)
        self.x[1] += d * math.sin(th_mid)
        self.x[2] = wrap(self.x[2] + dyaw)

        F = np.array([
            [1.0, 0.0, -d * math.sin(th_mid)],
            [0.0, 1.0,  d * math.cos(th_mid)],
            [0.0, 0.0,  1.0]
        ])
        self.P = F @ self.P @ F.T + self.Q

        self.prev_odom = (px, py, yaw)
        self.odom_x, self.odom_y, self.odom_yaw = px + self.odom_shift_x, py + self.odom_shift_y, yaw

    def scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
    
        # align scan yaw
        """"LIDAR_YAW_OFF = np.pi / 2 
        shift = int(round(LIDAR_YAW_OFF / float(msg.angle_increment)))
        r = np.roll(r, shift)"""

        
        # clamp / sanitize
        eff_max = min(float(msg.range_max), 3.5)
        r = np.nan_to_num(r, nan=eff_max, posinf=eff_max, neginf=eff_max)
        r = np.clip(r, float(msg.range_min), eff_max)
    
        # spatial median filter
        """if r.size >= 5:
            r = np.median(
                np.stack([np.roll(r, k) for k in (-2, -1, 0, 1, 2)], axis=0),
                axis=0
            )
    
        # temporal EMA
        prev = getattr(self, "scan", None)
        if isinstance(prev, np.ndarray) and prev.shape == r.shape:
            alpha = 0.30
            r = alpha * r + (1.0 - alpha) * prev
    
        # rear 90 deg -> no obstacle
        n = r.size
        rear_start = 3 * n // 8
        rear_end   = 5 * n // 8
        r[rear_start:rear_end] = eff_max"""
    
        if r.size >= 3:
          r = np.median(np.stack([np.roll(r, -1), r, np.roll(r, 1)]), axis=0)
        self.scan = r
        self.plot_scan_polar()
        msg.ranges = r.tolist()
    
        detections = self.extract_cylinders(msg)
    
        used = set()
        for r_meas, b_meas in detections:
            best_j = None
            best_score = 1e9
    
            for j, (bx, by) in enumerate(self.beacons):
                if j in used:
                    continue
    
                dx = bx - self.x[0]
                dy = by - self.x[1]
                r_pred = math.hypot(dx, dy)
                if r_pred < 1e-6:
                    continue
    
                b_pred = wrap(math.atan2(dy, dx) - self.x[2])
                score = abs(r_meas - r_pred) + 0.7 * abs(wrap(b_meas - b_pred))
    
                if score < best_score:
                    best_score = score
                    best_j = j
    
            if best_j is not None and best_score < 0.8:
                used.add(best_j)
                self.ekf_update(best_j, r_meas, b_meas)
    
        self.plot_lidar_world(msg)
        
    def plot_scan_polar(self):
        if not hasattr(self, 'scan'):
            return
        if not hasattr(self, 'pax'):
            plt.ion()
            self.pfig, self.pax = plt.subplots(subplot_kw={'projection': 'polar'})
        a = np.linspace(-np.pi, np.pi, len(self.scan), endpoint=False)
        self.pax.clear()
        self.pax.plot(a, self.scan, '.')
        self.pax.set_title('Filtered LiDAR Polar')
        self.pfig.canvas.draw()
        self.pfig.canvas.flush_events()
        
    def extract_cylinders(self, msg: LaserScan):
        clusters = []
        current = []

        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment

            valid = (
                np.isfinite(r) and
                msg.range_min < r < min(msg.range_max, self.max_scan_range)
            )

            if not valid:
                if len(current) >= self.min_cluster_pts:
                    clusters.append(current)
                current = []
                continue

            if not current:
                current = [(r, ang)]
            else:
                r_prev, a_prev = current[-1]
                p1 = np.array([r_prev * math.cos(a_prev), r_prev * math.sin(a_prev)])
                p2 = np.array([r * math.cos(ang), r * math.sin(ang)])

                if np.linalg.norm(p2 - p1) < self.cluster_gap:
                    current.append((r, ang))
                else:
                    if len(current) >= self.min_cluster_pts:
                        clusters.append(current)
                    current = [(r, ang)]

        if len(current) >= self.min_cluster_pts:
            clusters.append(current)

        detections = []
        for c in clusters:
            r1, a1 = c[0]
            r2, a2 = c[-1]

            p1 = np.array([r1 * math.cos(a1), r1 * math.sin(a1)])
            p2 = np.array([r2 * math.cos(a2), r2 * math.sin(a2)])
            width = np.linalg.norm(p2 - p1)

            if not (0.03 <= width <= 0.40):
                continue

            a_mid = wrap(0.5 * (a1 + a2))
            r_mean = float(np.mean([rr for rr, _ in c]))
            r_center = r_mean + self.cyl_radius
            detections.append((r_center, a_mid))

        return detections

    def ekf_update(self, beacon_idx, r_meas, b_meas):
        bx, by = self.beacons[beacon_idx]
    
        dx = bx - self.x[0]
        dy = by - self.x[1]
        q = dx * dx + dy * dy
        if q < 1e-8:
            return
    
        r_pred = math.sqrt(q)
        b_pred = wrap(math.atan2(dy, dx) - self.x[2])
    
        z = np.array([r_meas, b_meas], dtype=float)
        h = np.array([r_pred, b_pred], dtype=float)
        y = z - h
        y[1] = wrap(y[1])
    
        H = np.array([
            [-dx / r_pred, -dy / r_pred, 0.0],
            [ dy / q,      -dx / q,     -1.0]
        ], dtype=float)
    
        S = H @ self.P @ H.T + self.R
        Sinv = np.linalg.inv(S)
    
        # reject bad beacon corrections
        maha = float(y.T @ Sinv @ y)
        if maha > 9.21:   # 2D innovation gate
            return
    
        K = self.P @ H.T @ Sinv
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
    
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R @ K.T

    def plot_covariance(self, nstd=2.0):
        Pxy = self.P[:2, :2]   # covariance of x,y only

        vals, vecs = np.linalg.eigh(Pxy)
        order = vals.argsort()[::-1]
        vals = vals[order]
        vecs = vecs[:, order]

        t = np.linspace(0, 2*np.pi, 100)
        ellipse = np.array([
            nstd * np.sqrt(vals[0]) * np.cos(t),
            nstd * np.sqrt(vals[1]) * np.sin(t)
        ])

        ellipse = vecs @ ellipse
        ex = self.x[0] + ellipse[0]
        ey = self.x[1] + ellipse[1]

        self.ax.plot(ex, ey, 'g--', linewidth=2, label=f'EKF {nstd}σ cov')


    def plot_lidar_world(self, msg):
        if not hasattr(self, 'odom_x'):
            return
        if not hasattr(self, 'ax'):
            plt.ion()
            self.fig, self.ax = plt.subplots()
    
        a = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment + self.x[2]
 
        r = np.array(msg.ranges, dtype=float)
        m = np.isfinite(r)

        xs = self.x[0] + r[m] * np.cos(a[m])
        ys = self.x[1] + r[m] * np.sin(a[m])

        self.ax.clear()
        self.ax.plot(xs, ys, '.', markersize=1)

        # beacons
        self.ax.plot(self.beacons[:, 0], self.beacons[:, 1], 'yo', markersize=16)

        # Ideal odom arrow
        """self.ax.arrow(
            self.odom_x, self.odom_y,
            0.2 * math.cos(self.odom_yaw),
            0.2 * math.sin(self.odom_yaw),
            color='black', width=0.02
        )"""

        # Noisy odom arrow
        self.ax.arrow(
            self.noisy[0], self.noisy[1],
            0.2 * math.cos(self.noisy[2]),
            0.2 * math.sin(self.noisy[2]),
            color='red', width=0.03
        )

        # EKF pose arrow
        self.ax.arrow(
            self.x[0], self.x[1],
            0.2 * math.cos(self.x[2]),
            0.2 * math.sin(self.x[2]),
            color='green', width=0.04
        )

        self.plot_covariance()

        # legend
        #self.ax.plot([], [], color='black', linewidth=2, label='Ideal Odom')
        self.ax.plot([], [], color='red',   linewidth=2, label='Noisy Odom')
        self.ax.plot([], [], color='green', linewidth=2, label='EKF')
        self.ax.legend(loc='lower right')

        txt = (
            #f"Ideal Odom: x={self.odom_x:.2f} y={self.odom_y:.2f} yaw={math.degrees(self.odom_yaw):.1f}\n"
            f"Noisy Odom: x={self.noisy[0]:.2f} y={self.noisy[1]:.2f} yaw={math.degrees(self.noisy[2]):.1f}\n"
            f"EKF (Lidar+Odom): x={self.x[0]:.2f} y={self.x[1]:.2f} yaw={math.degrees(self.x[2]):.1f}"
        )

        self.ax.text(
            -3.8, 3.1, txt, fontsize=10,
            bbox=dict(facecolor='white', alpha=0.8)
        )

        self.ax.set_title('LiDAR and Robot Pose')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-4, 4)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def print_pose(self):
        if not hasattr(self, 'odom_x'):
            return

        print(
            #f"Ideal Odom: x={self.odom_x:.3f} y={self.odom_y:.3f} yaw={math.degrees(self.odom_yaw):.1f} | "
            f"Noisy Odom: x={self.noisy[0]:.3f} y={self.noisy[1]:.3f} yaw={math.degrees(self.noisy[2]):.1f} | "
            f"EKF: x={self.x[0]:.3f} y={self.x[1]:.3f} yaw={math.degrees(self.x[2]):.1f}",
            flush=True
        )


def main():
    rclpy.init()
    node = BeaconEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    for _ in range(5):
        self.cmd_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)

if __name__ == '__main__':
    main()
