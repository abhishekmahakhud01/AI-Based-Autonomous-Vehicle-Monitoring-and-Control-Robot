#!/usr/bin/env python3
"""Record TurtleBot3 ROS 2 data and generate six analysis plots on Ctrl+C.

Tested design target: ROS 2 Humble, Python 3, sensor_msgs/LaserScan,
nav_msgs/Odometry, geometry_msgs/Twist, and matplotlib.
"""
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SAMPLE_PERIOD = 0.2  # 5 samples/second
FRONT_HALF_ANGLE_DEG = 35.0


def valid_range(value, scan):
    if math.isinf(value) and value > 0:
        return float(scan.range_max)
    if not math.isfinite(value):
        return None
    if value < scan.range_min or value > scan.range_max:
        return None
    return float(value)


def sector_min(scan, lower_deg, upper_deg):
    values = []
    if not scan.ranges or scan.angle_increment == 0.0:
        return None
    for index, raw in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        angle = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
        if lower_deg <= angle <= upper_deg:
            value = valid_range(raw, scan)
            if value is not None:
                values.append(value)
    return min(values) if values else None


class RobotRecorder(Node):
    def __init__(self, output_dir):
        super().__init__('turtlebot3_data_recorder')
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'robot_data.csv'
        self.start_time = time.monotonic()
        self.latest_scan = None
        self.latest_scan_time = None
        self.pose_x = None
        self.pose_y = None
        self.odom_linear = None
        self.odom_angular = None
        self.cmd_linear = None
        self.cmd_angular = None
        self.rows = []
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 20)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 20)
        self.create_timer(SAMPLE_PERIOD, self.sample_callback)
        self.get_logger().info('Recording /scan, /odom and /cmd_vel. Press Ctrl+C to save CSV and six graphs.')
        self.get_logger().info(f'Output folder: {self.output_dir}')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose_x, self.pose_y = float(p.x), float(p.y)
        self.odom_linear = float(msg.twist.twist.linear.x)
        self.odom_angular = float(msg.twist.twist.angular.z)

    def cmd_callback(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    def sample_callback(self):
        elapsed = time.monotonic() - self.start_time
        scan = self.latest_scan
        scan_fresh = (self.latest_scan_time is not None and
                      time.monotonic() - self.latest_scan_time <= 1.0)
        front = left = right = None
        if scan is not None and scan_fresh:
            front = sector_min(scan, -FRONT_HALF_ANGLE_DEG, FRONT_HALF_ANGLE_DEG)
            left = sector_min(scan, FRONT_HALF_ANGLE_DEG, 145.0)
            right = sector_min(scan, -145.0, -FRONT_HALF_ANGLE_DEG)
        self.rows.append({
            'time_s': elapsed,
            'front_distance_m': front,
            'left_distance_m': left,
            'right_distance_m': right,
            'x_m': self.pose_x,
            'y_m': self.pose_y,
            'odom_linear_mps': self.odom_linear,
            'odom_angular_rps': self.odom_angular,
            'cmd_linear_mps': self.cmd_linear,
            'cmd_angular_rps': self.cmd_angular,
        })
        if len(self.rows) % 25 == 0:
            self.get_logger().info(f'Recorded {len(self.rows)} samples ({elapsed:.1f} s)')

    def save_and_plot(self):
        if not self.rows:
            print('No samples recorded. Check that ROS topics are available and try again.')
            return
        columns = list(self.rows[0].keys())
        with self.csv_path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            writer.writerows(self.rows)
        print(f'CSV saved: {self.csv_path}')
        generate_plots(self.rows, self.output_dir)


def series(rows, key):
    values = []
    for row in rows:
        value = row.get(key)
        values.append(float('nan') if value is None else float(value))
    return values


def save_figure(fig, output_dir, filename):
    fig.tight_layout()
    path = output_dir / filename
    fig.savefig(path, dpi=160, bbox_inches='tight')
    plt.close(fig)
    print(f'Graph saved: {path}')


def generate_plots(rows, output_dir):
    t = series(rows, 'time_s')

    # 1. Obstacle distances versus time.
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, series(rows, 'front_distance_m'), label='Front')
    ax.plot(t, series(rows, 'left_distance_m'), label='Left')
    ax.plot(t, series(rows, 'right_distance_m'), label='Right')
    ax.set(title='LiDAR Obstacle Distance vs Time', xlabel='Time (s)', ylabel='Nearest valid sector distance (m)')
    ax.grid(True, alpha=0.3); ax.legend()
    save_figure(fig, output_dir, '01_obstacle_distance_vs_time.png')

    # 2. Linear velocity versus time (commanded and odometry measured).
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, series(rows, 'cmd_linear_mps'), label='Commanded linear velocity')
    ax.plot(t, series(rows, 'odom_linear_mps'), label='Odometry linear velocity')
    ax.set(title='Linear Velocity vs Time', xlabel='Time (s)', ylabel='Linear velocity (m/s)')
    ax.grid(True, alpha=0.3); ax.legend()
    save_figure(fig, output_dir, '02_linear_velocity_vs_time.png')

    # 3. Angular velocity versus time.
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, series(rows, 'cmd_angular_rps'), label='Commanded angular velocity')
    ax.plot(t, series(rows, 'odom_angular_rps'), label='Odometry angular velocity')
    ax.set(title='Angular Velocity vs Time', xlabel='Time (s)', ylabel='Angular velocity (rad/s)')
    ax.grid(True, alpha=0.3); ax.legend()
    save_figure(fig, output_dir, '03_angular_velocity_vs_time.png')

    # 4. Actual XY trajectory from odometry.
    x, y = series(rows, 'x_m'), series(rows, 'y_m')
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(x, y, label='Robot path')
    valid_xy = [(a, b) for a, b in zip(x, y) if math.isfinite(a) and math.isfinite(b)]
    if valid_xy:
        ax.scatter([valid_xy[0][0]], [valid_xy[0][1]], marker='o', label='Start')
        ax.scatter([valid_xy[-1][0]], [valid_xy[-1][1]], marker='x', label='End')
    ax.set(title='Robot Trajectory (X vs Y)', xlabel='X position (m)', ylabel='Y position (m)')
    ax.axis('equal'); ax.grid(True, alpha=0.3); ax.legend()
    save_figure(fig, output_dir, '04_robot_xy_trajectory.png')

    # 5. X and Y position versus time.
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, x, label='X position')
    ax.plot(t, y, label='Y position')
    ax.set(title='Robot Position vs Time', xlabel='Time (s)', ylabel='Position (m)')
    ax.grid(True, alpha=0.3); ax.legend()
    save_figure(fig, output_dir, '05_position_vs_time.png')

    # 6. Front clearance versus commanded linear speed.
    front = series(rows, 'front_distance_m')
    speed = series(rows, 'cmd_linear_mps')
    pairs = [(d, v) for d, v in zip(front, speed) if math.isfinite(d) and math.isfinite(v)]
    fig, ax = plt.subplots(figsize=(8, 5.5))
    if pairs:
        ax.scatter([p[0] for p in pairs], [p[1] for p in pairs], alpha=0.65)
    else:
        ax.text(0.5, 0.5, 'No valid front-distance and speed samples', ha='center', va='center', transform=ax.transAxes)
    ax.set(title='Front Obstacle Clearance vs Commanded Speed', xlabel='Front LiDAR distance (m)', ylabel='Commanded linear velocity (m/s)')
    ax.grid(True, alpha=0.3)
    save_figure(fig, output_dir, '06_front_clearance_vs_speed.png')

    print('\nFinished. Six graphs and robot_data.csv are in:')
    print(output_dir)


def main():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = Path(__file__).resolve().parent / 'data' / f'run_{stamp}'
    rclpy.init()
    node = RobotRecorder(base)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopping recorder and generating graphs...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        node.save_and_plot()


if __name__ == '__main__':
    main()
