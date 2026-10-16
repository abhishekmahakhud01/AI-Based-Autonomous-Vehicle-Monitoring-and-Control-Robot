#!/usr/bin/env python3
"""Reactive TurtleBot3 obstacle avoidance for ROS 2 Humble / Gazebo Classic.

Runs indefinitely by default. Set ROBOT_MAX_DISTANCE to a positive number
only when a travel limit is desired; 0 disables the limit.
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_DISTANCE = max(0.0, float(os.environ.get('ROBOT_MAX_DISTANCE', '0')))
OBSTACLE_DISTANCE = 0.80       # begin turning inside this distance
CLEAR_DISTANCE = 0.95          # hysteresis: resume forward motion beyond this
FORWARD_SPEED = 0.08           # m/s; deliberately slow for simulation/demo
TURN_SPEED = 0.55              # rad/s
SCAN_TIMEOUT = 1.0
FRONT_HALF_ANGLE = 35.0


def sector_score(values):
    """Robust free-space score: 30th percentile, less sensitive to one ray."""
    if not values:
        return 0.0
    values = sorted(values)
    index = int(0.30 * (len(values) - 1))
    return values[index]


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.last_scan_time = None
        self.last_odom_position = None
        self.total_distance = 0.0
        self.limit_reached = False
        self.turn_direction = 0.0
        self.last_timeout_warning = 0.0
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)
        limit_text = 'unlimited' if MAX_DISTANCE == 0 else f'{MAX_DISTANCE:.1f} m'
        self.get_logger().info(f'Controller ready; travel limit: {limit_text}')

    def odom_callback(self, odom):
        p = odom.pose.pose.position
        position = (p.x, p.y)
        if self.last_odom_position is not None:
            old_x, old_y = self.last_odom_position
            step = math.hypot(p.x - old_x, p.y - old_y)
            # Ignore teleport/reset jumps; normal odometry increments are small.
            if math.isfinite(step) and step < 0.5:
                self.total_distance += step
        self.last_odom_position = position
        if MAX_DISTANCE > 0 and self.total_distance >= MAX_DISTANCE and not self.limit_reached:
            self.limit_reached = True
            self.stop_robot()
            self.get_logger().warning(f'Travel limit {MAX_DISTANCE:.1f} m reached; stopped.')

    def watchdog_callback(self):
        if self.limit_reached:
            self.stop_robot()
            return
        if self.last_scan_time is None:
            self.stop_robot()
            return
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > SCAN_TIMEOUT:
            self.stop_robot()
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_timeout_warning > 2.0:
                self.get_logger().warning('LiDAR scan timed out; robot stopped until scans resume.')
                self.last_timeout_warning = now

    def scan_callback(self, scan):
        self.last_scan_time = self.get_clock().now()
        if self.limit_reached:
            self.stop_robot()
            return
        if not scan.ranges or scan.angle_increment == 0.0:
            self.stop_robot()
            self.get_logger().warning('Empty/invalid LiDAR scan; stopped.', throttle_duration_sec=2.0)
            return

        front, left, right = [], [], []
        for i, raw in enumerate(scan.ranges):
            # +inf means no return before range_max, i.e. clear up to sensor range.
            if raw == math.inf:
                distance = scan.range_max
            elif math.isfinite(raw):
                distance = raw
            else:
                continue
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            # Normalize to [-pi, pi], matching ROS LaserScan's forward direction.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            degrees = math.degrees(angle)
            if abs(degrees) <= FRONT_HALF_ANGLE:
                front.append(distance)
            elif FRONT_HALF_ANGLE < degrees < 145.0:
                left.append(distance)
            elif -145.0 < degrees < -FRONT_HALF_ANGLE:
                right.append(distance)

        if not front:
            self.stop_robot()
            self.get_logger().warning('No valid forward LiDAR readings; stopped.', throttle_duration_sec=2.0)
            return

        nearest_front = min(front)
        cmd = Twist()
        if nearest_front < OBSTACLE_DISTANCE or self.turn_direction != 0.0 and nearest_front < CLEAR_DISTANCE:
            # Re-evaluate free space each scan so the robot can change direction
            # if the chosen side becomes blocked; do not drive forward while turning.
            left_score = sector_score(left)
            right_score = sector_score(right)
            if left_score == right_score == 0.0:
                self.stop_robot()
                self.get_logger().warning('No usable side LiDAR readings; stopped.', throttle_duration_sec=2.0)
                return
            self.turn_direction = 1.0 if left_score >= right_score else -1.0
            cmd.angular.z = self.turn_direction * TURN_SPEED
        else:
            self.turn_direction = 0.0
            cmd.linear.x = FORWARD_SPEED

        self.pub.publish(cmd)
        self.get_logger().info(
            f'Distance: {self.total_distance:.2f} m | front: {nearest_front:.2f} m | '
            f'left score: {sector_score(left):.2f} | right score: {sector_score(right):.2f} | '
            f'linear: {cmd.linear.x:.2f} | angular: {cmd.angular.z:.2f}',
            throttle_duration_sec=1.0)

    def stop_robot(self):
        self.pub.publish(Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
