#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CheckpointFollower(Node):
    def __init__(self):
        super().__init__('checkpoint_follower')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_tolerance_m', 0.20)
        self.declare_parameter('heading_tolerance_rad', 0.22)
        self.declare_parameter('forward_speed_mps', 0.16)
        self.declare_parameter('turn_speed_radps', 0.45)
        self.declare_parameter('control_period_s', 0.10)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        self.heading_tol = float(self.get_parameter('heading_tolerance_rad').value)
        self.forward_speed = float(self.get_parameter('forward_speed_mps').value)
        self.turn_speed = float(self.get_parameter('turn_speed_radps').value)
        period = float(self.get_parameter('control_period_s').value)

        self.points = []
        self.index = 0
        self.active = False
        self.loop_mode = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_cb, 10)
        self.create_subscription(String, '/checkpoint_control', self.control_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(period, self.control_loop)
        self.get_logger().info('Checkpoint follower ready. Click points in RViz, then publish /checkpoint_control: start/loop/stop/clear')

    def clicked_point_cb(self, msg: PointStamped):
        self.points.append((msg.point.x, msg.point.y))
        self.get_logger().info(f'Added checkpoint {len(self.points)}: x={msg.point.x:.3f}, y={msg.point.y:.3f}')

    def control_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'clear':
            self.points.clear()
            self.index = 0
            self.active = False
            self.stop()
            self.get_logger().info('Cleared checkpoints')
        elif cmd == 'stop':
            self.active = False
            self.stop()
            self.get_logger().info('Stopped')
        elif cmd == 'start':
            self.index = 0
            self.loop_mode = False
            self.active = len(self.points) > 0
            self.get_logger().info(f'Start once with {len(self.points)} checkpoints')
        elif cmd == 'loop':
            self.index = 0
            self.loop_mode = True
            self.active = len(self.points) > 0
            self.get_logger().info(f'Start loop with {len(self.points)} checkpoints')
        else:
            self.get_logger().warn('Unknown control. Use: start, loop, stop, clear')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def get_pose(self):
        tr = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        yaw = yaw_from_quat(tr.transform.rotation)
        return x, y, yaw

    def control_loop(self):
        if not self.active or not self.points:
            return
        try:
            x, y, yaw = self.get_pose()
        except Exception as e:
            self.get_logger().warn(f'No TF {self.map_frame}->{self.base_frame}: {e}', throttle_duration_sec=2.0)
            self.stop()
            return

        gx, gy = self.points[self.index]
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        err = norm_angle(target_yaw - yaw)

        if dist < self.goal_tol:
            self.get_logger().info(f'Reached checkpoint {self.index + 1}/{len(self.points)}')
            self.index += 1
            if self.index >= len(self.points):
                if self.loop_mode:
                    self.index = 0
                else:
                    self.active = False
                    self.stop()
                    self.get_logger().info('Finished all checkpoints')
                    return
            return

        cmd = Twist()
        if abs(err) > self.heading_tol:
            cmd.angular.z = self.turn_speed if err > 0.0 else -self.turn_speed
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = CheckpointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
