#!/usr/bin/env python3
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

CAN_ID_EMERGENCY = 0x001
CAN_ID_CONTROL   = 0x120  # Raspberry Pi -> ESP32 gateway command
CAN_ID_STATUS    = 0x222
CAN_ID_ANGLE     = 0x050
CAN_ID_ODOM      = 0x223  # optional future S32K frame

CMD_STOP       = 0x00
CMD_FORWARD    = 0x01
CMD_BACKWARD   = 0x02
CMD_TURN_LEFT  = 0x03
CMD_TURN_RIGHT = 0x04


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CanBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('cmd_period_s', 0.10)
        self.declare_parameter('cmd_timeout_s', 0.60)
        self.declare_parameter('forward_pwm', 135)
        self.declare_parameter('turn_pwm', 120)
        self.declare_parameter('backward_pwm', 115)
        self.declare_parameter('linear_deadband', 0.04)
        self.declare_parameter('angular_deadband', 0.20)
        self.declare_parameter('wheel_base_m', 0.285)
        self.declare_parameter('feedback_unit_to_mps', 0.0010)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.iface = self.get_parameter('interface').value
        self.cmd_period_s = float(self.get_parameter('cmd_period_s').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.forward_pwm = int(self.get_parameter('forward_pwm').value)
        self.turn_pwm = int(self.get_parameter('turn_pwm').value)
        self.backward_pwm = int(self.get_parameter('backward_pwm').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.feedback_unit_to_mps = float(self.get_parameter('feedback_unit_to_mps').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.iface,))
        self.sock.settimeout(0.05)

        self.last_twist = Twist()
        self.last_twist_time = 0.0
        self.last_cmd = CMD_STOP
        self.last_status_time = None
        self.feedback_l = 0
        self.feedback_r = 0
        self.emergency = 0
        self.esp_yaw_cdeg = 0
        self.esp_gz_cdps = 0
        self.esp_angle_valid = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.cmd_timer = self.create_timer(self.cmd_period_s, self.send_cmd_timer)
        self.get_logger().info(f'CAN bridge started on {self.iface}. Sending ESP32 gateway command ID 0x120, receiving S32K status 0x222.')

    def destroy_node(self):
        self.running = False
        try:
            self.send_motor(CMD_STOP, 0, 0)
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()

    def cmd_vel_cb(self, msg: Twist):
        self.last_twist = msg
        self.last_twist_time = time.monotonic()

    def pack_frame(self, can_id, data: bytes):
        data = data[:8]
        data = data + bytes(8 - len(data))
        return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(data.rstrip(b'\x00')) if False else min(len(data), 8), data)

    def send_raw(self, can_id, payload):
        payload = bytes(payload[:8])
        dlc = len(payload)
        payload = payload + bytes(8 - dlc)
        frame = struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, dlc, payload)
        self.sock.send(frame)

    def send_motor(self, cmd, spd_l, spd_r):
        spd_l = int(clamp(spd_l, 0, 255))
        spd_r = int(clamp(spd_r, 0, 255))
        payload = [cmd & 0xFF, (spd_l >> 8) & 0xFF, spd_l & 0xFF, (spd_r >> 8) & 0xFF, spd_r & 0xFF]
        try:
            self.send_raw(CAN_ID_CONTROL, payload)
            self.last_cmd = cmd
        except OSError as e:
            self.get_logger().warn(f'CAN TX failed: {e}')

    def twist_to_motor_cmd(self):
        now = time.monotonic()
        if self.last_twist_time == 0.0 or (now - self.last_twist_time) > self.cmd_timeout_s:
            return CMD_STOP, 0, 0

        v = float(self.last_twist.linear.x)
        w = float(self.last_twist.angular.z)
        if abs(v) < self.linear_deadband and abs(w) < self.angular_deadband:
            return CMD_STOP, 0, 0
        if abs(w) >= self.angular_deadband and abs(v) < 0.10:
            if w > 0.0:
                return CMD_TURN_LEFT, self.turn_pwm, self.turn_pwm
            return CMD_TURN_RIGHT, self.turn_pwm, self.turn_pwm
        if v > self.linear_deadband:
            return CMD_FORWARD, self.forward_pwm, self.forward_pwm
        if v < -self.linear_deadband:
            return CMD_BACKWARD, self.backward_pwm, self.backward_pwm
        return CMD_STOP, 0, 0

    def send_cmd_timer(self):
        cmd, l, r = self.twist_to_motor_cmd()
        self.send_motor(cmd, l, r)

    def rx_loop(self):
        while self.running:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(frame) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            can_id &= CAN_SFF_MASK
            data = data[:dlc]
            if can_id == CAN_ID_STATUS and dlc >= 5:
                self.feedback_l = (data[0] << 8) | data[1]
                self.feedback_r = (data[2] << 8) | data[3]
                self.emergency = data[4]
                self.handle_status_feedback()
            elif can_id == CAN_ID_ANGLE and dlc >= 6:
                self.esp_angle_valid = data[1] & 0x01
                self.esp_yaw_cdeg = struct.unpack('>h', data[2:4])[0]
                self.esp_gz_cdps = struct.unpack('>h', data[4:6])[0]
            elif can_id == CAN_ID_ODOM and dlc >= 8:
                # Optional future frame: int16 dL, int16 dR, int16 yaw_cdeg, flags, seq
                dl = struct.unpack('>h', data[0:2])[0]
                dr = struct.unpack('>h', data[2:4])[0]
                yaw_cdeg = struct.unpack('>h', data[4:6])[0]
                self.handle_odom_delta(dl, dr, yaw_cdeg)

    def handle_status_feedback(self):
        now = time.monotonic()
        if self.last_status_time is None:
            self.last_status_time = now
            return
        dt = now - self.last_status_time
        self.last_status_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        sign = 0.0
        if self.last_cmd == CMD_FORWARD:
            sign = 1.0
        elif self.last_cmd == CMD_BACKWARD:
            sign = -1.0
        elif self.last_cmd in (CMD_TURN_LEFT, CMD_TURN_RIGHT):
            sign = 1.0
        else:
            sign = 0.0

        vl = sign * float(self.feedback_l) * self.feedback_unit_to_mps
        vr = sign * float(self.feedback_r) * self.feedback_unit_to_mps
        v = 0.5 * (vl + vr)
        w = (vr - vl) / max(self.wheel_base_m, 0.001)
        self.integrate_and_publish(v, w, dt)

    def handle_odom_delta(self, dl, dr, yaw_cdeg):
        # Placeholder: tune pulses_to_meter before using this frame.
        # Kept for future S32K odom extension.
        pass

    def integrate_and_publish(self, v, w, dt):
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.th)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        msg = String()
        msg.data = f'fb_l={self.feedback_l} fb_r={self.feedback_r} emg={self.emergency} x={self.x:.3f} y={self.y:.3f} th={self.th:.3f}'
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
