import can
import struct
import math
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# -------------------------------------------------------
# THÔNG SỐ
# -------------------------------------------------------
SPEED_MIN       = 1000
SPEED_MAX       = 2000
DIST_EMERGENCY  = 0.2   # Dưới 0.2m → emergency CAN 0x001
DIST_STOP       = 0.4   # Dưới 0.4m → stop
DIST_SLOW       = 0.8   # Dưới 0.8m → chậm + quẹo
DIST_FAST       = 2.0   # Trên 2.0m → full speed

CONFIRM_COUNT   = 3     # Số lần liên tiếp cùng lệnh mới gửi
SPEED_DEADBAND  = 50    # Thay đổi speed < 50 thì không gửi lại
SEND_INTERVAL   = 0.3   # Tối thiểu 0.3s giữa 2 lần gửi

# Wheelbase robot (m) — chỉnh theo robot thực tế
WHEEL_BASE      = 0.5
WHEEL_RADIUS    = 0.1

# -------------------------------------------------------
# CAN
# -------------------------------------------------------
bus = can.interface.Bus(channel='can0', interface='socketcan')

def send_control(cmd, speed_L, speed_R):
    data = struct.pack('>BHH', cmd, speed_L, speed_R)
    msg = can.Message(arbitration_id=0x111, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"[TX CTRL] CMD:{cmd} | L:{speed_L} | R:{speed_R}")

def send_emergency(active: bool):
    data = [0xFF if active else 0x00]
    msg = can.Message(arbitration_id=0x001, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"[TX EMG] {'LOCK 0xFF' if active else 'RELEASE 0x00'}")

def calc_speed(distance):
    if distance <= DIST_SLOW:
        return SPEED_MIN
    elif distance >= DIST_FAST:
        return SPEED_MAX
    else:
        ratio = (distance - DIST_SLOW) / (DIST_FAST - DIST_SLOW)
        return int(SPEED_MIN + ratio * (SPEED_MAX - SPEED_MIN))

# -------------------------------------------------------
# THREAD NHẬN FEEDBACK TỪ S32K144
# -------------------------------------------------------
feedback = {'speed_L': 0, 'speed_R': 0, 'emg': 0}
feedback_lock = threading.Lock()

def feedback_thread():
    print("[FEEDBACK] Thread started")
    while True:
        try:
            msg = bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x222 and len(msg.data) >= 5:
                with feedback_lock:
                    feedback['speed_L'] = (msg.data[0] << 8) | msg.data[1]
                    feedback['speed_R'] = (msg.data[2] << 8) | msg.data[3]
                    feedback['emg']     = msg.data[4]
        except Exception:
            pass

# -------------------------------------------------------
# ROS2 NODE
# -------------------------------------------------------
class LidarCanBridge(Node):
    def __init__(self):
        super().__init__('lidar_can_bridge')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.last_cmd        = -1
        self.last_speed      = 0
        self.pending_cmd     = -1
        self.pending_count   = 0
        self.last_send_time  = 0.0
        self.emergency_state = False
        self.get_logger().info('LidarCanBridge started')

    def get_sector_min(self, ranges, angle_min_deg, angle_max_deg, angle_increment):
        inc_deg   = math.degrees(angle_increment)
        base      = math.degrees(-3.14159)
        start_idx = max(0, int((angle_min_deg - base) / inc_deg))
        end_idx   = min(len(ranges) - 1, int((angle_max_deg - base) / inc_deg))
        sector    = [r for r in ranges[start_idx:end_idx] if 0.1 < r < 12.0]
        return min(sector) if sector else 12.0

    def scan_callback(self, msg):
        inc = msg.angle_increment

        # 6 sector
        front_min      = self.get_sector_min(msg.ranges, -30,   30,  inc)
        front_left_min = self.get_sector_min(msg.ranges,  30,   60,  inc)
        front_right_min= self.get_sector_min(msg.ranges, -60,  -30,  inc)
        left_min       = self.get_sector_min(msg.ranges,  60,  120,  inc)
        right_min      = self.get_sector_min(msg.ranges, -120, -60,  inc)
        back_min       = self.get_sector_min(msg.ranges,  150, -150, inc)

        # Feedback từ S32K144
        with feedback_lock:
            fb_L = feedback['speed_L']
            fb_R = feedback['speed_R']
            fb_emg = feedback['emg']

        self.get_logger().info(
            f"F:{front_min:.2f} FL:{front_left_min:.2f} FR:{front_right_min:.2f} "
            f"L:{left_min:.2f} R:{right_min:.2f} B:{back_min:.2f} | "
            f"FB L:{fb_L} R:{fb_R} EMG:{fb_emg}"
        )

        # --- EMERGENCY: vật cản cực gần ---
        if front_min < DIST_EMERGENCY:
            if not self.emergency_state:
                self.get_logger().error(f"!!! EMERGENCY - front:{front_min:.2f}m !!!")
                send_emergency(True)
                self.emergency_state = True
                self.last_cmd = -1
            return
        else:
            if self.emergency_state:
                self.get_logger().warn("EMERGENCY RELEASED")
                send_emergency(False)
                self.emergency_state = False

        # --- Quyết định lệnh ---
        if front_min < DIST_STOP:
            new_cmd = 5

        elif front_min < DIST_SLOW:
            # Chọn hướng trống nhất
            left_score  = min(front_left_min, left_min)
            right_score = min(front_right_min, right_min)
            new_cmd = 3 if left_score > right_score else 4

        else:
            new_cmd = 1

        # --- Debounce ---
        if new_cmd == self.pending_cmd:
            self.pending_count += 1
        else:
            self.pending_cmd   = new_cmd
            self.pending_count = 1

        if self.pending_count < CONFIRM_COUNT:
            return

        # --- Tính speed ---
        speed = calc_speed(front_min)

        # --- Kiểm tra có cần gửi không ---
        now           = time.time()
        cmd_changed   = (new_cmd != self.last_cmd)
        speed_changed = (abs(speed - self.last_speed) > SPEED_DEADBAND)
        time_ok       = (now - self.last_send_time >= SEND_INTERVAL)

        if not (cmd_changed or (speed_changed and time_ok)):
            return

        # --- Gửi lệnh ---
        if new_cmd == 5:
            self.get_logger().warn(f"STOP - front:{front_min:.2f}m")
            send_control(5, 0, 0)

        elif new_cmd == 3:
            self.get_logger().warn(
                f"TURN LEFT - front:{front_min:.2f}m "
                f"FL:{front_left_min:.2f} L:{left_min:.2f} speed:{speed}"
            )
            send_control(3, speed // 2, speed)

        elif new_cmd == 4:
            self.get_logger().warn(
                f"TURN RIGHT - front:{front_min:.2f}m "
                f"FR:{front_right_min:.2f} R:{right_min:.2f} speed:{speed}"
            )
            send_control(4, speed, speed // 2)

        elif new_cmd == 1:
            self.get_logger().info(f"FORWARD - front:{front_min:.2f}m speed:{speed}")
            send_control(1, speed, speed)

        self.last_cmd       = new_cmd
        self.last_speed     = speed
        self.last_send_time = now


def main():
    # Start feedback thread
    t = threading.Thread(target=feedback_thread, daemon=True)
    t.start()

    rclpy.init()
    node = LidarCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[STOP]")
        send_control(5, 0, 0)
        send_emergency(False)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        bus.shutdown()
        print("[DONE]")

if __name__ == '__main__':
    main()
