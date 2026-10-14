#!/usr/bin/env python3
# =====================================================================================
#  [THÊM] teleop_keyboard.py - Điều khiển robot 2 bánh vi sai bằng bàn phím.
#
#  VÌ SAO PHẢI CÓ FILE NÀY?
#      ros2_control chỉ *nhận* lệnh rồi đẩy xuống phần cứng; nó KHÔNG tự sinh lệnh.
#      Trước đây workspace không có node nào phát lệnh => robot đứng yên mãi.
#
#  VÌ SAO KHÔNG DÙNG teleop_twist_keyboard CHO NHANH?
#      Vì package đó phát geometry_msgs/msg/Twist, còn diff_drive_controller bản
#      ROS 2 Jazzy CHỈ nhận geometry_msgs/msg/TwistStamped (bắt buộc có header.stamp).
#      Gửi Twist (không stamp) vào topic này sẽ KHÔNG có tác dụng.
#
#  CÁCH CHẠY:
#      # (1) Terminal 1: bật mô phỏng (đã bao gồm việc spawn controller)
#      ros2 launch my_mobilerobot gazebo.launch.xml
#
#      # (2) Terminal 2: chạy teleop (mặc định đã bật use_sim_time=true)
#      ros2 run my_mobilerobot teleop_keyboard
#
#      # (3) Nếu chạy trên ROBOT THẬT (không có /clock của Gazebo):
#      ros2 run my_mobilerobot teleop_keyboard --ros-args -p use_sim_time:=false
#
#  PHÍM ĐIỀU KHIỂN:
#      w : tăng vận tốc tiến   (linear.x  += 0.05 m/s,  tối đa 0.5)
#      s : giảm / lùi          (linear.x  -= 0.05 m/s,  tối thiểu -0.5)
#      a : quay trái           (angular.z += 0.20 rad/s, tối đa 2.0)
#      d : quay phải           (angular.z -= 0.20 rad/s, tối thiểu -2.0)
#      SPACE : DỪNG NGAY (đặt cả hai về 0)
#      q (hoặc Ctrl+C) : thoát (tự gửi lệnh dừng trước khi thoát)
# =====================================================================================

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped

HELP_TEXT = """
------------------------------ TELEOP DIFF DRIVE ------------------------------
   w : tien nhanh hon                s : cham lai / lui
   a : quay trai (nguoc chieu KDH)   d : quay phai (thuan chieu KDH)
   SPACE : DUNG NGAY                 q : thoat
-------------------------------------------------------------------------------
"""


class TeleopKeyboard(Node):
    """Node publish TwistStamped vào topic lệnh của diff_drive_controller."""

    def __init__(self):
        # [THÊM] use_sim_time = True: BẮT BUỘC khi chạy Gazebo.
        # LÝ DO: diff_drive_controller so sánh header.stamp của lệnh với ĐỒNG HỒ CỦA NÓ
        # để tính "tuổi lệnh" (cmd_vel_timeout). Trong Gazebo, controller dùng /clock.
        # Nếu node lại dùng giờ hệ thống thì lệnh bị coi là quá cũ và controller phanh:
        #     [WARN] Velocity command timed out. Braking.
        super().__init__(
            'teleop_keyboard',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        # ---- Các tham số chỉnh được khi chạy: --ros-args -p ten:=gia_tri ----
        self.declare_parameter('linear_step', 0.05)     # m/s mỗi lần bấm
        self.declare_parameter('angular_step', 0.20)    # rad/s mỗi lần bấm
        self.declare_parameter('max_linear', 0.50)      # m/s
        self.declare_parameter('max_angular', 2.00)     # rad/s
        self.declare_parameter('publish_rate', 10.0)    # Hz, phải > 1/cmd_vel_timeout

        self.linear = 0.0
        self.angular = 0.0

        # [THÊM] Topic lệnh của diff_drive_controller (Jazzy):
        #   tên  : /diff_drive_controller/cmd_vel  (controller dùng topic riêng ~/cmd_vel)
        #   kiểu : geometry_msgs/msg/TwistStamped
        # Kiểm tra lại bất cứ lúc nào bằng:
        #   ros2 topic info /diff_drive_controller/cmd_vel --verbose
        self.publisher_ = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        # [THÊM] Timer phát lệnh LIÊN TỤC.
        # LÝ DO: controller.yaml đặt cmd_vel_timeout: 0.5 (giây). Nếu node chỉ publish khi
        # bấm phím thì sau 0.5 s controller sẽ phanh => robot đi giật cục.
        # 10 Hz (0.1 s) là an toàn.
        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.create_timer(period, self.publish_cmd)

    def publish_cmd(self):
        """Đóng gói vận tốc hiện tại thành TwistStamped rồi gửi đi."""
        msg = TwistStamped()
        # BẮT BUỘC: điền stamp bằng đồng hồ của node (đã là /clock nếu use_sim_time=true).
        # Để stamp = 0 thì controller coi là lệnh cũ và phanh.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'   # quy ước: lệnh tính trong hệ base_footprint
        msg.twist.linear.x = self.linear         # m/s   - tiến / lùi
        msg.twist.angular.z = self.angular       # rad/s - quay trái / phải (quanh trục z)
        self.publisher_.publish(msg)

    def update_from_key(self, key):
        """Nhận 1 phím, cập nhật self.linear/self.angular. Trả True nếu là phím lệnh."""
        step_l = float(self.get_parameter('linear_step').value)
        step_a = float(self.get_parameter('angular_step').value)
        max_l = float(self.get_parameter('max_linear').value)
        max_a = float(self.get_parameter('max_angular').value)

        if key == 'w':
            self.linear = min(self.linear + step_l, max_l)
        elif key == 's':
            self.linear = max(self.linear - step_l, -max_l)
        elif key == 'a':
            self.angular = min(self.angular + step_a, max_a)
        elif key == 'd':
            self.angular = max(self.angular - step_a, -max_a)
        elif key == ' ':
            self.linear = 0.0
            self.angular = 0.0
        else:
            return False

        self.get_logger().info(
            f'linear.x = {self.linear:+.2f} m/s | angular.z = {self.angular:+.2f} rad/s',
            throttle_duration_sec=0.2)
        return True

    def publish_stop(self):
        """Gửi vài lần lệnh dừng để chắc chắn controller nhận được."""
        self.linear = 0.0
        self.angular = 0.0
        for _ in range(5):
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    # Nếu stdin không phải terminal (bị pipe) thì không đọc được bàn phím từng phím.
    if not sys.stdin.isatty():
        node.get_logger().error(
            'stdin khong phai terminal => khong doc duoc ban phim. '
            'Hay chay truc tiep trong terminal, hoac dung node demo: '
            'ros2 run my_mobilerobot demo_square')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Bật chế độ đọc từng phím (không cần bấm Enter).
    settings = termios.tcgetattr(sys.stdin)
    print(HELP_TEXT)

    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            # Cho timer publish_cmd chạy (timeout 0 => không block).
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in ('q', '\x03'):      # 'q' hoặc Ctrl+C
                    running = False
                else:
                    node.update_from_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        # DỌN DẸP: luôn gửi lệnh dừng trước khi thoát + trả terminal về chế độ bình thường.
        node.get_logger().info('Dang dung robot va thoat...')
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
