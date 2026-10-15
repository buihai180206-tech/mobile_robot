#!/usr/bin/env python3
# =====================================================================================
#  [THÊM] demo_square.py - Robot TỰ chạy theo lộ trình cố định để KIỂM TRA ros2_control.
#
#  MỤC ĐÍCH: chứng minh vòng điều khiển kín hoạt động đúng mà KHÔNG cần bấm phím nào.
#  Đây là "bài test sức khoẻ" của cả hệ thống.
#
#  VÒNG DỮ LIỆU ĐƯỢC KIỂM TRA:
#      demo_square.py
#        -> publish /diff_drive_controller/cmd_vel (geometry_msgs/TwistStamped)
#        -> diff_drive_controller: động học nghịch -> 2 command_interface velocity
#        -> GazeboSimSystem.write() -> Gazebo cho 2 bánh quay
#        -> GazeboSimSystem.read()  -> 2 state_interface (position, velocity)
#        -> /diff_drive_controller/odom (+ TF odom -> base_footprint)
#        -> demo_square.py đọc odom để quyết định khi nào rẽ (PHẢN HỒI KÍN)
#
#  LỘ TRÌNH: đi thẳng side_length (m) -> quay 90 độ, lặp lại num_sides lần (hình vuông).
#  Kết thúc: tự dừng + in "sai số khép kín" (robot có quay về gần điểm xuất phát không).
#  Nếu bánh quay SAI HƯỚNG hoặc wheel_radius/wheel_separation sai thì sai số này sẽ lớn.
#
#  CÁCH CHẠY:
#      # Terminal 1
#      ros2 launch my_mobilerobot gazebo.launch.xml
#      # Terminal 2
#      ros2 run my_mobilerobot demo_square
# =====================================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def wrap_angle(a):
    """Đưa góc về khoảng (-pi, pi] để so sánh góc cho đúng."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DemoSquare(Node):

    def __init__(self):
        super().__init__(
            'demo_square',
            # [THÊM] Bắt buộc khi chạy Gazebo: dùng /clock thay cho giờ hệ thống,
            # nếu không diff_drive_controller coi lệnh là quá cũ và phanh.
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        self.declare_parameter('side_length', 1.0)      # m - độ dài mỗi cạnh
        self.declare_parameter('linear_speed', 0.20)    # m/s
        self.declare_parameter('angular_speed', 0.60)   # rad/s
        self.declare_parameter('num_sides', 4)

        # [THÊM] Ghi lệnh vào topic của diff_drive_controller (TwistStamped, Jazzy).
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        # [THÊM] Đọc odom (phản hồi kín). Odom do chính diff_drive_controller phát ra
        # từ state_interface velocity mà nó đọc được từ Gazebo.
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.on_odom, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self.state = 'WAIT_ODOM'        # WAIT_ODOM -> DRIVE -> TURN -> ... -> DONE
        self.side_idx = 0
        self.start = (0.0, 0.0, 0.0)    # toạ độ + yaw lúc bắt đầu pha hiện tại

        self.create_timer(0.1, self.loop)   # 10 Hz > 1/cmd_vel_timeout (0.5s)
        self.get_logger().info('demo_square: dang doi odom tu diff_drive_controller...')

    # ------------------------------------------------------------------ odom callback
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        # Chuyển quaternion -> yaw (góc quay quanh trục z)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_ready = True

    # ------------------------------------------------------------------ gửi lệnh
    def send(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()   # BẮT BUỘC có stamp
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    # ------------------------------------------------------------------ vòng điều khiển
    def loop(self):
        if not self.odom_ready:
            return   # chưa có odom => chưa biết đang ở đâu => chưa làm gì

        side = float(self.get_parameter('side_length').value)
        v = float(self.get_parameter('linear_speed').value)
        w = float(self.get_parameter('angular_speed').value)
        n = int(self.get_parameter('num_sides').value)

        if self.state == 'WAIT_ODOM':
            self.start = (self.x, self.y, self.yaw)
            self.get_logger().info(
                f'Bat dau: di thang {side} m roi quay 90 do, lap {n} lan.')
            self.state = 'DRIVE'

        elif self.state == 'DRIVE':
            dx, dy = self.x - self.start[0], self.y - self.start[1]
            if math.hypot(dx, dy) >= side:
                self.send(0.0, 0.0)                       # tới nơi -> dừng lại
                self.start = (self.x, self.y, self.yaw)   # mốc mới cho pha quay
                self.state = 'TURN'
                self.get_logger().info(
                    f'Canh {self.side_idx + 1}/{n}: da di {math.hypot(dx, dy):.2f} m '
                    '-> bat dau quay 90 do.')
            else:
                self.send(v, 0.0)                         # đi thẳng

        elif self.state == 'TURN':
            turned = abs(wrap_angle(self.yaw - self.start[2]))
            if turned >= math.pi / 2.0:
                self.send(0.0, 0.0)
                self.side_idx += 1
                if self.side_idx >= n:
                    self.state = 'DONE'
                    self.report()
                else:
                    self.start = (self.x, self.y, self.yaw)
                    self.state = 'DRIVE'
                    self.get_logger().info(f'Da quay {math.degrees(turned):.1f} do.')
            else:
                self.send(0.0, w)                         # quay trai (CCW)

        elif self.state == 'DONE':
            self.send(0.0, 0.0)                           # giữ yên

    # ------------------------------------------------------------------ tổng kết
    def report(self):
        err = math.hypot(self.x, self.y)
        self.get_logger().info('==============================================================')
        self.get_logger().info('HOAN THANH LO TRINH HINH VUONG')
        self.get_logger().info(
            f'  Vi tri cuoi (he odom) : x = {self.x:+.3f} m , y = {self.y:+.3f} m')
        self.get_logger().info(
            f'  Sai so khep kin       : {err:.3f} m  (cang nho cang tot)')
        self.get_logger().info(
            '  Neu sai so > 0.2 m: kiem tra wheel_radius / wheel_separation '
            'trong controller.yaml')
        self.get_logger().info('==============================================================')


def main(args=None):
    rclpy.init(args=args)
    node = DemoSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)     # luôn dừng robot khi thoát
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
