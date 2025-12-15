import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3, Point
from visualization_msgs.msg import Marker
from omni_msgs.msg import OmniFeedback
import numpy as np


class ForceFeedbackNode(Node):

    def __init__(self):
        self.prev_s = None
        self.click_time = None
        # ---------- 接触 click 参数 ----------
        self.click_amp = 1.0      # N，接触瞬间的脉冲幅值
        self.click_tau = 0.05     # s，指数衰减时间常数（50 ms）
        self.click_time = None    # 时间戳


        super().__init__('force_feedback_node')

        # ===============================
        # 状态
        # ===============================
        self.z_virtual = None
        self.side_sign = None

        # ===============================
        # 虚拟墙参数
        # ===============================
        self.wall_center_z = 0.50
        self.wall_thickness = 0.02
        self.wall_surface_z = self.wall_center_z + self.wall_thickness / 2.0

        # ===============================
        # 力场参数（更“轻”的力场）
        # ===============================
        self.field_range = 0.50     # 20 cm
        # self.F_max = 3.5            # N（接触区限幅）

        self.K_field = 3.0         # N 级别的“轻外推”（注意这里我们把它当 N 用）
        self.K_wall = 1000.0          # N/m  靠近/接触墙面时的“硬度”（建议 400~900 起调）

        self.B_min = 6.0            # Ns/m
        self.B_max = 70.0           # Ns/m

        # 远场速度阻力（更轻）
        self.B_field_scale = 0.10   # 0.1*B_min*(xi^2)

        # 几何硬止挡（不可穿透）
        self.pen_hard_limit = 0.01  # 1 cm

        # ===============================
        # 速度估计
        # ===============================
        self.last_z = None
        self.last_time = None
        self.v_est = 0.0
        self.v_clip = 0.30
        self.v_alpha = 0.15

        # ===============================
        # ROS 接口
        # ===============================

        # 订阅“原始 Touch 位姿命令”
        self.create_subscription(
            PoseStamped,
            '/touch_control/pose_cmd',
            self.touch_callback,
            10
        )

        # 发布“几何约束后的位姿”（给 RViz 位姿球/控制节点用）
        self.pose_virtual_pub = self.create_publisher(
            PoseStamped,
            '/touch_control/pose_cmd_virtual',
            10
        )

        # 力反馈
        self.force_pub = self.create_publisher(
            OmniFeedback,
            '/phantom/force_feedback',
            10
        )

        # 力向量可视化
        self.force_marker_pub = self.create_publisher(
            Marker,
            '/virtual_env/force_vector',
            10
        )

        self.get_logger().info("Force Feedback Node started (constraint pose + smooth impedance wall).")

    def touch_callback(self, msg: PoseStamped):
        z_raw = float(msg.pose.position.z)

        # ---------- 初始化虚拟位置 ----------
        if self.z_virtual is None:
            self.z_virtual = z_raw

        # ---------- 锁定单侧（只做一次） ----------
        if self.side_sign is None:
            # 自由空间在哪一侧
            self.side_sign = -1.0 if z_raw < self.wall_surface_z else 1.0
            side_str = "below (z<wall)" if self.side_sign < 0 else "above (z>wall)"
            self.get_logger().info(f"Wall side locked: free space is {side_str}, side_sign={self.side_sign}")

        # 法向（指向自由空间）
        normal = np.array([0.0, 0.0, self.side_sign], dtype=float)

        # ---------- 不可穿透：受限更新 z_virtual ----------
        dz_raw = z_raw - self.z_virtual
        dz_n = dz_raw * self.side_sign                     # >0 离墙，<0 进墙
        s_virtual = (self.z_virtual - self.wall_surface_z) * self.side_sign  # >0 自由侧距离，<0 穿透

        dz_allowed = dz_raw
        # 已经到达硬止挡且还想继续进墙 -> 冻结
        if s_virtual <= -self.pen_hard_limit and dz_n < 0.0:
            dz_allowed = 0.0

        self.z_virtual += dz_allowed

        # 重新计算 s（严格基于虚拟位置）
        s = (self.z_virtual - self.wall_surface_z) * self.side_sign

        # 双保险夹持（防浮点/累积误差）
        if s < -self.pen_hard_limit:
            self.z_virtual = self.wall_surface_z - self.side_sign * self.pen_hard_limit
            s = -self.pen_hard_limit

        # ---------- 发布“几何约束后的虚拟位姿” ----------
        pose_virtual = PoseStamped()
        pose_virtual.header = msg.header
        pose_virtual.pose.position.x = msg.pose.position.x
        pose_virtual.pose.position.y = msg.pose.position.y
        pose_virtual.pose.position.z = float(self.z_virtual)
        pose_virtual.pose.orientation = msg.pose.orientation
        self.pose_virtual_pub.publish(pose_virtual)

        # ---------- 速度估计（基于虚拟位置） ----------
        now = self.get_clock().now()
        v_raw = 0.0
        if self.last_z is not None and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 1e-6:
                v_raw = (self.z_virtual - self.last_z) / dt

        self.last_z = self.z_virtual
        self.last_time = now

        v_raw = float(np.clip(v_raw, -self.v_clip, self.v_clip))
        self.v_est = (1.0 - self.v_alpha) * self.v_est + self.v_alpha * v_raw
        v = self.v_est
        v_n = v * self.side_sign  # 法向速度

        # ---------- 力计算 ----------
        F_total = np.zeros(3, dtype=float)

        # ==============================
        # Zone 1：远场力场（只有距离斥力）
        # wall_thickness < s <= field_range
        # ==============================
        if self.wall_thickness < s <= self.field_range:

            ratio = 1.0 - s / self.field_range        # 远→近：0→1
            ratio = float(np.clip(ratio, 0.0, 1.0))

            # 远场斥力（轻，N 级）
            F_field_push = (self.K_field * (ratio ** 1)) * normal

            F_total = F_field_push


        # ==============================
        # Zone 2：墙面区（远场 + 强墙面阻力叠加）
        # 0 < s <= wall_thickness
        # ==============================
        elif 0.0 < s <= self.wall_thickness:

            # ---------- 远场力仍然存在 ----------
            ratio = 1.0 - s / self.field_range
            ratio = float(np.clip(ratio, 0.0, 1.0))
            F_field_push = (self.K_field * (ratio ** 1)) * normal

            # ---------- 墙面力 ----------
            xi = 1.0 - s / self.wall_thickness        # 0→1
            xi = float(np.clip(xi, 0.0, 1.0))

            # 刚度随接近墙面迅速增大
            K_eff = self.K_wall * (xi ** 2)

            # 墙面压缩位移
            x = self.wall_thickness - s               # s=wall_thickness → 0

            F_spring = (K_eff * x) * normal

            # 墙面阻尼（稳住）
            B_eff = self.B_min + (self.B_max - self.B_min) * (xi ** 2)
            if v_n < 0.0:
                F_damp = (-B_eff * v_n) * normal
            else:
                F_damp = (-0.2 * B_eff * v_n) * normal

            # 直接叠加（符合你说的逻辑）
            F_total = F_field_push + F_spring + F_damp


        # ==============================
        # Zone 3：接触 / 穿透区（s <= 0）
        # 强硬、稳住、不再靠位移变大
        # ==============================
        elif s <= 0.0:

            # ---------- 远场力仍然存在（作为基础推力） ----------
            ratio = 1.0 - 0.0 / self.field_range   # 已到墙面，ratio=1
            F_field_push = (self.K_field * (ratio ** 1)) * normal

            # ---------- 墙面“等效最大刚度” ----------
            # 这里不再用 x = wall_thickness - s
            # 而是固定在 wall_thickness，避免硬弹簧
            K_eff = self.K_wall
            x = self.wall_thickness

            F_spring = (K_eff * x) * normal

            # ---------- 强阻尼：核心是“稳住” ----------
            # 只要还在往墙里推，就给强阻尼
            if v_n < 0.0:
                F_damp = (-self.B_max * v_n) * normal
            else:
                F_damp = (-0.2 * self.B_max * v_n) * normal

            # ---------- 合力 ----------
            F_total = F_field_push + F_spring + F_damp



        # ==============================
        # 接触 click（附加效果）
        # ==============================
        if s < 0.0 and (self.prev_s is not None and self.prev_s >= 0.0):
            self.click_time = self.get_clock().now()

        F_click = np.zeros(3)
        if self.click_time is not None:
            dt = (self.get_clock().now() - self.click_time).nanoseconds * 1e-9
            if dt < 4 * self.click_tau:
                F_click = (self.click_amp * np.exp(-dt / self.click_tau)) * normal
            else:
                self.click_time = None

        F_total = F_total + F_click


        # ---------- 发布力反馈 ----------
        feedback = OmniFeedback()
        feedback.force = Vector3(
            x=float(F_total[0]),
            y=float(F_total[1]),
            z=float(F_total[2])
        )
        feedback.position = Vector3(x=0.0, y=0.0, z=0.0)
        self.force_pub.publish(feedback)

        # ---------- 力向量可视化 ----------
        virtual_pos = Point(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(self.z_virtual)
        )
        self.publish_force_marker(virtual_pos, F_total)

        self.get_logger().info(f"s={s:.3f}, z_raw={z_raw:.3f}, z_virtual={self.z_virtual:.3f}, |F|={np.linalg.norm(F_total):.2f}")
        self.prev_s = s


    def publish_force_marker(self, pos: Point, force: np.ndarray):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "force_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point(x=float(pos.x), y=float(pos.y), z=float(pos.z))

        vis_scale = 0.5
        end = Point(
            x=float(pos.x + force[0] * vis_scale),
            y=float(pos.y + force[1] * vis_scale),
            z=float(pos.z + force[2] * vis_scale)
        )

        marker.points = [start, end]

        marker.scale.x = 0.10
        marker.scale.y = 0.20
        marker.scale.z = 0.18

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.force_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
