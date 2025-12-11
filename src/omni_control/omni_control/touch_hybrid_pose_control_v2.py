import rclpy
from rclpy.node import Node

from omni_msgs.msg import OmniState, OmniButtonEvent
from geometry_msgs.msg import PoseStamped

import numpy as np


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class TouchHybridPoseControl(Node):
    def __init__(self):
        super().__init__('touch_hybrid_pose_control')

        # 控制启用（灰键按住）
        self.control_enabled = False

        # Touch 的参考点（按灰键时记录）
        self.touch_ref_pos = np.zeros(3)

        # 最近一次接收到的 Touch 位置（用于作为参考点）
        self.last_touch_pos = None

        # 机械臂目标末端位置（初始化为0，可根据需要改成实际初始位姿）
        self.ee_target_pos = np.zeros(3)

        # 位置缩放系数
        self.position_scale = 0.1

        # ☆☆ 新增：整体限幅（避免 RViz 边界导致越界挂掉） ☆☆
        self.max_range = 2.0   # ±20 cm，足够大，也非常安全

        # 订阅 Touch 状态
        self.create_subscription(
            OmniState,
            '/phantom/state',
            self.touch_state_callback,
            10
        )

        # 订阅按钮
        self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        # 发布机械臂控制指令
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/touch_control/pose_cmd',
            10
        )

        self.get_logger().info(
            "Hybrid Touch Control node started.\n"
            "Hold GREY button to control (clutch mode)."
        )

    # ---------------------------
    # 按钮回调
    # ---------------------------
    def button_callback(self, msg: OmniButtonEvent):
        # 灰键按下（0 -> 1）
        if msg.grey_button == 1 and not self.control_enabled:
            # 没有最新 touch 位置就先不启动，避免 None 出错
            if self.last_touch_pos is None:
                self.get_logger().warn(
                    "Grey button pressed but no touch state received yet."
                )
                return

            self.control_enabled = True
            # 记录当前 touch 位置作为参考点
            self.touch_ref_pos = self.last_touch_pos.copy()
            self.get_logger().info("🟢 Control ENABLED")

        # 灰键松开（1 -> 0）
        elif msg.grey_button == 0 and self.control_enabled:
            self.control_enabled = False
            self.get_logger().info("🔴 Control DISABLED")

    # ---------------------------
    # Touch 状态回调（1000Hz）
    # ---------------------------
    def touch_state_callback(self, state: OmniState):
        # 更新最近一次 touch 位置
        pos_now = np.array([
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z
        ])
        self.last_touch_pos = pos_now

        # 姿态（绝对同步）
        orientation_now = state.pose.orientation

        # 未按灰键则不控制
        if not self.control_enabled:
            return

        # 增量位置 Δp = pos_now - ref
        delta_pos = pos_now - self.touch_ref_pos

        # 缩放
        delta_pos *= self.position_scale

        # 更新目标末端位置
        self.ee_target_pos += delta_pos

        # ☆☆ 新增：整体限幅（防止越界）☆☆
        # for i in range(3):
        #     self.ee_target_pos[i] = clamp(
        #         self.ee_target_pos[i],
        #         -self.max_range,
        #         self.max_range
        #     )

        # 更新参考点
        self.touch_ref_pos = pos_now

        # 组装并发布 PoseStamped
        pose_cmd = PoseStamped()
        pose_cmd.header.stamp = self.get_clock().now().to_msg()
        pose_cmd.header.frame_id = "world"

        pose_cmd.pose.position.x = float(self.ee_target_pos[0])
        pose_cmd.pose.position.y = float(self.ee_target_pos[1])
        pose_cmd.pose.position.z = float(self.ee_target_pos[2])

        pose_cmd.pose.orientation = orientation_now

        self.pose_pub.publish(pose_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TouchHybridPoseControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
