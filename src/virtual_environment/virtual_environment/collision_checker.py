import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np


class CollisionChecker(Node):

    def __init__(self):
        super().__init__('collision_checker')

        # 订阅 Touch 末端位置（你可以切换不同来源）
        self.create_subscription(
            PoseStamped,
            '/touch_control/pose_cmd',    # 来自你 Hybrid Pose Control 节点
            self.touch_callback,
            10
        )

        # 发布接触点 marker（用于 RViz 显示）
        self.contact_marker_pub = self.create_publisher(
            Marker,
            '/virtual_env/contact_point',
            10
        )

        # -------- 虚拟墙参数（需与 virtual_wall_publisher 保持一致） --------
        self.wall_center_z = 0.20
        self.wall_thickness = 0.02
        self.wall_frame = "world"

        # 墙面平面位置（法向朝 -Z）
        self.wall_surface_z = self.wall_center_z + self.wall_thickness / 2.0

        self.get_logger().info("Collision checker started.")

    # ----------------------------------------------------------
    # Touch 末端位置回调
    # ----------------------------------------------------------
    def touch_callback(self, msg: PoseStamped):

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        # 与墙的带符号距离：d >= 0 → 未触碰；d < 0 → 穿透
        d = pz - self.wall_surface_z

        # 穿透深度
        penetration = max(0.0, -d)

        # 墙法向（朝向 -Z）
        normal = np.array([0.0, 0.0, -1.0])

        # 接触点位置（投影在墙面上）
        contact_point = np.array([px, py, self.wall_surface_z])

        # 在终端打印
        self.get_logger().info(
            f"Touch z={pz:.3f}, wall_z={self.wall_surface_z:.3f}, d={d:.3f}, penetration={penetration:.3f}"
        )

        # 发布接触点 marker（越靠近墙越显眼）
        self.publish_contact_marker(contact_point, penetration)

    # ----------------------------------------------------------
    # 在 RViz 中显示接触点
    # ----------------------------------------------------------
    def publish_contact_marker(self, contact_point, penetration):

        marker = Marker()
        marker.header.frame_id = self.wall_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "contact"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(contact_point[0])
        marker.pose.position.y = float(contact_point[1])
        marker.pose.position.z = float(contact_point[2])

        marker.pose.orientation.w = 1.0

        # 球大小随穿透程度改变
        marker.scale.x = 0.03 + penetration * 2.0
        marker.scale.y = 0.03 + penetration * 2.0
        marker.scale.z = 0.03 + penetration * 2.0

        # 颜色：未接触 = 蓝色；接触 = 红色
        if penetration > 0:
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0
        else:
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 1.0
            marker.color.a = 0.6

        marker.lifetime.sec = 0
        self.contact_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
