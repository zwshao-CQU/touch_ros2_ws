import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class TouchControlVisualizer(Node):
    def __init__(self):
        super().__init__('touch_control_visualizer')

        # 订阅 pose_cmd
        self.create_subscription(
            PoseStamped,
            '/touch_control/pose_cmd',
            self.pose_callback,
            10
        )

        # 发布 Marker
        self.marker_pub = self.create_publisher(
            Marker,
            '/touch_control/marker',
            10
        )

        self.get_logger().info("Visualizer started.")

    def pose_callback(self, msg: PoseStamped):

        # 创建一个球体 Marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "touch_control"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 球心位置 = Touch 控制输出的位置
        marker.pose = msg.pose

        # 球体尺度（0.05m 即 5cm 大小）
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        # 颜色（红色球体）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 不透明

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TouchControlVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
