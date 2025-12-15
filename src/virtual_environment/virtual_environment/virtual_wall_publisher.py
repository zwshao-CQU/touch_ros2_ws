import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose


class VirtualWallPublisher(Node):
    def __init__(self):
        super().__init__('virtual_wall_publisher')

        # 发布 Marker 到 RViz
        self.marker_pub = self.create_publisher(
            Marker,
            '/virtual_env/wall_marker',
            10
        )

        # 定时发布（10 Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 墙参数（你之后可以按需修改）
        self.wall_frame_id = 'world'   # 你选的坐标系
        self.wall_center_z = 0.50      # 墙中心在 z=0.5 m
        self.wall_size_x = 1.0         # 墙宽度 1 m
        self.wall_size_y = 1.0         # 墙高度 1 m
        self.wall_thickness = 0.02     # 墙厚度 2 cm

        self.get_logger().info('Virtual wall publisher started (frame: world).')

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = self.wall_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'virtual_wall'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # 墙中心位置：放在 (0, 0, wall_center_z)
        marker.pose = Pose()
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.wall_center_z

        # 暂时不旋转（墙的法向朝 +Z 或 -Z 可以之后用 RPY 再调）
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 尺寸（x：宽；y：高；z：厚度）
        marker.scale.x = self.wall_size_x
        marker.scale.y = self.wall_size_y
        marker.scale.z = self.wall_thickness

        # 颜色（红色、不透明）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # 让 Marker 不自动消失
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualWallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
