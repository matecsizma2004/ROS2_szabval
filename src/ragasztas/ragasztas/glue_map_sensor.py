"""Mock szenzor: a konfigurációs rácsot sensor_msgs/Image-ként publikálja."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from ragasztas.glue_path import grid_to_pixels, parse_grid


class GlueMapSensor(Node):

    def __init__(self):
        super().__init__('glue_map_sensor')
        rows = self.declare_parameter('grid', ['']).value
        rate = self.declare_parameter('publish_rate', 1.0).value

        grid = parse_grid(rows)
        self.msg = Image()
        self.msg.header.frame_id = 'glue_sheet'
        self.msg.height = len(grid)
        self.msg.width = len(grid[0])
        self.msg.encoding = 'mono8'
        self.msg.is_bigendian = 0
        self.msg.step = self.msg.width
        self.msg.data = grid_to_pixels(grid)

        # transient_local: a később induló feliratkozó is megkapja az utolsó képet.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Image, 'glue_map', qos)
        self.create_timer(1.0 / rate, self.publish)
        self.publish()
        self.get_logger().info(
            f'Ragasztási térkép publikálása: {self.msg.width}x{self.msg.height} '
            f'mono8 kép a /glue_map topicra, {rate} Hz')

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = GlueMapSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
