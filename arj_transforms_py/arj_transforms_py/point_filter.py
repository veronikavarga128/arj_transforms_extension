import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import random

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class PointFilterNode(Node):

    def __init__(self):
        super().__init__('point_filter_node')

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.timer_callback)

        self.point_id = 0

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'orbit2',
                rclpy.time.Time()
            )

            pt = PointStamped()
            pt.header.frame_id = 'orbit2'
            pt.point.x = random.uniform(-1.0, 1.0)
            pt.point.y = random.uniform(-1.0, 1.0)
            pt.point.z = 0.0

            pt_map = do_transform_point(pt, transform)

            in_region = -2.0 <= pt_map.point.x <= 2.0 and -2.0 <= pt_map.point.y <= 2.0

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'points'
            marker.id = self.point_id
            self.point_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pt_map.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            if in_region:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            marker.color.a = 1.0

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform point: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
