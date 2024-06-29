import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
import tf2_ros



class PathToTFConverterNode(Node):



    def __init__(self):
        super().__init__('path_to_tf_converter_node')
        self.subscription = self.create_subscription(
            Path,
            'path_topic',
            self.path_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.poses = []
        # NOTE: We need to republish because otherwise the tf's will disapear
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer alle 0.1 Sekunden




    def path_callback(self, msg):
        self.poses = msg.poses  # Speichern der empfangenen Posen
        self.get_logger().info("Recived new 3D poses")

    

    def timer_callback(self):
        for i, pose in enumerate(self.poses):
            transform_stamped = TransformStamped()

            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'map'
            transform_stamped.child_frame_id = f'pose_{i}'

            transform_stamped.transform.translation.x = pose.pose.position.x
            transform_stamped.transform.translation.y = pose.pose.position.y
            transform_stamped.transform.translation.z = pose.pose.position.z

            transform_stamped.transform.rotation = pose.pose.orientation

            self.tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().info(f'Published TF for pose {i}')



def main(args=None):
    rclpy.init(args=args)
    node = PathToTFConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
