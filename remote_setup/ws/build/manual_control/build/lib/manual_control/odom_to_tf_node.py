#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class odom_to_tf_node(Node):

    def __init__(self):
        super().__init__("odom_to_tf")

        self.tf_pub = self.create_publisher(TransformStamped, "/tf", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.send_tf, 10)


    def send_tf(self, odom):
        tf = TransformStamped()

        tf.header = odom.header
        tf.header.frame_id = odom.header.frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_pub.publish(tf)
        


def main(args=None):
    rclpy.init(args=args)
    node = odom_to_tf_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()