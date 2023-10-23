#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


#Node takes controller inputs from /joy and publishes
#inputs on /turtle1/cmd_vel to control turtlesim bot
class turtleNode(Node):
    def __init__(self):
        super().__init__("turtlePublisher")
        #create turtle publishing node
        self.publisher = self.create_publisher(Twist,"turtle1/cmd_vel",10)

        #create controller subscribing node
        self.subscribe = self.create_subscription(Joy,"joy",self.joy_callback,
                                                  10)

    def joy_callback(self, msg: Joy):
        twist = Twist()
        twist.linear.x = 4*msg.axes[1]
        twist.angular.z = 4*msg.axes[0]
        self.publisher.publish(twist)
        print(msg.axes[3])


def main(args=None):
    rclpy.init(args=args)

    t_node = turtleNode()
    rclpy.spin(t_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    