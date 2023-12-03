#!/usr/bin/env python3
import rclpy, serial, threading, signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

import math
import numpy as np

odom_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200)

class Odom_node(Node):

    TIMER_INTERVAL = 0.1 
    WHEEL_BASE = 0.265
    WHEEL_RADIUS = 0.0402  
    R = 0.0042457542

    def __init__(self):
        super().__init__("odom_publsiher")
        self.current_steering_angle = 0.0
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0 
        self.forwards = True
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tfpublisher = self.create_publisher(TFMessage, 'tf', 10)
        self.tmr = self.create_timer(self.TIMER_INTERVAL, self.publish)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', 
            self.update_kinematics, 10)
        
        self.get_logger().info("Kinematic Arduino odom node init")

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish(self):
        if odom_arduino.is_open:
            try:
                counts_bytes = odom_arduino.read_until(size=4)
                counts = np.frombuffer(counts_bytes, dtype=np.float32)[0]
            except:
                counts = 0.0
            
            #calculate linear distance traveled
            if self.forwards:
                linear_d = (float(counts) * self.R * 2.0 * \
                    math.pi * self.WHEEL_RADIUS)
            else:
                linear_d = (-float(counts) * self.R * 2.0 * \
                    math.pi * self.WHEEL_RADIUS)
            

            #change in heading from last sample to this one
            d_heading = ((linear_d / self.WHEEL_BASE)) \
                * math.tan(self.current_steering_angle)

            #avg change in heading over that time
            avg_d_heading = d_heading / 2

            #trig to calc distance traveled
            self.x = linear_d * math.cos(self.heading + avg_d_heading)
            self.y = linear_d * math.sin(self.heading + avg_d_heading)
            
            # update heading
            self.heading += d_heading
            self.get_logger().info(f"{counts}")

            self.publish_odom()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        r = self.euler_to_quaternion([self.heading, 0.0, 0.0])
        msg.pose.pose.orientation.x = r[0]
        msg.pose.pose.orientation.y = r[1]
        msg.pose.pose.orientation.z = r[2]
        msg.pose.pose.orientation.w = r[3]
        msg.child_frame_id = "base_link"

        self.publish_tf(msg)
        self.odom_publisher.publish(msg)

    def publish_tf(self, odom):
        tf = TransformStamped()

        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        msg = TFMessage()

        msg.transforms = [tf]
        self.tfpublisher.publish(msg)
        
    def update_kinematics(self, twist):
        target_lin_velocity = twist.linear.x
        target_ang_velocity = twist.angular.z
        self.current_steering_angle = self.get_steer_angle(target_lin_velocity, \
            target_ang_velocity)

        if twist.linear.x > 0:
            self.forwards = True
        elif twist.linear.x < 0.0:
            self.forwards = False
        else:
            pass
        
    def get_steer_angle(self, x, z):
        if z == 0:
            return self.current_steering_angle
        return math.atan((z/x) * self.WHEEL_BASE)

def main(args=None):
    rclpy.init(args=args)
    odom_node = Odom_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    odom_arduino.close()

