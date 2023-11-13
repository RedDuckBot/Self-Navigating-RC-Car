#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
import math

sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200)


class Odom_node(Node):

    TIMER_INTERVAL = 0.1
    WHEEL_BASE = 0.265
    WHEEL_RADIUS = 0.0402
    R = 0.0042457542
    STEER_RATIO = 0.005


    steer = 0
    forwards = True
    x = 0
    y = 0
    heading = 0 #corresponds to yaw

    


    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tmr = self.create_timer(self.TIMER_INTERVAL, self.publish)
        self.dirve_sub = self.create_subscription(Twist,"/cmd_vel",
            self.update_steer,10)


    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish(self):

        if sensor_arduino.is_open:
            try:
                counts = sensor_arduino.readline().decode().strip()
            except:
                counts = 0

            if self.forwards:
                lin = float(counts) * self.R * 2.0 * math.pi * self.WHEEL_RADIUS
                ang = math.tan(self.steer * self.STEER_RATIO) * lin / self.WHEEL_BASE
            else:
                lin = -float(counts) * self.R * 2.0 * math.pi * self.WHEEL_RADIUS
                ang = math.tan(self.steer * self.STEER_RATIO) * lin / self.WHEEL_BASE

            dir = self.heading - ang/2
            self.x += lin * math.cos(dir)
            self.y += lin * math.sin(dir)
            self.heading -= ang

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
 
        self.publisher.publish(msg)

    def update_steer(self, msg):
        self.steer = msg.angular.z
        
        if msg.linear.x > 0:
            self.forwards = True
        elif msg.linear.x < 0: 
            self.forwards = False



def main(args=None):
    rclpy.init(args=args)
    odom_node = Odom_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()