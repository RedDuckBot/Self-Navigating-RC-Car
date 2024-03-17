#!/usr/bin/env python3
import rclpy, serial, threading, signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
#from mac_messages.msg import Imu
import math
import numpy as np

odom_arduino = serial.Serial(port="/dev/ttyACM0",baudrate=115200)

class Odom_node(Node):

    TIMER_INTERVAL = 0.1 
    WHEEL_BASE = 0.265
    WHEEL_RADIUS = 0.0402  
    R = 0.0042457542

    def __init__(self):
        super().__init__('odom_publisher')
        self.current_linear_vel = 0.0 # m/s
        self.current_angular_vel = 0.0 # rads/s
        self.current_steering_angle = 0.0 #Radians, relative to an imaginary center wheel
        self.current_IMU_heading = 0.0 #Degrees, direction based on steer heading
        self.steer_heading = 0.0 #Radians, corresponding to bot's steer angle  
                            #angle offset from center of rear axis
        self.last_IMU_heading = 0.0
        self.heading_x = 0.0 # meters
        self.heading_y = 0.0
        self.forwards = True
        self.heading = 0.0
        self.lock = threading.Lock()

        self.odom_publisher = self.create_publisher(Odometry, 'odom',10)
        #self.imu_subscriber = self.create_subscription(Imu,"/imu", \
           # self.update_IMU_heading, 5)
        self.tfpublisher = self.create_publisher(TFMessage, 'tf', 10)
        self.tmr = self.create_timer(self.TIMER_INTERVAL, self.publish)
        self.dirve_sub = self.create_subscription(Twist,"/cmd_vel",
            self.update_kinematic_values,10)

        self.get_logger().info("Arduino odom node initialized")


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
                counts = np.frombuffer(counts_bytes,dtype=np.float32)[0]
            except:
                counts = 0
            
            if self.forwards:
                #Distance = linear velocity since counts are read one 1 sec. intervals
                self.current_linear_vel = (float(counts) * self.R * 2.0 * \
                    math.pi * self.WHEEL_RADIUS) 
            else:
                self.current_linear_vel = (-float(counts) * self.R * 2.0 * \
                    math.pi * self.WHEEL_RADIUS) 

            self.get_logger().info(f"{counts}")
            self.lock.acquire()
            self.heading += ((self.current_linear_vel / self.WHEEL_BASE) \
                * math.tan(self.current_steering_angle))
            self.modulate_steer_heading()            
            self.lock.release()

            self.heading_x += self.current_linear_vel * math.cos( \
                self.heading)
            self.heading_y += self.current_linear_vel * math.sin( \
                    self.heading)
            self.publish_odom(self.heading)

    def modulate_steer_heading(self):
        #convert steer heading to the following range: 0 <= heading =< 2pi

        heading_degrees = self.heading * (180/math.pi)
        heading_degrees = heading_degrees % 360
        self.heading = heading_degrees * (math.pi/180)

    def publish_odom(self, heading):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self.heading_x
        msg.pose.pose.position.y = self.heading_y
        msg.pose.pose.position.z = 0.0
        r = self.euler_to_quaternion([heading, 0.0, 0.0])
        msg.pose.pose.orientation.x = r[0]
        msg.pose.pose.orientation.y = r[1]
        msg.pose.pose.orientation.z = r[2]
        msg.pose.pose.orientation.w = r[3]
        msg.child_frame_id = "base_link"
 
        self.odom_publisher.publish(msg)
        self.publish_tf(msg)

    def publish_tf(self, odom):
        tf = TransformStamped()

        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation =odom.pose.pose.orientation

        msg = TFMessage()

        msg.transforms = [tf]
        self.tfpublisher.publish(msg)

    def update_kinematic_values(self, twist):
        #Update angular velocity and steering angle
        target_lin_velocity = twist.linear.x
        target_ang_velocity = twist.angular.z
        self.current_angular_vel = target_ang_velocity 
        self.current_steering_angle = self.get_steer_angle(target_lin_velocity, \
            target_ang_velocity)

        #Update Drive direction
        if twist.linear.x > 0.0:
            self.forwards = True
        elif twist.linear.x < 0.0:
            self.forwards = False
        else: #Robot is not in motion 
            pass
    
    def update_IMU_heading(self,message):
        self.lock.acquire()
        self.heading = (360 - message.heading)* (math.pi/180)
        self.lock.release()
        #self.get_logger().info(f"imu {360 - message.heading}")

    def get_steer_angle(self, vel_x, ang_z):
        if vel_x == 0.0:
            return self.current_steering_angle 
        return math.atan((ang_z/vel_x) * self.WHEEL_BASE)

def main(args=None):
    rclpy.init(args=args)
    odom_node = Odom_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    odom_arduino.close()
