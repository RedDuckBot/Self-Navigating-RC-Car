import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mac_messages.msg import Drive
import math

class Odom_test_node(Node):

    yaw = 0



    def __init__(self):
        super().__init__('odom_test_node')
        self.drive_pub = self.create_publisher(Drive, "/arduino_channel", 100)
        self.tmr = self.create_timer(0.1, self.pub)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.update_odom, 10)
        msg = Drive()
        msg.command = "S"
        msg.control_input = 100
        self.drive_pub.publish(msg)



    def quaternion_to_euler(self, q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]

    def pub(self):
        
        
        msg = Drive()
        msg.command = "F"
        p = False
        if abs(self.yaw) < 0.785:
            msg.control_input = 210
            if not p:
                self.drive_pub.publish(msg)
                p
        else:
            msg.control_input = 0
            self.drive_pub.publish(msg)
       
        print("yaw = ", self.yaw)
            
        #msg2 = Drive()
        #msg2.command = "S"
        #msg2.control_input = 100
        #self.drive_pub.publish(msg2)



    def update_odom(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        

        e = self.quaternion_to_euler(q)


        self.yaw = e[0]


def main(args=None):
    rclpy.init(args=args)
    test_node = Odom_test_node()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()