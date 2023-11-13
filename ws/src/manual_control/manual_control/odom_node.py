import rclpy, serial
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mac_messages.msg import Drive
import math

sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200)


class Odom_node(Node):

    _TIMER_INTERVAL = 0.1
    _WHEEL_BASE = 0.265
    _WHEEL_RADIUS = 0.0402
    _R = 0.0042457542
    _STEER_RATIO = 0.005


    _steer = 0
    _x = 0
    _y = 0
    _heading = 0

    


    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tmr = self.create_timer(self._TIMER_INTERVAL, self.publish)
        self.dirve_sub = self.create_subscription(Drive,"/arduino_channel",
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
                print(counts, self._steer)
            except:
                print("Failed to get msg")
                counts = 0


            lin = float(counts) * self._R * 2.0 * math.pi * self._WHEEL_RADIUS
            ang = math.tan(self._steer * self._STEER_RATIO) * lin / self._WHEEL_BASE


            dir = self._heading - ang/2
            self._x += lin * math.cos(dir)
            self._y += lin * math.sin(dir)
            self._heading -= ang


            print("x =", self._x, 
                  "\ny =", self._y, 
                  "\nlin =", lin,
                  "\nang =", ang,
                  "\nh =", self._heading
                  )


            print("-----")
            self.publish_odom()



        

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        r = self.euler_to_quaternion([self._heading, 0.0, 0.0])
        msg.pose.pose.orientation.x = r[0]
        msg.pose.pose.orientation.y = r[1]
        msg.pose.pose.orientation.z = r[2]
        msg.pose.pose.orientation.w = r[3]
        msg.child_frame_id = "base_link"
 
        self.publisher_.publish(msg)

    def update_steer(self, msg):
        if msg.command == "S":
            self._steer = msg.control_input


def main(args=None):
    rclpy.init(args=args)
    odom_node = Odom_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()