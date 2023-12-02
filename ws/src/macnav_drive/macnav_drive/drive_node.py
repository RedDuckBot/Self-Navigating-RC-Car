#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node 
from geometry_msgs.msg import Twist
import struct 

#Node passes xbox input to topic /arduino_channel
arduino = serial.Serial(port="/dev/ttyACM0",baudrate=115200)

prevAng = 0.0
prevLin = 0.0

class Arduino_node(Node):
    prevAng = 0.0
    prevLin = 0.0
    def __init__(self):
        super().__init__("manual_node")
        #create arduino subscribing node
        self.vel_sub = self.create_subscription(Twist,"/cmd_vel",
            self.send_drive_msg,10)
        self.get_logger().info("Arduino drive node initialized")

    def send_drive_msg(self,msg):
        if(msg.angular.z != self.prevAng or msg.linear.x != self.prevLin):
            sendTwist(arduino, msg.angular.z, msg.linear.x)
            self.prevAng = msg.angular.z
            self.prevLin = msg.linear.x

def sendTwist(ard, ang, lin):
    ard.write(f"{lin}\n{ang}\n".encode())


def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino_node()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    arduino.close()