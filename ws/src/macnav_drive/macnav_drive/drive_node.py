#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node 
from geometry_msgs.msg import Twist

#Node passes xbox input to topic /arduino_channel
arduino = serial.Serial(port="/dev/ttyACM0",baudrate=115200)

class Arduino_node(Node):
    def __init__(self):
        super().__init__("manual_node")
        #create arduino subscribing node
        self.vel_sub = self.create_subscription(Twist,"/cmd_vel",
            self.send_drive_msg,10)
        self.get_logger().info("Arduino manual node initialized")

    def send_drive_msg(self,msg):
        input("wait")
        #print(str(int(msg.angular.z)).encode(), "<- encodes")

        print("wrote: ", "10".encode(),  arduino.write("10".encode()))

        print("reading")
        line = arduino.readline()
        print(line)
        print(line.decode())

def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino_node()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    arduino.close()