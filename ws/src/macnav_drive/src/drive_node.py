#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node 
from geometry_msgs import Twist

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
        arduino.write(msg.angular.z)
        arduino.flush()
        arduino.write(msg.linear.x)
        arduino.flush()

def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino_node()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    arduino.close()