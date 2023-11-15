#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node 
from mac_messages.msg import Drive 

#Node passes xbox input to topic /arduino_channel

arduino = serial.Serial(port="/dev/ttyACM0",baudrate=115200)

class Arduino_node(Node):
    def __init__(self):
        input("waiting1...")
        super().__init__("manual_node")
        input("waiting2...")
        #create arduino subscribing node
        self.arduino_sub = self.create_subscription(Drive,"/arduino_channel",
            self.send_drive_msg,10)
        self.get_logger().info("Arduino node initialized")

    def send_drive_msg(self,msg):
        command = msg.command

        if command in ["E","D"]:
            #command for drive modes
            arduino.write(command.encode())
            arduino.flush()
        elif command in ["S","F","B"]:
            #command for steering, forward and backward modes
            arduino.write(command.encode())
            arduino.flush()
            arduino.write(str(msg.control_input).encode())
            arduino.flush()
        else:
            self.get_logger().info("Invalid drive message")

def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino_node()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    arduino.close()