#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
from xbox360controller import Xbox360Controller
from mac_messages.msg import Drive as D_MSG

#Node passes xbox input to topic /arduino_channel

class Xbox360_contr_node(Node):
    SPEEDS = [0,120,190,255]

    def __init__(self):
        super().__init__("xbox_controller")
        self.speedIndex = 1
        self.drive_mode_isEnabled = False

        #Initialize and setup xbox controller
        self.controller = Xbox360Controller(0,axis_threshold=0)
        self.controller.button_a.when_pressed = self.on_drive_mode
        self.controller.axis_r.when_moved = self.on_axis_moved 
        self.controller.button_x.when_pressed = self.on_speed
        self.controller.button_y.when_pressed = self.on_speed

        #create xbox controller publishing node
        self.xbox_pub = self.create_publisher(D_MSG,"/arduino_channel",10)

        self.get_logger().info("xbox node initialized")

    #Callback function associated with drive_mode change
    def on_drive_mode(self, button):
        msg = D_MSG()

        if (not self.drive_mode_isEnabled):
            msg.command = "E"
            self.drive_mode_isEnabled = True
            self.get_logger().info("Drive mode Enabled")
        else:
            msg.command = "D"
            self.drive_mode_isEnabled = False
            self.get_logger().info("Drive mode Disabled")
            self.speedIndex = 1

        self.xbox_pub.publish(msg)

    #Callback function associated with steering servo angle
    def on_axis_moved(self, axis):

        if self.drive_mode_isEnabled: 
            msg = D_MSG()
            msg.command = "S"
            #Check if joystick is not at neutral
            if not (axis.x > -0.25 and axis.x < 0.25):
                msg.control_input = int(axis.x * 100)
            else:
                msg.control_input = 0
            self.xbox_pub.publish(msg)

    #Callback function associated with shifting speed
    def on_speed(self, button):
        msg = D_MSG()

        if self.drive_mode_isEnabled:
            if button.name == "button_x": 
                #Move forward
                msg.command = "F"
            else: 
                #Move backward
                msg.command = "B"
            msg.control_input = self.SPEEDS[self.speedIndex]
            self.speedIndex = (1 + self.speedIndex) % 4
            self.get_logger().info(f"Speed shifted {self.SPEEDS[self.speedIndex - 1]} {(msg.command)}")

            self.xbox_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    contr_node = Xbox360_contr_node()
    rclpy.spin(contr_node)
    contr_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()