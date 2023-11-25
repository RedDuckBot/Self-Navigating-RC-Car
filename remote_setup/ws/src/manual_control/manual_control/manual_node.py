#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node 
from xbox360controller import Xbox360Controller
from geometry_msgs.msg import Twist


#Node passes xbox input to topic /arduino_channel

class Xbox360_contr_node(Node):
    SPEEDS = [0.0, 0.6557, 0.8461, 1.23] # m/s

    #Scaled by order of 2 from joystick inputs
    JOY_MIN = -100
    JOY_MAX = 100

    #Steering servo angle range
    STEER_MIN = 0
    STEER_MAX = 76
    STEER_CENTER = 38
    WHEEL_BASE = 0.265 #meters

    prev_ang = 0
    prev_lin = 0

    def __init__(self):
        super().__init__("xbox_controller")
        self.speedIndex = 1
        self.cur_angular_vel =  0.0
        self.current_linear_vel = 0.0
        self.allow_joystick_steering = False

        #Initialize and setup xbox controller
        self.controller = Xbox360Controller(0,axis_threshold=0)
        self.controller.button_b.when_pressed = self.apply_brake
        self.controller.axis_r.when_moved = self.on_joy_move 
        self.controller.button_x.when_pressed = self.on_speed #Forwards
        self.controller.button_y.when_pressed = self.on_speed #Reverse
        self.controller.button_a.when_pressed = self.steering_mode_toggle

        #create xbox controller publishing node
        self.xbox_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.get_logger().info("xbox node initialized")

    #Callback function associated with drive_mode change
    def apply_brake(self, button):
        msg = self.create_message(self.SPEEDS[0],self.cur_angular_vel)

        self.speedIndex = 1
        print("Brake applied!")
        self.xbox_pub.publish(msg)

    #Callback function disables/enables joystick steering
    def steering_mode_toggle(self, button):
        if self.allow_joystick_steering == False:
            print("Joystick steering enabled.")
            self.allow_joystick_steering = True
        else:
            print("Joystick steering disabled.")
            self.allow_joystick_steering = False

    #Callback function associated with steering servo angle
    def on_joy_move(self, axis):
        if self.allow_joystick_steering:
            #Check if joystick is not at neutral
            if not (axis.x > -0.25 and axis.x < 0.25):
                joy_input = int(axis.x * 100)
                servo_angle = self.map_range(joy_input)  
                #steer_angle =  self.STEER_CENTER - servo_angle
                steer_angle = self.get_steering_angle(servo_angle)
                #if (servo_angle > self.STEER_CENTER): steer_angle *= -1 
                angular_vel_z = self.current_linear_vel * \
                    ((math.tan(steer_angle)) / self.WHEEL_BASE)
            else:
                angular_vel_z = 0.0

            if self.current_linear_vel != self.prev_lin or angular_vel_z != self.prev_ang:
                msg = self.create_message(self.current_linear_vel, angular_vel_z )  
                self.xbox_pub.publish(msg)
                self.prev_ang = angular_vel_z
                self.prev_lin = self.current_linear_vel

    #Callback function associated with shifting speed
    def on_speed(self, button):
        if button.name == "button_x": 
            #Move forward
            lin_vel_x = self.SPEEDS[self.speedIndex] 
            print("Moving forward")
            print(f"Speed: {self.SPEEDS[self.speedIndex]}")
        else: 
            #Move backward
            print("Moving Backward")
            lin_vel_x = (-1) * self.SPEEDS[self.speedIndex] 

        print(f"Speed shifted: {self.SPEEDS[self.speedIndex]} m/s")
        self.speedIndex = (self.speedIndex + 1) % len(self.SPEEDS)
        if lin_vel_x != self.prev_lin or self.cur_angular_vel != self.prev_ang:
            msg = self.create_message(lin_vel_x, self.cur_angular_vel)
            self.xbox_pub.publish(msg)
            self.prev_lin = lin_vel_x
            self.prev_ang = self.cur_angular_vel

    def create_message(self, lin_vel_x, ang_vel_z):
        msg = Twist()

        msg.linear.x = lin_vel_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ang_vel_z 

        self.cur_angular_vel = ang_vel_z
        self.current_linear_vel = lin_vel_x

        return msg

    def map_range(self, joy_input) :
        return (joy_input - self.JOY_MIN) * (self.STEER_MAX - self.STEER_MIN) //  \
                (self.JOY_MAX - self.JOY_MIN) + self.STEER_MIN

   #Return steering angles (rads) based on linear regression lines
   #Derived from test data for estimating steering angle for MacNav 
    def get_steering_angle(self, servo_angle_input):
        if servo_angle_input > 38: #Steering right
            return (9.58E-3*servo_angle_input - 0.349) * (-1)
        else: #Steering left
            return -8.02E-3*servo_angle_input + 0.356

def main(args=None):
    rclpy.init(args=args)
    contr_node = Xbox360_contr_node()
    rclpy.spin(contr_node)
    contr_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()