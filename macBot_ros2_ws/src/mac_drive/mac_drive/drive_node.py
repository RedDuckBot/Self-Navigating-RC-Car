from rclpy.node import Node 
from geometry_msgs.msg import Twist
from gpiozero import Servo 
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import DigitalOutputDevice, PWMOutputDevice
import time, math, rclpy


factory = PiGPIOFactory()

#Class for operating MacNav's steering servo
class Steer_Servo():
    MAX_PULSE_WIDTH = 1.760 #ms, 76 degrees turning right (Arduino angle servo input)
    MIN_PULSE_WIDTH = 0.980 #ms, 0 degrees turning left
    MID_PULSE_WIDTH = 1.360 #ms, 38 degrees center
    STEER_PIN = 13
    WHEEL_BASE = 0.265 #meters
    
    #Servo values related to GPIO Zero
    MAX_ANGLE = 1
    MIN_ANGLE = -1
    ANGLE_RANGE = 2

    def __init__(self):
        self.servo = Servo(pin=self.STEER_PIN,min_pulse_width=self.MIN_PULSE_WIDTH/1000,
                    max_pulse_width=self.MAX_PULSE_WIDTH/1000,pin_factory=factory)
    
    #Purpose: Perform steering for Ackerman Vehicle
    def steer(self, ang_vel, lin_vel):

        if 0 < ang_vel < 0.1: #Without this check the bot will turn right instead of left
            ang_vel = 0.1
        if -0.05 < ang_vel < 0: #Similarily, bot turns right instead of left
            ang_vel = -0.05

        if ang_vel == 0.0: #Center wheels
            self.servo.mid()
        else:
            steering_angle = math.atan(self.WHEEL_BASE * (ang_vel / lin_vel))
            if ang_vel > 0:
                arduino_input = self._get_servo_left_input(steering_angle)
            else:
                arduino_input = self._get_servo_right_input(steering_angle * -1)

            #Map arduino input to GPIO Zero input
            #i.e. working from scale [0,76] --> [-1,1]
            k = arduino_input / 76
            servo_input = self.ANGLE_RANGE * k + self.MIN_ANGLE
            if servo_input < -1: servo_input = -1
            if servo_input > 1: servo_input = 1
            self.servo.value = servo_input
    
    #Purpose: Getting steer left servo input was derived from steering test data for Arduino
    #Parameters: steer angle in radians for Ackerman Vehicle
    #Returns: Arduino servo angle in degrees 
    def _get_servo_left_input(self, steering_angle):
        angle = (int) ((steering_angle - 0.356) / (0.00802 * -1))
        if (angle < 0): angle = 0
        return angle

    #Purpose: Getting steer right servo input was derived from steering test data for Arduino
    #Parameters: steer angle in radians for Ackerman Vehicle
    #Returns: Arduino servo angle in degrees 
    def _get_servo_right_input(self, steering_angle):
        angle = (int) ((steering_angle + 0.349) / 0.00958)
        if (angle > 76): angle = 76
        return angle
  
#Class for operating MacNav's drive motor
class Motor():
    #L298N pins     
    enA_pin = 12 #pwm pin
    in1_pin = 5 #digital pins
    in2_pin = 6
    WHEEL_BASE = 0.265 #meters

    #PWM Range for GPIO Zero Library
    MIN_PWM_GPZ = 0.78
    MAX_PWM_GPZ = 1
    GPZ_RANGE = 0.22 

    #PWM Range for Arduino
    MIN_PWM_ARD = 200
    MAX_PWM_ARD = 255
    ARD_RANGE = 55

    def __init__(self):
        #Setup L298N controller pins
        self.enA = PWMOutputDevice(pin=self.enA_pin,frequency=500,pin_factory=factory)
        self.in1 = DigitalOutputDevice(self.in1_pin,pin_factory=factory)
        self.in2 = DigitalOutputDevice(self.in2_pin,pin_factory=factory)
    
    #Purpose: Control drive motor
    #Parameters: vel is linear velocity (m/s)
    #Post: PWM signal sent to enA pin 
    def drive(self, vel):
        pwm_input = self._get_motor_input(vel)   
        if (vel > 0): #Drive forward
            self.in1.on()
            self.in2.off()
            self.enA.value = pwm_input #speed input
        else: #Drive backward
            self.in1.off()
            self.in2.on()
            self.enA.value = pwm_input #speed input

    def brake(self):
        self.in1.off()
        self.in2.off()
        time.sleep(0.100)
            
    #Purpose: Get the correpsonding PWM signal from linear velocity. 
    #Parameters: vel is linear velocity
    #Return: PWM signal for drive motor
    def _get_motor_input(self,vel):
        arduino_pwm = self._vel_to_Arduino_PWM(vel) 
        
        #Map arduino_pwm value to this class's PWM range
        k = (arduino_pwm - self.MIN_PWM_ARD) / self.ARD_RANGE  
        motor_input = k * self.GPZ_RANGE + self.MIN_PWM_GPZ

        return motor_input
    
    #Purpose: Originally, the PWM values for the motor was calculated for 
    #an Arduino script, however, the PWM scale for Arduino is different than
    #the PWM scale used from the GPIO Zero library. This function gets
    #the PWM value related Arduino script.
    def _vel_to_Arduino_PWM(self,lin_velocity):
        if (lin_velocity < 0): lin_velocity = lin_velocity * (-1)
        pwm_input = (int) ((lin_velocity + 1.46) / 0.0105)
        if (pwm_input < 200): pwm_input = 200
        if (pwm_input > 255): pwm_input = 255

        return pwm_input

    #Purpose: Close all pins
    def shutdown(self):
        self.enA.close()
        self.in1.close()
        self.in2.close()

class Drive_Node(Node):
    def __init__(self):
        super().__init__("drive")
        self. prev_ang = 0.0
        self.prev_lin = 0.0
        self.sub_drive_node = self.create_subscription(Twist, "/cmd_vel",
                                    self.operate_actuators,5)
        self.steer_servo = Steer_Servo()
        self.motor = Motor()
        self.get_logger().info("Initialized driving node")

    def operate_actuators(self, msg):
        if msg.angular.z != self.prev_ang or msg.linear.x != self.prev_lin:
            if msg.linear.x == 0.0:
                self.motor.brake()
            else:
                self.motor.drive(msg.linear.x)
                self.steer_servo.steer(msg.angular.z,msg.linear.x)
            self.prev_ang = msg.angular.z
            self.prev_lin = msg.linear.x

def main(args=None):
    rclpy.init(args=args)
    drive_node = Drive_Node()
    rclpy.spin(drive_node)
    drive_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
