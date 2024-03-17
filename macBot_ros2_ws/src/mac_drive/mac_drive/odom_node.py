import rclpy, math, adafruit_bno055, board
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import smbus2, struct

SLAVE_ARDUINO = 0x8 #Arduino's address for encoder
i2c = board.I2C()
imu_sensor = adafruit_bno055.BNO055_I2C(i2c)
arduino_i2c_bus = smbus2.SMBus(1)

class OdomNode(Node):
    TIMER_INTERVAL = 0.1 
    GEAR_TRAIN_RATIO = 0.0042457542

    #Units in meters
    WHEEL_BASE = 0.265 
    WHEEL_RADIUS = 0.0402

    #Encoder info for MacNav's drive motor
    PPR = 11
    ENCODER_RESOLUTION = 2 * PPR #2 Channels in Use 

    def __init__(self):
        super().__init__('odom_publisher')
        self.current_linear_vel = 0.0 # m/s
        self.current_angular_vel = 0.0 # rads/s
        self.current_steering_angle = 0.0 #Radians, relative to an imaginary center wheel
        self.heading_x = 0.0 # meters
        self.heading_y = 0.0
        self.forwards = True
        self.vehicle_heading = 0.0 #rads 

        self.odom_publisher = self.create_publisher(Odometry, 'odom',10)
        self.tfpublisher = self.create_publisher(TFMessage, 'tf', 10)
        self.tmr = self.create_timer(self.TIMER_INTERVAL, self.odom_publish)
        self.dirve_sub = self.create_subscription(Twist,"/cmd_vel",
            self.update_kinematic_values,10)

        self.get_logger().info("Odom node initialized")

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def odom_publish(self):
        #Get encoder counts
        try:
            data = arduino_i2c_bus.read_i2c_block_data(SLAVE_ARDUINO,0,4)
            counts = int(struct.unpack('<f', bytes(data))[0])
        except:
            counts = 0

        #Update linear velocity 
        self.current_linear_vel = (float(counts) * self.GEAR_TRAIN_RATIO * \
            2 * math.pi * self.WHEEL_RADIUS) 
        if not self.forwards:
            self.current_linear_vel *= -1

        #Update vehicle heading
        heading = imu_sensor.euler[0] # range: 0 to 360 degrees
        if heading != None and heading > 0:
            self.vehicle_heading = (360 - heading) * (math.pi/180) 

        #Update vehicle position
        self.heading_x += self.current_linear_vel * math.cos( \
            self.vehicle_heading)
        self.heading_y += self.current_linear_vel * math.sin( \
                self.vehicle_heading)
        self.help_odom_publish()

    def help_odom_publish(self):
        """Utility function for odom_publish"""

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self.heading_x
        msg.pose.pose.position.y = self.heading_y
        msg.pose.pose.position.z = 0.0
        r = self.euler_to_quaternion([self.vehicle_heading, 0.0, 0.0])
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
        #Sent from Nav2
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
    
    def get_steer_angle(self, vel_x, ang_z):
        if vel_x == 0.0:
            return self.current_steering_angle 
        return math.atan((ang_z/vel_x) * self.WHEEL_BASE)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    lgpio.i2c_close(i2c_bus1)
