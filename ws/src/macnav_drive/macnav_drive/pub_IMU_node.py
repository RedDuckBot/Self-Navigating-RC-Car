
#!/usr/bin/env python3
import rclpy, serial, math
from rclpy.node import Node
import numpy as np
from mac_messages.msg import Imu

"""IMU node receives data from imu as heading values on one second intervals"""

imu_arduino = serial.Serial(port="/dev/ttyACM1",baudrate=115200)

class IMU_Node(Node):
    TIMER_INTERVAL = 0.5

    def __init__(self):
        super().__init__('imu_publisher')

        self.tmr = self.create_timer(self.TIMER_INTERVAL, self.publish)
        self.IMU_publisher = self.create_publisher(Imu, '/imu',5)
        self.get_logger().info("Arduino IMU node initialized")

    def publish(self):
        msg = Imu()
        try:
            heading_bytes = imu_arduino.read_until()
            heading_bytes = heading_bytes[:4]
            new_heading = float((np.frombuffer(heading_bytes,dtype=np.float32))[0])
        except:
            new_heading = 0.0
        #self.get_logger().info(f"{new_heading}")
        msg.heading = new_heading 
        self.IMU_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_Node()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    imu_arduino.close()