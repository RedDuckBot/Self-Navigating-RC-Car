import rclpy, serial, random, time
from rclpy.node import Node 
from mac_messages.msg import Sensors

#sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=5)

class Sensor_test_node(Node):
    def __init__(self):
        super().__init__("sensor_pub_node")
        self.publisher_ = self.create_publisher(Sensors, 'sensor_test', 20)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Arduino guy sensor node initialized")
        self.count = 0

    def timer_callback(self):
        #if sensor_arduino.is_open: 
        time.sleep(random.choice(self.numbers))
        try:
            #data = sensor_arduino.readline().decode().strip()
            msg = Sensors()
            #msg.compass_heading = data
            msg.num = self.count 
            self.publisher_.publish(msg)
            self.count += 1
        except:
            self.get_logger().info("Failed to get msg")

def main():
    rclpy.init()
    sensor_node = Sensor_test_node()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    #sensor_arduino.close()