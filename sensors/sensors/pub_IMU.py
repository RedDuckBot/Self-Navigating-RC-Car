import rclpy, serial, ahrs, threading, queue
from rclpy.node import Node 
from mac_messages.msg import Sensors
import numpy as np

sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=10)
is_listening = True

IMU_data = queue.Queue(15) #Queue of IMU_Dataclass objects
lock = threading.Lock()

def listen_serial_conn():
    done = False

    while not done:
        if sensor_arduino.is_open: 
            data_ready = False
            temp_IMU = IMU_Dataclass()
            try:
                for i in ["ACCEL","GYRO"]: #Get each type of IMU data
                    temp_imu_data = []
                    for i in range(3): #Get all three axis values
                        data_received = sensor_arduino.readline().decode().strip()
                        temp_imu_data.append(float(data_received)) 
                    if i == "ACCEL": #Accelerometer data
                        temp_IMU.accel_np = np.array(temp_imu_data)  
                    else: #Gyro data
                        temp_IMU.gyro_np = np.array(temp_imu_data)  
                data_ready = True
            except:
                self.get_logger().info("Failed to get data")
            if data_ready = True:
                IMU_data.put(temp_IMU,block=False)
            
        else:
            done = True

        lock.acquire()
        if is_listening = False:
            done = True
        lock.release()

    print("Closing listening thread.")

class IMU_Dataclass:
    def __init__(self):
        self.accel_np = None
        self.gyro_np = None

class Sensor_pub_node(Node):
    def __init__(self):
        super().__init__("sensor_pub_node")
        self.publisher_IMU = self.create_publisher(Sensors, 'sensor_test', 5)
        timer_period = 1 
        self.timer = self.create_timer(timer_period, self.callback)
        self.get_logger().info("Arduino sensor node initialized")

    def callback(self):
        pass
            

def main():
    rclpy.init()
    sensor_node = Sensor_pub_node()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()
    lock.acquire()
    is_listening = False
    lock.release()

if __name__ == "__main__":
    main()
    sensor_arduino.close()