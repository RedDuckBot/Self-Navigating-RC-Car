import serial, signal, threading

"""Program can be used to test accuracy of sensors by comparing printed data
   coming from sesnor arduino
"""
sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=5)

is_done = False
lock = threading.Lock()

def handler(num, frame):
    global is_done
    lock.acquire()
    is_done = True
    lock.release()

def main():
    signal.signal(signal.SIGINT,handler) #When done test press ctrl + c
    is_reading = True
    while is_reading:
        data = float(sensor_arduino.readline().decode().strip())
        lock.acquire()
        if is_done == True:
            is_reading = False
        lock.release()
        print("%.10f" % data)
    #print(f"total distance: {dist}")
    sensor_arduino.close()

if __name__ == "__main__":
    main()
