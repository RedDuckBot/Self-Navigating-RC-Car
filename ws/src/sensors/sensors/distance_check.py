import serial, signal, threading

sensor_arduino = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=5)

is_done = False
lock = threading.Lock()

def handler(num, frame):
    global is_done
    lock.acquire()
    is_done = True
    lock.release()

def main():
    global total_distance
    signal.signal(signal.SIGINT,handler)
    is_reading = True
    while is_reading:
        dist = float(sensor_arduino.readline().decode().strip())
        lock.acquire()
        if is_done == True:
            is_reading = False
        lock.release()
        print("%.10f" % dist)
    print(f"total distance: {dist}")
    sensor_arduino.close()

if __name__ == "__main__":
    main()
