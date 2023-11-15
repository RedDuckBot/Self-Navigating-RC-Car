import serial

arduino_sensor = serial.Serial(port="/dev/ttyUSB0",baudrate=9600)
print("arduino sensor initialized")
while True:
    if arduino_sensor.is_open:
        msg = arduino_sensor.readline().decode().strip()
        print(msg + "  " )
    else: break

arduino_sensor.close()
print("arduino sensor closed")