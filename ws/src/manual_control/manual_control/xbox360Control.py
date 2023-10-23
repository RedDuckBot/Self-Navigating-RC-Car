#!usr/bin/python
import signal, serial, time
from xbox360controller import Xbox360Controller

"""Not a Node script rather just a script that passes xbox input to Arduino"""

arduino = serial.Serial(port="/dev/ttyACM0",baudrate=115200)

speeds = [0,100,190,255]
speedIndex = 1

drive_mode_isEnabled = False

#Callback function associated with drive_mode change
def on_drive_mode(button):
    global drive_mode_isEnabled, speedIndex

    if (not drive_mode_isEnabled):
        drive_mode_isEnabled = True
        print("Drive mode Enabled")
        arduino.write("E".encode())
    else:
        drive_mode_isEnabled = False
        print("Drive mode Disabled")
        speedIndex = 1
        arduino.write("D".encode())

#Callback function associated with steering servo angle
def on_axis_moved(axis):

    if drive_mode_isEnabled: 
        arduino.write("S".encode())
        arduino.flush()
        #Check if joystick is not at neutral
        if not (axis.x > -0.25 and axis.x < 0.25):
            #driveMSG.drive_message["servo"]["angle"] = axis.x
            arduino.write(f"{int(axis.x * 100)}".encode())
        else:
            arduino.write("0".encode())
        arduino.flush()

#Callback function associated with shifting speed
def on_speed(button):
    global speedIndex

    if drive_mode_isEnabled:
        arduino.write("M".encode())
        arduino.flush()
        arduino.write(f"{speeds[speedIndex]}".encode())
        arduino.flush()
        speedIndex = (1 + speedIndex) % 4
        print(f"Speed shifted {speeds[speedIndex - 1]}")

def main():
    try:
        with Xbox360Controller(0,axis_threshold=0) as controller:
            #Drive event
            controller.button_a.when_pressed = on_drive_mode

            #Actuator events
            controller.axis_r.when_moved = on_axis_moved 
            controller.button_x.when_pressed = on_speed

            signal.pause()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    arduino.close()
