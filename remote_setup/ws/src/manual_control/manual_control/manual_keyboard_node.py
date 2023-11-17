import rclpy
import os
import sys
import select

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

import termios
import tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def main(args=None):
    #settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node("keyboard_control")
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    speeds = [0.0, 170.0, 200.0, 225.0, 255.0]
    speeds_index = 0
    lin = 0.0
    ang = 0.0

    try:
        while(1):
            key = getKey()
            if key =='1':
                speeds_index = 0
            elif key =='2':
                speeds_index = 1
            elif key =='3':
                speeds_index = 2
            elif key =='4':
                speeds_index = 3
            elif key =='5':
                speeds_index = 4 
            elif key =='w':
                lin = speeds[speeds_index]
            elif key =='s':
                lin = 0.0
            elif key =='x':
                lin = -speeds[speeds_index]
            elif key =='d':
                if ang > -100.0:
                    ang -= 20.0
            elif key =='a':
                if ang < 100.0:
                    ang += 20.0
            else:
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = lin
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = ang

            pub.publish(twist)
        
    except Exception as e:
        print(e)
    
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

if __name__ == '__main__':
    main()