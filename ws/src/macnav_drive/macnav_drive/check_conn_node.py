import socket, threading, signal,time
from mac_messages.msg import Drive
import rclpy
from rclpy.node import Node

"""
Node checks for the tcp connection between MacNav and remote machine, constantly.
In the event a connection is lost a message is published to the drive arduino
to stop moving.
"""

done_listening = False
lock = threading.Lock()

host = ""
port = 6670

def handler(signum, frame):
    #Handle user press ctrl + c
    #Function notifies main that connection between MacNav and remote will 
    #be no in longer use 

    global done_listening

    lock.acquire()
    done_listening = True
    lock.release()

def check_connection(socket_obj):
    global done_listening
    try:
        # Try to send a small amount of data to the server
        socket_obj.send(b'ping')
        time.sleep(1)
        return True  # Connection is still alive
    except socket.error:
        lock.acquire()
        done_listening = False
        lock.release()
        return False  # Connection is lost

class Client_conn_node(Node):
    def __init__(self):
        super().__init__("connection_node")
        self.publisher_ = self.create_publisher(Drive, '/arduino_channel',1)
        self.get_logger().info("Connection node initialized")

    def publish_MSG(self):
        msg = Drive()
        msg.command = "D"
        self.publisher_.publish(msg)

def main():
    signal.signal(signal.SIGINT,handler)

    #Initialize connection and node
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    rclpy.init()
    conn_node = Client_conn_node() #connect to remote machine
    still_listening = True

    while still_listening: 
        try:
            # Check the connection
            if check_connection(client_socket):
                print("Connection is alive")
            else:
                print("Connection is lost")
                still_listening = False
        except:
            still_listening = False
        lock.acquire()
        if done_listening == True:
            still_listening = False
        lock.release()
    
    conn_node.publish_MSG() #Publish a message to halt drive system

    #Clean up resources
    client_socket.close()
    conn_node.destroy_node()
    rclpy.shutdown()
    print("Closing MacNav's connection to remote machine")

if __name__ == "__main__":
    main()
