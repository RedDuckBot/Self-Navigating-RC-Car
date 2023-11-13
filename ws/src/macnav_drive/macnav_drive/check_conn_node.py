import socket, threading, signal

done_listening = False
lock = threading.Lock()

host = ""
port = 8080

def handler():
    global done_listening

    lock.acquire()
    done_listening = True
    lock.release()

class Client_conn_node(Node):
    def __init__(self):
        super().__init__("connection_node")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel',1)
        self.get_logger().info("Connection node initialized")

    def publish_MSG():
        pass

def check_connection(socket_obj):
    global done_listening
    try:
        # Try to send a small amount of data to the server
        socket_obj.send(b'ping')
        return True  # Connection is still alive
    except socket.error:
        lock.acquire()
        done_listening = False
        lock.release()
        return False  # Connection is lost


def main():
    signal.signal(signal.SIGKILL,handler)

    #Initialize connection and node
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    rclpy.init()
    conn_node = Client_conn_node()
    still_listening = True

    while still_listening: 
        try:
            # Check the connection
            if check_connection(client_socket):
                print("Connection is alive")
            else:
                print("Connection is lost")
        except:
            # Close the socket
            still_listening = False
        lock.acquire()
        if done_listening == False:
            still_listening = False
        lock.release()
    
    conn_node.publish_MSG() #Publish a message to halt drive system

    #Clean up resources
    client_socket.close()
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
