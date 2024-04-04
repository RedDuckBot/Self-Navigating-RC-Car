#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class localization_node(Node):
    
    def __init__(self):
        super().__init__("localization")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.odom_sub = self.create_subscription(Odometry, "/odom", None, 10)
        self.local_map_sub = self.create_subscription(OccupancyGrid, "/local_map", None, 10)
    
    def on_local_map(self, msg):
        w = msg.info.width
        h = msg.info.height
        resolution = msg.info.resoultion
        orgin_x = msg.info.origin.x
        orgin_y = msg.info.origin.y

        data1d = np.array(msg.data)
        local_map = np.reshape(data1d, (h,w))
        print(local_map)
        

        
