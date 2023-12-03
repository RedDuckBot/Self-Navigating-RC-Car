#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy as np
from PIL import Image, ImageFilter
import math
from functools import partial

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

GAUSSIAN_BLUR_RADIUS = 2 

SOBEL_FILTER_X = [
    [-1, 0, 1],
    [-2, 0, 2],
    [-1, 0, 1]
]

SOBEL_FILTER_Y = [
    [-1, -2, -1],
    [-0,  0,  0],
    [ 1,  2,  1]
]


class localization_node(Node):
    global_map_metadata = None
    local_map_metadata = None
    global_sobel_x = None
    global_sobel_y = None
    local_sobel_x = None
    local_sobel_y = None

    def __init__(self):
        super().__init__("localization_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.request_global_map()
        self.map_sub = self.create_subscription(OccupancyGrid, "/local_map", self.load_local_map, 10)

        

    def request_global_map(self):
        global_map_client = self.create_client(GetMap, "/map_server/map")
        while not global_map_client.wait_for_service(10.0):
            self.get_logger().info("Waiting for global map...")

        request = GetMap.Request()
        future = global_map_client.call_async(request)
        future.add_done_callback(partial(self.init_global_map))



    def lerp(self, value, min, max, newmin, newmax):
        olddif = max-min
        newdif = newmax - newmin
        return ((value - min) / olddif * newdif) + newmin


    def filterPixel(self, image, xy, filter):
        x, y = xy
        value = image.getpixel((x-1, y-1)) * filter[0][0] + \
                image.getpixel((x-1, y  )) * filter[1][0] + \
                image.getpixel((x-1, y+1)) * filter[2][0] + \
                image.getpixel((x,   y-1)) * filter[0][1] + \
                image.getpixel((x,   y  )) * filter[1][1] + \
                image.getpixel((x,   y+1)) * filter[2][1] + \
                image.getpixel((x+1, y-1)) * filter[0][2] + \
                image.getpixel((x+1, y  )) * filter[1][2] + \
                image.getpixel((x+1, y+1)) * filter[2][2]
        return int(self.lerp(value, -1020, 1020, 0, 254))



    def sobel(self, a):
        im = Image.fromarray(a, mode="L")
        im = im.filter(ImageFilter.GaussianBlur(2))
        filtered_x = Image.new("L", (im.width, im.height), color=0)
        filtered_y = Image.new("L", (im.width, im.height), color=0)

        for x in range(1, im.width-1):
            for y in range(1, im.height-1):
                filtered_x.putpixel((x,y), self.filterPixel(im, (x,y), SOBEL_FILTER_X))
                filtered_y.putpixel((x,y), self.filterPixel(im, (x,y), SOBEL_FILTER_Y))

        #filtered_x.save("imx.pgm")
        #filtered_y.save("imy.pgm")
        
        return filtered_x, filtered_y

        print("global map loaded")

        


    def init_global_map(self, future):
        try:
            r = future.result()
        except Exception as e:
            self.get_logger().error("Get global map failed: %r" % (e,))
            return
        self.global_map_metadata= r.map.info
        global_map = np.reshape(r.map.data, (r.map.info.height,r.map.info.width))

        self.global_sobel_x, self.global_sobel_y = self.sobel(global_map)

        
    def get_dir(self, x, y, x_img, y_img):
        gx = x_img.getpixel((x,y))
        gy = y_img.sobel_y.getpixel((x,y))

        gx = self.lerp(gx, 0, 254, -np.pi, np.pi)
        gy = self.lerp(gy, 0, 254, -np.pi, np.pi)

        if gx == 0:
            return np.pi/2
        return np.arctan(gy/gx)

    def get_mag(self, x, y, x_img, y_img):
        gx = x_img.getpixel((x,y))
        gy = y_img.getpixel((x,y))

        gx = self.lerp(gx, 0, 254, -np.pi, np.pi)
        gy = self.lerp(gy, 0, 254, -np.pi, np.pi)


        return np.sqrt(gx**2 + gy**2)

    def get_image_from_array(self, a, w, h):
        f = np.reshape(a, (h,w))
        f = np.where(f == -1, 50, f)
        im = Image.fromarray(f, mode="L")
        return im




    def load_local_map(self, msg):
        self.local_map_metadata = msg.info
        im = self.get_image_from_array(msg.data, msg.info.width, msg.info.height)
        self.local_sobel_x, self.local_sobel_y = self.sobel(im)


    def localize(self, msg):
        self.local_sobel_x, self.local_sobel_y = self.load_local_map(msg)
        #calc global -> local map tf

        #gen n particles within r of pose est
        #maxW = 0
        #for particle
        #   w = 0
        #   for pixel in local map    
        #       q = 1 - (2/pi)*abs( atan( sin( getdir(local) - getdir(global)) / cos( getdir(local) - getdir(global)))
        #       if q>0
        #           w += q * mag(global) / d base->pixel
        #   if w > maxW
        #       maxW = w
        #       best particle = this particle
        #
        #pose est = best particle  






def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()