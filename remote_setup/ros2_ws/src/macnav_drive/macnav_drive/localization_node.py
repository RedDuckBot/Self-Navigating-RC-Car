#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from scipy.spatial.transform import Rotation
import math

import cv2
import numpy as np

class localization_node(Node):
    
    initialized = False
    pose = None
    wrote = False

    def __init__(self):
        super().__init__("localization")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)


        self.get_global_map() 

        self.initial_posee_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.set_pose_listener, 2)
        self.local_map_sub = self.create_subscription(OccupancyGrid, "/local_map", self.on_local_map, 10)



        self.get_logger().info("finished init")


    def scale_array(self, arr, min_val, max_val):
        # Get the minimum and maximum values of the input array
        arr_min = np.min(arr)
        arr_max = np.max(arr)
        
        # Scale the values to the desired range
        scaled_arr = (arr - arr_min) * (max_val - min_val) / (arr_max - arr_min) + min_val
        
        return scaled_arr

    def get_transform(self, parent_frame_id, child_frame_id):
        try:
            # Get the latest transform from parent_frame_id to child_frame_id
            transform = self.tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    # walls = 0, open space = 100   
    def setup_map(self, im):
        imx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=5)
        imy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=5)
        mag = np.sqrt(imx**2 + imy**2)
        temp = np.ones(im.shape)
        temp[imx == 0] = np.nan
        angle = np.where(temp == 1, np.arctan2(imy, imx), temp)
        return mag, angle


    def calc_weight(self, global_angle, local_angle, local_mag):
        h,w = local_angle.shape
        xx,yy = np.meshgrid(np.arange(w), np.arange(h))
        m = (int(w/2),int(h/2))
        d = np.sqrt((xx - m[0])**2 + (yy - m[1])**2)        
        dx = m[0] - xx
        dy = m[1] - yy
        cv2.imwrite("dx.png", dx)
        angs = np.arctan2(dy,dx)
        angs[local_mag == 0] = np.nan
        #calc q
        weights = 1 - (2 * np.abs(np.arctan(np.sin(angs - global_angle) / np.cos(angs - global_angle))) / np.pi)
        # calc w
        weights[weights<0] = 0
        weights = weights * local_mag 
        weights = weights / d
        return np.nansum(weights)



    def get_global_map(self):
        global_map_client = self.create_client(GetMap, '/map_server/map')
        while not global_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for map server...")

        request = GetMap.Request()
        future = global_map_client.call_async(request)
        future.add_done_callback(self.on_get_global_map)

    def on_get_global_map(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(f"failed to get global map: {e}")
        else:
            w = response.map.info.width
            h = response.map.info.height
            self.resolution = response.map.info.resolution

            global_map = np.array(response.map.data)
            global_map = np.reshape(global_map, (h,w))

            global_map = np.flip(global_map, axis=0)

            global_map[global_map == 100] = -1
            global_map[global_map == 0] = 100
            global_map[global_map == -1] = 1 
            global_map = global_map.astype(np.uint8)
            self.global_angs = []
            for i in range(5,0,-1):
                global_map_blur = cv2.GaussianBlur(global_map, (61,61), i)
                mag, ang = self.setup_map(global_map_blur)
                cv2.imwrite(f"tmp{5-i}.png", (ang +3.15) *10)
                self.global_angs.append(ang)

            self.get_logger().info("Global map processed")



    


    # params:
    # global_map - matlike, represents the global map
    # local_map - matlike, represents the local map
    # pose_est - (x, y, r), position and rotation of pose estimate
    # rotation_sigma - tightness of rotation estimate in degrees
    # distribution_sigma - tightness of point estimation distribution in pixels
    # num_points - number of points to generate
    # num_iter - number of iterations to resample
    def localize(self,local_mag, local_ang, pose,
                 rotation_sigma=0.03,
                 distribution_sigma=1, 
                 num_points=100, 
                 num_iter=3, 
                 c=0.5, 
                 it=2):
        
        h,w = local_ang.shape

        points = np.random.multivariate_normal([0,0], [[distribution_sigma,0],[0,distribution_sigma]], num_points)
        angles = np.random.normal(0, rotation_sigma, num_points)
        particles = np.hstack((points, angles.reshape(-1,1)))
        particles = np.vstack((particles, np.array([0,0,0])))
        pose_pix_x = int(pose[0]/self.resolution)
        pose_pix_y = int(pose[1]/self.resolution)



        wrote=False 
        weights = []
        for particle in particles:
            rotation_matrix = cv2.getRotationMatrix2D((pose_pix_x, pose_pix_y), particle[2], 1)
            translation_matrix = np.array([[1, 0, particle[0]], [0, 1, particle[1]]])
            particle_ang = cv2.warpAffine(local_ang, rodtsfation_matrix, (w,h))
            particle_mag = cv2.warpAffine(local_mag, rotation_matrix, (w,h))

            particle_ang = cv2.warpAffine(particle_ang, translation_matrix, (w,h))
            particle_mag = cv2.warpAffine(particle_mag, translation_matrix, (w,h))

            value_indices = np.argwhere(particle_mag>0)
            min_x = np.min(value_indices[:,0])
            min_y = np.min(value_indices[:,1])
            max_x = np.max(value_indices[:,0])
            max_y = np.max(value_indices[:,1])


            if not wrote:
                wrote = True
                temp = np.copy(self.global_angs[it])
                temp[min_x, min_y] = 255
                temp[min_x, max_y] = 255
                temp[max_x, min_y] = 255
                temp[max_x, max_y] = 255
                temp[int(min_x + (max_x-min_x)/2),int(min_y + (max_y-min_y)/2)] = 255
                temp = (temp+3.15) * 10
                cv2.imwrite("temp.png", temp)
            weights.append(self.calc_weight(self.global_angs[it][min_x:max_x, min_y:max_y], particle_ang[min_x:max_x, min_y:max_y], particle_mag[min_x:max_x, min_y:max_y]))
        index = weights.index(max(weights)) 
        pose_x = particles[index][0]*self.resolution
        pose_y = particles[index][1]*self.resolution
        pose_t = particles[index][2]
        
        return (pose_x, pose_y, pose_t)

        if it < num_iter:
            return self.localize(
                local_mag,
                local_ang,
                (pose_x, pose_y, pose_t),
                rotation_sigma=rotation_sigma*c,
                distribution_sigma=distribution_sigma*c,
                num_points=int(num_points*c),
                c=c,
                it=it+1
            )
        else:
            return (pose_x, pose_y, pose_t)

    def quaternion_to_euler(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return Quaternion(w=qw, x=qx, y=qy, z=qz)

    def set_pose_listener(self, msg):
        tf_local_base = self.tf_buffer.lookup_transform("base_link", "local_map", rclpy.time.Time())

        tf_global_base = TransformStamped()
        tf_global_base.header.stamp = self.get_clock().now().to_msg()
        tf_global_base.header.frame_id = "map"
        tf_global_base.child_frame_id = "base_link"
        tf_global_base.transform.translation.x = msg.pose.pose.position.x
        tf_global_base.transform.translation.y = msg.pose.pose.position.y
        tf_global_base.transform.translation.z = msg.pose.pose.position.z
        tf_global_base.transform.rotation = msg.pose.pose.orientation


        buf = tf2_ros.Buffer()
        buf.set_transform_static(tf_local_base, "")
        buf.set_transform_static(tf_global_base, "")
        tf_global_local = buf.lookup_transform("map", "local_map", rclpy.time.Time())
        tf_global_local.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(tf_global_local)
        self.tf_buffer.set_transform_static(tf_global_local, "")

        _, _, yaw = self.quaternion_to_euler(tf_global_base.transform.rotation)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.initialized = True


    def set_pose(self, pose):
        tf_local_base = self.tf_buffer.lookup_transform("base_link", "local_map", rclpy.time.Time())

        tf_global_base = TransformStamped()
        tf_global_base.header.stamp = self.get_clock().now().to_msg()
        tf_global_base.header.frame_id = "map"
        tf_global_base.child_frame_id = "base_link"
        tf_global_base.transform.translation.x = pose[0]
        tf_global_base.transform.translation.y = pose[1]
        tf_global_base.transform.translation.z = 0.0
        tf_global_base.transform.rotation = self.euler_to_quaternion(0,0,pose[2])

        buf = tf2_ros.Buffer()
        buf.set_transform_static(tf_local_base, "")
        buf.set_transform_static(tf_global_base, "")
        tf_global_local = buf.lookup_transform("map", "local_map", rclpy.time.Time())

        tf_global_local.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(tf_global_local)
        self.get_logger().info(f"Setting pose to: {pose}" )
        self.pose = pose




    def on_local_map(self, msg):
        if not self.initialized:
            return
        self.get_logger().info("got local map")
        w = msg.info.width
        h = msg.info.height
        resolution = msg.info.resolution


        data1d = np.array(msg.data)
        local_map = np.reshape(data1d, (h,w))
        local_map[local_map == -1] = 100


        gh,gw = self.global_angs[0].shape 

        local_pad = np.zeros((gh,gw))
        local_pad[0:h, 0:w] = local_map

        origin = msg.info.origin

        top_left = PointStamped()
        top_left.point.x = origin.position.x
        top_left.point.y = origin.position.y

        top_right = PointStamped()
        top_right.point.x = origin.position.x + w*resolution
        top_right.point.y = origin.position.y

        bot_left = PointStamped()
        bot_left.point.x = origin.position.x
        bot_left.point.y = origin.position.y + h*resolution

        
        tf_global_local = self.tf_buffer.lookup_transform("map", "local_map", rclpy.time.Time())

        top_left_global = tf2_geometry_msgs.do_transform_point(top_left, tf_global_local)
        top_right_global = tf2_geometry_msgs.do_transform_point(top_right, tf_global_local)
        bot_left_global = tf2_geometry_msgs.do_transform_point(bot_left, tf_global_local)


        top_left_global_pixel = gh - int(top_left_global.point.y/resolution), int(top_left_global.point.x/resolution)
        top_right_global_pixel = gh - int(top_right_global.point.y/resolution), int(top_right_global.point.x/resolution)
        bot_left_global_pixel = gh - int(bot_left_global.point.y/resolution), int(bot_left_global.point.x/resolution) 

        src_pts = np.array([[0,0],[w,0],[0,h]], dtype=np.float32)
        dst_pts = np.array([top_left_global_pixel, top_right_global_pixel, bot_left_global_pixel], dtype=np.float32)
        affine_tf = cv2.getAffineTransform(src_pts, dst_pts)
        local_pad = cv2.warpAffine(local_pad, affine_tf, (gh,gw))
        local_pad = cv2.rotate(local_pad, cv2.ROTATE_90_CLOCKWISE)
        local_pad = np.flip(local_pad, axis=1)

        mask = (local_pad != 0)
        mask2 = (local_pad == 0)
        local_pad[mask] = 0
        local_pad[mask2] = 100

        local_pad = cv2.GaussianBlur(local_pad, (21,21), 10)
        local_mag, local_ang = self.setup_map(local_pad.astype(np.uint8))

        tmp = (local_ang +3.15) * 10

        

        tf_base_local = self.tf_buffer.lookup_transform("local_map", "base_link", rclpy.time.Time())
        base_pose = PoseStamped()
        base_pose.header.frame_id = "base_link"

        local_pose = tf2_geometry_msgs.do_transform_pose_stamped(base_pose, tf_base_local)
        global_pose = tf2_geometry_msgs.do_transform_pose_stamped(local_pose, tf_global_local)
        _, _, t = self.quaternion_to_euler(global_pose.pose.orientation)
        pose = (global_pose.pose.position.x, global_pose.pose.position.y, t)

        pose_pix = self.localize(local_mag, local_ang, pose)
        px = pose_pix[0] + pose[0]
        py = pose_pix[1] + pose[1]
        pt = pose_pix[2] + pose[2]
        self.set_pose((px,py,pt))








        


def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
