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
from geometry_msgs.msg import PoseWithStamped
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
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
        temp[imx == 100] = np.nan
        angle = np.where(temp == 1, np.arctan2(imy, imx), temp)
        return mag, angle


    def calc_weight(self, global_angle, local_angle, local_mag, pose, origin):
        # requires the local map to be rotated
        # global map should be padded with nans 
        pad = max(local_angle.shape)
        start_row = global_angle.shape[1] - (origin[1] + pad)
        end_row = start_row + local_angle.shape[0]
        start_col = origin[0] + pad
        end_col = start_col + local_angle.shape[1]

        

        # crop global map to size of new map
        new_map = np.copy(global_angle[start_row:end_row, 
                                       start_col:end_col])
        # calc q at
        new_map = 1 - (2 * np.abs(np.arctan(np.sin( local_angle - new_map) / np.cos(local_angle - new_map))) / np.pi)
        # calc w
        
        new_map[new_map<0] = 0
        new_map = new_map * local_mag
        tmp = np.copy(global_angle)
        tmp[start_row:end_row, start_col:end_col] = local_angle
        if not self.wrote:
            cv2.imwrite("tmp.png", (tmp + 3.15) * 10)
            print("pose", pose)
            self.wrote = True
        return new_map



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

            global_map = np.array(response.map.data)
            global_map = np.reshape(global_map, (h,w))
            global_map = np.flip(global_map, axis=0)
            global_map[global_map > 0 ] = 0
            global_map[global_map == -1] = 100 
            global_map = global_map.astype(np.uint8)
            self.global_angs = []
            for i in range(5,0,-1):
                global_map_blur = cv2.GaussianBlur(global_map, (61,61), i)
                _, ang = self.setup_map(global_map_blur)
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
    def localize(self,local_mag, local_ang, pose_est, origin, 
                 rotation_sigma=0.03,
                 distribution_sigma=50, 
                 num_points=100, 
                 num_iter=3, 
                 iter_tightness_constraint=0.5, 
                 it=0):
        
           
        global_ang = np.pad(self.global_angs[it], pad_width=max(local_ang.shape), constant_values=np.nan)
        points = np.random.multivariate_normal(pose_est[0], [[distribution_sigma, 0],[0, distribution_sigma]], num_points)
        points = np.vstack((pose_est[0], points))
        angles = np.random.normal(pose_est[1], rotation_sigma, num_points)
        angles = np.hstack((pose_est[1], angles))
        points = np.concatenate((points, angles[:, np.newaxis]), axis=1)
        
        h,w = local_ang.shape
        weights = []
        self.wrote = False
        for point in points:
            rotation_matrix = cv2.getRotationMatrix2D((w//2,h//2), point[2], 1)
            rotated_ang = cv2.warpAffine(local_ang, rotation_matrix, (w,h), borderValue=np.nan)
            rotated_mag = cv2.warpAffine(local_mag, rotation_matrix, (w,h), borderValue=0)
            x = int(point[0])
            y = int(point[1])
            weights.append(np.nansum(self.calc_weight(global_ang, rotated_ang, rotated_mag, (x,y), origin)))

        max_weight = max(weights)
        max_index = weights.index(max_weight)
        best_point = points[max_index]

        if it < num_iter:
            return self.localize(local_mag, local_ang,
                                 ((best_point[0], best_point[1]), best_point[2]), origin,
                                 rotation_sigma=rotation_sigma*iter_tightness_constraint,
                                 distribution_sigma=distribution_sigma*iter_tightness_constraint,
                                 num_points=num_points,
                                 num_iter=num_iter,
                                 iter_tightness_constraint=iter_tightness_constraint,
                                 it=it+1)
        else:
            return best_point



        ksize = int(gaussian_sigma*10)
        if ksize % 2 == 0: ksize+=1
        global_blur = cv2.GaussianBlur(global_ang, (ksize,ksize), gaussian_sigma, gaussian_sigma)
        global_ang = np.pad(global_blur, pad_width=max(local_ang.shape), constant_values=np.nan)

        points = np.random.multivariate_normal(pose_est[0], [[distribution_sigma, 0],[0, distribution_sigma]], num_points)

        angles = np.random.normal(pose_est[1], rotation_sigma, num_points)
        points = np.concatenate((points, angles[:, np.newaxis]), axis=1)
        

        h, w = local_ang.shape
        weights = []
        x = int(pose_est[0][0])
        y = int(pose_est[0][1])
        weights.append(np.nansum(self.calc_weight(global_ang, local_ang, local_mag, (x,y))))
        for point in points:
            rotation_matrix = cv2.getRotationMatrix2D((w/2,h/2), point[2], 1)
            rotated_ang = cv2.warpAffine(local_ang, rotation_matrix, (w,h), borderValue=np.nan)
            rotated_mag = cv2.warpAffine(local_mag, rotation_matrix, (w,h), borderValue=0)
            x = int(point[0])
            y = int(point[1])
            weights.append(np.nansum(self.calc_weight(global_ang, rotated_ang, rotated_mag, (x,y))))

        max_weight = max(weights)
        max_index = weights.index(max_weight)
        best_point = points[max_index]
        if num_iter > 1:
            p, w = self.localize(global_ang,
                            local_mag, 
                            local_ang,
                            ((best_point[0],best_point[1]),best_point[2]), 
                            rotation_sigma=rotation_sigma*iter_tightness_constraint, 
                            distribution_sigma=distribution_sigma*iter_tightness_constraint,
                            num_points=num_points,
                            num_iter=num_iter-1,
                            gaussian_sigma=gaussian_sigma*iter_tightness_constraint,
                            iter_tightness_constraint=iter_tightness_constraint)
            if w > max_weight:
                best_point = p

        return best_point, max(weights)

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

        self.tf_broadcaster.sendTransform(tf_global_local)

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

        self.tf_broadcaster.sendTransform(tf_global_local)
        self.pose = pose




    def on_local_map(self, msg):
        if not self.initialized:
            return
        self.get_logger().info("got local map")
        w = msg.info.width
        h = msg.info.height
        resolution = msg.info.resolution
        


        #self.set_pose(self.pose)


        data1d = np.array(msg.data)
        local_map = np.reshape(data1d, (h,w))
        local_map[local_map<0] = 0
        local_map[local_map == -1] = 100
        local_map = np.flip(local_map, axis=0)
        
        h,w = local_map.shape
        tf_global_local = self.tf_buffer.lookup_transform("map", "local_map", rclpy.time.Time())

        origin_pose = msg.info.origin.position
        origin_stamped = PoseStamped()
        orogin_stamped.header.frame
        origin_stamped.point = origin_pose
        origin_transformed = tf2_geometry_msgs.do_transform_pose_stamped(origin_stamped, tf_global_local)

        _, _, theta = self.quaternion_to_euler(tf_global_local.transform.rotation)
        theta = theta * (180/np.pi)
        rotation_matrix = cv2.getRotationMatrix2D((w//2,h//2), -theta, 1)
        local_map = cv2.warpAffine(local_map.astype(np.uint8), rotation_matrix, (w,h), borderValue=np.nan)
        local_map = cv2.GaussianBlur(local_map.astype(np.uint8), (5,5), 2)
        local_mag, local_ang = self.setup_map(local_map)


        pose = ((self.pose[0]/resolution, self.pose[1]/resolution), self.pose[2])
        origin = (int(origin_transformed.point.x/resolution), int(origin_transformed.point.y/resolution))
        print("origin",origin)
        best_point = self.localize(local_mag, local_ang, pose, origin)
        best_point[0:2] = best_point[0:2] * resolution
        print(f'best point: {best_point}')
        self.set_pose(best_point)


        


def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
