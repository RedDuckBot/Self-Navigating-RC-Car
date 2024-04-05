#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import cv2
import numpy as np

class localization_node(Node):
    
    initialized = False
    pose = None

    def __init__(self):
        super().__init__("localization")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_global_map() 

        self.initial_posee_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.set_pose_listener, 2)
        self.local_map_sub = self.create_subscription(OccupancyGrid, "/local_map", self.on_local_map, 10)

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 2)


        self.get_logger().info("finished init")

    # walls = 0, open space = 100   
    def setup_map(self, im):
        imx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=5)
        imy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=5)
        mag = np.sqrt(imx**2 + imy**2)
        temp = np.ones(im.shape)
        temp[imx == 100] = np.nan
        angle = np.where(temp == 1, np.arctan2(imy, imx), temp)
        return mag, angle


    def calc_weight(self, global_angle, local_angle, local_mag, pose):
        # requires the local map to be rotated
        # global map should be padded with nans 
        end_row = pose[0] + local_mag.shape[0]
        end_col = pose[1] + local_mag.shape[1]
        # crop global map to size of new map
        new_map = np.copy(global_angle[pose[0]+max(local_angle.shape):end_row+max(local_angle.shape), 
                                       pose[1]+max(local_angle.shape):end_col+max(local_angle.shape)])
        # calc q at
        new_map = 1 - (2 * np.abs(np.arctan(np.sin( local_angle - new_map) / np.cos(local_angle - new_map))) / np.pi)
        # calc w
        new_map[new_map<0] = 0
        new_map = new_map * local_mag
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
            global_map[global_map == -1] = 100 # walls are -1 in the occupancy grid for some reason
            global_map = global_map.astype(np.uint8)
            print(f'global {np.unique(global_map)}')
            self.global_mag, self.global_ang = self.setup_map(global_map)
            self.get_logger().info("Global map processed")

    
    # params:
    # global_map - matlike, represents the global map
    # local_map - matlike, represents the local map
    # pose_est - (x, y, r), position and rotation of pose estimate
    # rotation_sigma - tightness of rotation estimate in degrees
    # distribution_sigma - tightness of point estimation distribution in pixels
    # num_points - number of points to generate
    # num_iter - number of iterations to resample
    def localize(self, global_ang, local_mag, local_ang, pose_est, 
                 rotation_sigma=3, gaussian_sigma=5, 
                 distribution_sigma=30, num_points=75, 
                 num_iter=3, iter_tightness_constraint=0.7):
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

    def quaternion_to_euler(self, x, y, z, w):
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


        return roll, pitch, yaw

    def set_pose_listener(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(quat.x,quat.y,quat.z,quat.w)
        print(f"setting pose to {x,y,yaw}")
        self.set_pose((x,y,yaw))
        self.initialized = True


    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion =TransformStamped()
        quaternion.transform.rotation.x = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        quaternion.transform.rotation.y = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        quaternion.transform.rotation.z = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        quaternion.transform.rotation.w = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return quaternion.transform.rotation

    def set_pose(self, pose):
        tfa = TransformStamped()

        print(pose)
        tfa.transform.translation.x = float(pose[0])
        tfa.transform.translation.y = float(pose[1])
        tfa.transform.translation.z = 0.0
        tfa.header.frame_id = "map"
        tfa.child_frame_id = "local_map_center"

        tfb = TransformStamped()
        tfb.header.frame_id = "local_map_center"
        tfb.child_frame_id = "local_map"
        tfb.transform.rotation = self.euler_to_quaternion(0,0,pose[2])
    
        msg = TFMessage()
        msg.transforms = [tfa, tfb]

        self.pose = pose
        self.tf_pub.publish(msg)

    def on_local_map(self, msg):
        if not self.initialized:
            return
        self.get_logger().info("got local map")
        w = msg.info.width
        h = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        

        data1d = np.array(msg.data)
        local_map = np.reshape(data1d, (h,w))
        local_map = local_map.astype(np.uint8)
        space_mask = (local_map == 0)
        walls_mask = (local_map == 100) | (local_map == 255)
        local_map[space_mask] = 100
        local_map[walls_mask] = 0
        local_map = local_map.astype(np.uint8)
        local_mag, local_ang = self.setup_map(local_map)

        print(f"x y {origin_x, origin_y}")


        #best_point, _ = self.localize(self.global_ang, local_mag, local_ang, ((100,100),0))
        #best_point[0:2] = best_point[0:2] * resolution
        #print(f'best point: {best_point}\nresolution: {resolution}')
        #self.set_pose(best_point)


        


def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
