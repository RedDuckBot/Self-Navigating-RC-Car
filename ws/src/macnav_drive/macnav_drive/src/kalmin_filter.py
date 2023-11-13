#!/usr/bin/env python3

class KalmanFilter(object):

    def __init__(self):
        # Initially the robot has no idea about how fast is going
        self.mean_ = 0.0
        self.variance_ = 1000.0

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

        # Store the messages - only for the orientation
        self.imu_yaw_ = 0.0

        self.is_first_odom_ = True
        self.last_odom_yaw_ = 0.0
        self.motion_ = 0.0

    def filter_yaw(self, new_imu_yaw, new_odom_yaw):
        """Return the filtered yaw as a mean"""

        self.imu_yaw_ = new_imu_yaw
        if self.is_first_odom_:
            self.last_odom_yaw_ = new_odom_yaw 
            self.is_first_odom_ = False
            self.mean_ = new_odom_yaw
            return
        
        self.motion_ = new_odom_yaw - self.last_odom_yaw_

        self.statePrediction()
        self.measurementUpdate()

        # Update for the next iteration
        self.last_odom_yaw_ = new_odom_yaw 

        return self.mean_


    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_yaw_) \
                   / (self.variance_ + self.measurement_variance_)
                     
        self.variance_ = (self.variance_ * self.measurement_variance_) \
                       / (self.variance_ + self.measurement_variance_)


    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_