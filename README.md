# LIDAR, RC Car, Navigation Robot

A RC car turned into an autonomously navigating robot that was able to navigate around MacEwan University's buildings, hence, given the name MacNav. This undertaking was a capstone project, towards an undergraduate computer science degree, done at MacEwan University. The bot's mechanical composition is an amalgamation of original RC and 3D printed parts, with the latter parts consisiting of platforms and supports for the added hardware. The main hardware utilized was a Raspberry PI 4B (running Ubuntu Server 22.04), the device guiding and controlling the bot, and sensor devices involving a 360 degree, 2D LIDAR, IMU and motor encoder for localization. The middleware ROS2 Humble was used to build the robotic application as well as a ROS2 package Nav2, for navigation. The video bellow shows a sped up version of MacNav's trip, but here's a youtube link showing the entire trip: https://www.youtube.com/watch?v=KRr-bcZy1FA 

<p align="center">
  <img title='Navigating MacEwan Clip' src=docs/images/macnav_repo_clip.gif width="600">
</p>

<p align="center">
  <img title='MacNav' src=docs/images/macnav.png width="600">
</p>

## 🗃️ Package Overview
- [slam tool box](https://github.com/SteveMacenski/slam_toolbox): ROS2 Package used to generate a occupancy grid of robots environment and localize the bot within it by using algorithms like AMCL. The map was created using LIDAR data.
- [LIDAR](https://github.com/Slamtec/sllidar_ros2): ROS2 package made for SLAMTEC LIDAR products (this project used A1M8 LIDAR), which takes the sensors data and publishes it on the topic "/scan".
- [Nav2](https://github.com/ros-navigation/navigation2): ROS2 package representing a navigation framework.

## Project Overview 
<p align="center">
  <img title='MacNav Architecture' src=docs/images/macnav_architecture.png width="800">
</p>

The above image provides a highly abstract diagram for how auto-navigation was setup for MacNav. Each box demonstrates what ROS2 nodes were running on the device. In the case of the Raspberry PI, the bot was able to use it's own 'faculities' for navigating: sensor data was feed into SLAM Tool Box nodes for generating an occupancy grid of the bot's environmnet, and then the grid was passed into Nav2's navigation stack; subsequently, this stack would output kinematic values, the linear and angular velocity, that the controller node, who controls the steering servo and motor, utilizes to maintain the bot's course. The other case being a remote laptop running two nodes in a Docker container. One of the nodes being a visualization tool called RVIZ2 that showed a map of the bot's environment. Moreover, this tool was used to specify goal points of travel on the map. In the opening video of this README, the video's background was costmap, a map created in the navigation stack, that showed colored regions representing the degrees to which the space maybe occupated. Indeed, the map being a view of what MacNav is able to see in the environment. And, the other node representing the inputs for an Xbox 360 controller for manual control. This was needed to build a map of the indoor buildings by manually driving the bot around, given that the bot needed to be familiarized with it's environment before a journey could be specified.    

A more in depth explanation of MacNav's inner workings are discussed in a [`report`](./docs/macnav_report.pdf), such as academic research explored, circuit diagrams, powering hardware, the software navigation stack, networking setup, or the tests performed to understand the bot's kinematics, specifically, how data was used to model the relationships between actuator inputs and velocities. However, this report does not reflect the hardware changes made recently since at the time a Jetson Nano was used instead of a RPI 4, and there were three Arduinos used which have been narrowed down to one. Also, the report shows a dumn robot setup, meaning instead of running the SLAM nodes and the navigation stack on the bot it was running on the remote machine, thus, providing MacNav with course updates.  

Last remark on the report, there were mentions of short comings and future work. For example, one short coming noticeable in the full video was that the bot's forward motion was not continuous through out its journey, i.e. the bot frequently would stop and go despite being many meters behind waypoints. Likely, the cause being an unleveled LIDAR sensor that kept producing phantom walls ahead of its path, a consequence of a downward tilt of the sensor, but this was never conclusively confirmed. 
