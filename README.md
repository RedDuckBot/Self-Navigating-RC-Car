# LIDAR, RC Car, Navigation Robot

A RC car turned into an, autonomously, navigating robot that was able to navigate around MacEwan University's buildings, hence, given the name MacNav. This undertaking was a capstone project, towards an undergraduate computer science degree, done at MacEwan University. The bot's mechanical composition is an amalgamation of original RC and 3D printed parts, with the ladder parts consisiting of platforms and supports for the electronic hardware. The main hardware utilized was a Raspberry PI 4B+ (running Ubuntu Server 22.04), the device guiding and controlling the bot, and sensor devices like a 360 degree, 2D LIDAR, IMU and motor encoder for localization. The middleware ROS2 Humble was used to build the robotic application as well as a ROS2 package Nav2, for navigation.

<p align="center">
  <img title='Navigating MacEwan Clip' src=docs/images/macnav_repo_clip.gif width="600">
</p>

<p align="center">
  <img title='MacNav' src=docs/images/macnav.png width="600">
</p>