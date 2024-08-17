The goal of this project is to achieve controlled drone flight in GPS-denied environments by utilizing visual-inertial odometry (VIO) data from an Intel 
RealSense T265 camera. Typically, in order to fly and move a drone contollabely there is a need to close a control loop based on the location of the drone,
often done by the GPS signal. In the absence of GPS signal, other methods for localization must be applied. The one used in this project as mentioned is odometry data 
from theT265 Intel RealSense camera. The odometry data is transformed to odometry data that can integrate with the flight controller's firmware using PX4's 
uXRCE-DDS middleware. This setup involves a client running on the flight controller and an agent on the companion computer, enabling bi-directional data exchange 
over a serial link. The agent functions as a proxy, allowing the client to publish and subscribe to topics within the global DDS data space. Odometry data in 
ROS2's nav_msgs format is translated to VehicleOdometry messages in the uORB format, more specifically on the /fmu/in/vehicle_visual_odometry topic. This data is 
then published at 30 Hz to prevent overloading the flight controller, which could cause it to freeze.

Our setup:
  Ubuntu 20.04 
  ROS2 Foxy
  librealsense v.2.50.0 -  https://github.com/IntelRealSense/librealsense/blob/v2.50.0/doc/distribution_linux.md (Option 1: Install librealsense2 debian package)
  realsense-ros version 4.0.4 - https://github.com/IntelRealSense/realsense-ros/tree/4.0.4

After sucsesfully initzalizing the camera runing in the termianl - ros2 launch realsense2_camera rs_launch.py 

We can continue to download on the companion computer the PX4-Autopilot firmeware. 
