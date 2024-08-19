The goal of this project is to achieve controlled drone flight in GPS-denied environments by utilizing visual-inertial odometry (VIO) data from an Intel 
RealSense T265 camera. Typically, in order to fly and move a drone controllably there is a need to close a control loop based on the location of the drone,
often done by the GPS signal. In the absence of GPS signal, other methods for localization must be applied. The one used in this project as mentioned is odometry data 
from the Intel RealSense T265 camera. The odometry data is transformed to odometry data that can integrate with the flight controller's firmware using PX4's 
uXRCE-DDS middleware. This setup involves a client running on the flight controller and an agent on the companion computer, enabling bi-directional data exchange 
over a serial link. The agent functions as a proxy, allowing the client to publish and subscribe to topics within the global DDS data space. Odometry data in 
ROS2's nav_msgs format is translated to VehicleOdometry messages in the uORB format, more specifically on the /fmu/in/vehicle_visual_odometry topic. This data is 
then published at 30 Hz to prevent overloading the flight controller, which could cause it to freeze.

## **Our setup:**

- Ubuntu 20.04 - https://releases.ubuntu.com/focal/

- ROS2 Foxy - https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html

- librealsense v.2.50.0 -  https://github.com/IntelRealSense/librealsense/blob/v2.50.0/doc/distribution_linux.md 
(Option 1: Install librealsense2 debian package)

- realsense-ros version 4.0.4 - https://github.com/IntelRealSense/realsense-ros/tree/4.0.4

- PX4-Autopilot v.1.14 - https://docs.px4.io/main/en/ros2/user_guide.html#install-px4

### **Install PX4-Autopilot v.1.14** 

Set up the PX4 development environment:

    cd
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    install some python dependencies - pip install --user -U empy==3.3.4 pyros-genmsg setuptools
  
### **setup of the agent**
Build the agent from source:

    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/

**building the worksapce:**

    mkdir -p ~/ros2_ws/src/
    cd ~/ros2_ws/src/
    git clone https://github.com/PX4/px4_msgs.git
    git clone https://github.com/PX4/px4_ros_com.git
    cd ..
    source /opt/ros/foxy/setup.bash
    colcon build

### **Cloning this repository**
We now need to clone this repository to our ROS2 workspace by following this commands in the termianl.

    cd /ros2_ws/src
    https://github.com/bute88/t265_px4.git


### Downloading QgroundControl daily build - linux
Before installing QgroundControl insret the next commands in the command prompt:

    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libfuse2 -y
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

Download QgroundControl daily build from here - https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html
Werever you downloaded QgroundControl. Let's assume in the Download files, we will go to tat file and insert this command lines.

    cd /Downloads
    chmod +x ./QGroundControl.AppImage
    ./QGroundControl.AppImage  #(or double click)

Now we need to setup our vehicle, you can follow the guide - https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html and finally tune the parameters for VIO and disabling GPS as follows.

Set `EKF2_EV_CTRL` - enable: 0 : Horizontal position fusion 1 : Vertical position fusion 2 : 3D velocity fusion 3 : Yaw

Set `UXRCE_DDS_CFG` to TELEM2

Set `SER_TEL2_BAUD` to TELEM2

Disable  `MAV_0_CONFIG=0` or `MAV_1_CONFIG=0` 

`EKF2_GPS_CTRL = 0` (no GPS fusion)

`EKF2_HGT_REF = 3` (height reference coming from EV)

`EKF2_MAG_TYPE = 5` (no magnetometer fusion)  **need to be tested**

for more information in here - https://docs.px4.io/main/en/middleware/uxrce_dds.html (under Starting the Client)

### Communication
Now we need to handle the communication between the LattePanda and the Pixhawk using FTDI adapter from USB to serial UART on TEME2 port
on the pixhawk.

we need to set the usb port with udev-rules as follows:
inside the file `60-Ftdi.rules` for instance under `/etc/udev/rules.d` (it can be another file other then `60-Ftdi.rules`)

Iside that file configure the parameters for instance `KERNEL=="ttyUSB*", ATTRS{idVendor}=="0405", ATTRS{idProduct}=="8005", ATTRS{serial}=="A30472BI", SYMLINK+="telem2"`

##**Initializing the Agent and our Transformation Node**

Initzalizing the camera by runing in the termianl - `ros2 launch realsense2_camera rs_launch.py`

Initzalizing transformation node - `ros2 run t265_px4 listener` 

Initzalizing the Agent - `MicroXRCEAgent serial --dev /telem2 -b 57600`


Troubleshooting
If any missing dependencies, they ca be added separately:
sudo apt install python3-colcon-common-extensions
sudo apt install ros-foxy-eigen3-cmake-module
