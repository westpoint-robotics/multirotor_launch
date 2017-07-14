# multirotor_launch
ROS launch files used in multirotor UAV control.

## Installation
To receive optitrack data, we used vrpn client in this package. Install vrpn client with:
```
$ sudo apt-get install ros-kinetic-vrpn ros-kinetic-vrpn-client-ros
```
We use [multimaster_fkie](http://wiki.ros.org/multimaster_fkie) package, install it with:
```
sudo apt-get install ros-kinetic-multimaster-fkie
```
To use a PointGrey camera, simply install their official driver:
```
sudo apt-get install ros-kinetic-pointgrey-camera-driver
```
To be able to run the camera driver without super user privileges , execute flycap2-conf from the [scripts](scripts) folder and follow the script instructions. Script creates pgrimaging group, adds a user to the group and creates udev rules.
```
sudo -s
./flycap2-conf
```
If running a ROS barebone version, install image transport pluggins:
```
sudo apt-get install ros-kinetic-image-transport-plugins
```
To compile mavros extras install:
```
sudo apt-get install ros-kinetic-urdf ros-kinetic-control-toolbox
```

## Running a SITL simulation

### Brief description
Details on software-in-the-loop (SITL) simulation are given in https://dev.px4.io/en/simulation/ros_interface.html. 

In short, we simulate a multirotor-UAV in Gazebo (default vehicle is quadrotor - 3DR Solo) with PX4 flight stack. MAVROS is used to exchange data with simulated PX4 stack. In our setup we use Spectrum RC controller to give commands to the vehicle. RC receiver is connected to a real Pixhawk flight control unit, which in turn has a USB connection with PC. Another MAVROS node is used to get raw RC channel values. Basically, Pixhawk FCU is used only to get RC values and could be replaced with a much simpler board.

### Dependencies
To run SITL, the following packages are required (their dependencies and instructions for installing are given within each package):
  * [mavros](http://wiki.ros.org/mavros)
  * [mav_control_rw](https://github.com/westpoint-robotics/mav_control_rw) (linear MPC is used for control)
  * [multirotor_transformations](https://github.com/westpoint-robotics/multirotor_transformations)
In addition, you will also need PX4 stack source code:
  * [px4_firmware](https://github.com/PX4/Firmware)

### Running
In first terminal, run Gazebo simulation by executing [script](https://github.com/westpoint-robotics/multirotor_launch/blob/master/scripts/gazebo_px4_sitl_launch.sh). Edit the first line in the script to indicate the path to your local PX4 source code and run:
```
   $ ./gazebo_px4_sitl_launch.sh 
```
In second terminal, run a lunch which starts all other packages required for controlling the multirotor UAV:
```
  $ roslaunch multirotor_launch sitl_px4_mpc.launch
```
To arm the vehicle, call service:
  * /sitl/mavros/cmd/arming (True)

To put PX4 in offboard control, call service:
  * /sitl/mavros/set_mode (OFFBOARD)
