# multirotor_launch
ROS launch files used in multirotor UAV control.

The instructions given here are valid for a multirotor UAV running custom PX4 stack on a Pixhawk board and running ROS packages on a PC (either onboard, like Gigabyte Brix or Intel Nuc, or ground PC station).

## Installation & Dependencies

### Pixhawk setup
It is important to flash a proper PX4 stack on Pixhawk flight control unit (fcu). The proper firmware version is available online:
  * [PX4 firmware](https://github.com/westpoint-robotics/Firmware/tree/mpc_offboard_control) (branch mpc_offboard_control)

Once you cloned the valid git repository, checkout the branch mpc_offboard_control, build and upload the firmware. To set up your environment for cross-compile, follow these [instructions](https://dev.px4.io/en/setup/dev_env_linux.html).

Now built and upload the firmare with the following command:
```
make nuttx_px4fmu-v2_default upload
```
The file containing PX4 parameters used on F550 frame (hexrotor) is given [here](https://github.com/westpoint-robotics/Firmware/blob/mpc_offboard_control/parameters/f550_mpc_offboard_control.params). If you are using the same frame, the recommendation is to upload these parameters to your Pixhawk (using QGroundControl) and then perform sensor calibration (definitely) and radio calibration (optional, if using different rc controller).

TODO: add a link to prebuilt firmware binary file.

### ROS setup

#### Core packages for control, estimation and communication
For estimation of the UAV states, namely pose (position and orientation) and velocity (linear and angular), we use the following package 
  * [ethzasl_msf](https://github.com/westpoint-robotics/ethzasl_msf) (branch master)
 
Just follow the instructions for installation on the above link. There is also a link to tutorial which describes in detail what's behind this package.

For UAV position and yaw control, the following package is used
  * [mav_contol_rw](https://github.com/westpoint-robotics/mav_control_rw) (branch master)

Again, just follow the instructions for installation on the given link. This package contains a model predictive control (MPC) algorithm for UAV position control and PI controller for yaw control. More details on MPC algorithm are given in the papers listed in the package github page.

To connect above ROS packages with Pixhawk, the following packages are required:
  * [mavros](https://github.com/westpoint-robotics/mavros) (branch offboard_yaw_control)
  * [multirotor_transformations](https://github.com/westpoint-robotics/multirotor_transformations) (branch master)

We modified mavros to allow for offboard yaw control. This means that instead of yaw reference, we send yaw rate reference to Pixhawk. The link to PX4 stack supporting this modification is given in [Pixhawk section](#pixhawk-setup).

In short, multirotor transformation package contains various ROS nodes which provide interfaces between controller node and mavros, and estimation node and optitrack node, etc.

If your are using Optitrack stream as UAV pose feedback source, in this setup we use vrpn client package. Install vrpn client with:
```
$ sudo apt-get install ros-kinetic-vrpn ros-kinetic-vrpn-client-ros
```
The pose of the vehicle is given in topic /vrpn_client_node/<tracker_name>/pose, where <tracker_name> is the name of the tracker assigned in Motive software.

If you are using multi marker tracking algorithm as UAV pose feedback source, install the following package:

  * [ar_marker_client](https://github.com/westpoint-robotics/ar_marker_client) (branch master)

The installation instructions are given in the above link.

Finally, a set of launch files for running everything is given in package:

  * [multirotor_launch](https://github.com/westpoint-robotics/multirotor_launch) (branch master)

#### Dependecies
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

## Running a SITL simulation

### Brief description
Details on software-in-the-loop (SITL) simulation are given in https://dev.px4.io/en/simulation/ros_interface.html. 

In short, we simulate a multirotor-UAV in Gazebo (default vehicle is quadrotor - 3DR Solo) with PX4 flight stack. MAVROS is used to exchange data with simulated PX4 stack. In our setup we use Spectrum RC controller to give commands to the vehicle. RC receiver is connected to a real Pixhawk flight control unit, which in turn has a USB connection with PC. Another MAVROS node is used to get raw RC channel values. Basically, Pixhawk FCU is used only to get RC values and could be replaced with a much simpler board.

### Running a simulation
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

## Running everything for flying
To run all packages required for flying in optitrack use the following launch file:
```  
  $ roslaunch multirotor_launch px4_msf_mpc_mavros_optitrack.launch 
```

To run all packages required for flying with multi-marker tracking algorihm, use the following launch file:
```
  $ roslaunch multirotor_launch px4_msf_mpc_mavros_multimarker.launch
```
