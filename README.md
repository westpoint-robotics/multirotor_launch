# multirotor_launch
ROS launch files used in multirotor UAV control.

The instructions given here are valid for a multirotor UAV running custom PX4 stack on a Pixhawk board and running ROS packages on a PC (either onboard, like Gigabyte Brix or Intel Nuc, or ground PC station).

## LPE Position Estimator
There are a few modifications with the below setup.  
1. The firmware is the LPE position estimator as described here: https://dev.px4.io/en/ros/external_position_estimation.html
2. This thread is useful: https://github.com/mavlink/mavros/issues/767

## Installation & Dependencies

### Pixhawk setup

#### Compiling source code
It is important to flash a proper PX4 stack on Pixhawk flight control unit (fcu). The proper source of the firmware version is available online:
  * [PX4 firmware](https://github.com/westpoint-robotics/Firmware/tree/mpc_offboard_control) (branch mpc_offboard_control)

Once you cloned the valid git repository, checkout the branch mpc_offboard_control, build and upload the firmware. To set up your environment for cross-compile, follow these [instructions](https://dev.px4.io/en/setup/dev_env_linux.html).

Now built and upload the firmware with the following command:
```
make nuttx_px4fmu-v2_default upload
```
#### Uploading prebuilt firmware binary
An alternative to compiling the source code is to download a prebuilt firmware version available [here](https://www.dropbox.com/s/9whpzoaj7u21y1b/px4fmu-v2_offboard_yaw_control.px4?dl=0). Download this file, open QGroundControl and go to the Firmware update section. Select advanced options and then browse for the downloaded binary file.

#### Pixhawk parameters
The file containing PX4 parameters used on F550 frame (hexrotor) is given [here](https://github.com/westpoint-robotics/Firmware/blob/mpc_offboard_control/parameters/f550_mpc_offboard_control.params). If you are using the same frame, the recommendation is to upload these parameters to your Pixhawk (using QGroundControl) and then perform sensor calibration (definitely) and radio calibration (optional, if using different rc controller).

Make sure to map your rc channels correctly. For offboard control mode, select rc channel (this can be also done through QGroundControl GUI) and threshold by setting the following parameters:
```
RC_MAP_OFFB_SW 9 
RC_OFFB_TH 0.5
```

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

For mavros dependencies, follow instructions on [mavros](https://github.com/westpoint-robotics/mavros) git page, but make sure you switch to *offboard_yaw_control* branch (there are no such instructions on master branch). We modified mavros to allow for offboard yaw control. This means that instead of yaw reference, we send yaw rate reference to Pixhawk. The link to PX4 stack supporting this modification is given in [Pixhawk section](#pixhawk-setup). To be able to communicate with the Pixhawk board, add your user to *dialout* group (restart is required for changes to make effect):
```
sudo usermod -a -G dialout <user>
```

In short, multirotor transformation package contains various ROS nodes which provide interfaces between controller node and mavros, estimation node and optitrack node, etc.

If your are using Optitrack stream as UAV pose feedback source, in this setup we use vrpn client package. Install vrpn client with:
```
$ sudo apt-get install ros-kinetic-vrpn ros-kinetic-vrpn-client-ros
```
The pose of the vehicle is given in topic /vrpn_client_node/<tracker_name>/pose, where <tracker_name> is the name of the tracker assigned in Motive software. Make sure that you select vrpn engine streaming in Motive software and Z-up coordinate frame in broadcast options. Also, make sure that your Linux PC is connected to the same network as Motive PC. Finally, get the IP address of the PC running Motive and add the host named *mocap_station* in the */etc/hosts* file of your Linux PC running ROS vrpn_client_node. The added line should look like:
```
<ip_address> mocap_station
```

If you are using multi marker tracking algorithm as UAV pose feedback source, install the following package:

  * [ar_marker_client](https://github.com/westpoint-robotics/ar_marker_client) (branch master)

The installation instructions are given in the above link.

Finally, a set of launch files for running everything is given in package:

  * [multirotor_launch](https://github.com/westpoint-robotics/multirotor_launch) (branch master)
  
#### Packages for simulation
If you want to run software-in-the-loop (SITL) simulation described [here](#running-a-sitl-simulation), you will need to install additional ROS packages. The main package is rotor_simulator, however to run it in under ROS Kinetic, you will have to checkout previous versions of some packages, namely glog_catkin, mav_comm and rotors_simulator. The complete installation instructions are given in [rotors_simulator](https://github.com/westpoint-robotics/rotors_simulator). 

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

## RC controller setup
In this setup, the vehicle is controlled through RC controller. As with the current version of sotware, you need a RC controller with at least 7 channels: 4 analog (altitude, heading, roll (y position) and pitch (x position) control) and 3 switches. The following switches are used:
  * offboard mode switch (pwm 1000 (0) - Pixhawk control, pwm 1900 (1) - offboard computer control)
  * rc detect switch (pwm 1000 (0) - rc controller off, pwm 1900 (1) - rc controller on)
  * controller mode switch (pwm 1000 (0) - manual control mode, pwm 1500-1900 (1) - position control mode)

The offboard mode switch is used to put Pixhawk in offboard mode control where it listens mavlink messages to control thrust, yaw rate, roll and pitch angles. To be able to turn on the offboard mode control, you should publish references for thrust and attitude through mavlink messages. If the Pixhawk does not receive those messages, it will reject the transition to offboard mode control. In our setup, MPC controller publishes thrust and attitude references, which we receive and convert to ROS messages expected by mavros, which in turn publishes corresponding mavlink messages.

Rc detect switch is used by MPC controller to detect if RC controller is turned on. If you turn off this switch (pwm value 1000 (0)), the MPC controller will not run nor publish anything. As a consequence, there will be no thrust nor attitude messages published through mavlink, and the Pixhawk would reject the transition to offboard mode control. It is safe and recommended to turn on this switch (pwm value 1900) as soon as you turn on your RC controller.

Controller mode switch determines the mode of the MPC controller. There are two basic modes: manual control (the position of the sticks determine the references for thrust, yaw rate, roll and pitch), and position control mode (the sticks determine the references for x,y,z position and yaw angle). It is recommended that during the transition to offboard mode control, the controller is put in manual control mode. After the Pixhawk is switched to offboard mode control, use this switch to enter the position control mode.

In addition, we use another channel for a safety (kill) switch, which you can assign through QGroundControl GUI. However, this channel is only used by Pixhawk and not by MPC controller.

The mapping of the RC channels used by MPC controller can be modified through [this source file](https://github.com/westpoint-robotics/multirotor_transformations/blob/master/include/RcToJoy.h). Just change the number of the rc channel that you wish to use for each function and compile the code. As stated in this [issue](https://github.com/westpoint-robotics/multirotor_transformations/issues/1), this mapping should be done through a parameter file, which will not require a code compilation after a change is made (TODO).


### F550 RC controller
To control f550 vehicle, we have been using a Spektrum DX9 controller. Initial position of all switches should be away from the operator (pwm value 1000). To turn on a specific function, pull the corresponding switch in the position closer to the operator (pwm 1900). The following figure describes the function of the used switches.

![f550_spektrum_dx9](https://user-images.githubusercontent.com/2662767/28971568-b4b0e940-792c-11e7-95cf-ade582b58557.png)

## MPC controller modes
As stated earlier, the MPC position controller has 2 basic modes:  manual control where the position of the sticks determine the references for thrust, yaw rate, roll and pitch, and position control mode, where the sticks determine the references for x,y,z position and yaw angle.

Furthermore, the position control mode has 3 submodes. After entering the position control mode, the controller is in the *carrot* mode. In this mode, by moving the sticks you control the position and heading of the vehicle relative to its measured position and heading (basically, the velocity of the vehicle is controlled). 

The second mode is waypoint following. To enter this mode, you have to call ros service *back_to_position_hold*. Once the service is called, the vehicle holds its current position and waits for a new waypoint. The new waypoint is commanded by publishing a ROS message to *command/pose* topic. To switch back to *carrot* mode, just move any stick used for thrust, heading, roll and pitch control.

The third mode is trajectory following mode. The instructions for this mode will be added once this mode is properly tested. Note that this mode also requires a trajectory planning node, which has not been yet tested (TODO).

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
There are two important parameters that you need to adjust. *px4_dev* contains the path to the serial device used for communicating with Pixhawk and the corresponding baud rate. If you use Pixhawk telemetry port for serial communication, the parameter is usually set to */dev/ttyUSB0:57600* if using 57600 as baud rate or */dev/ttyUSB0:921600* if using higher baud rate. If you are using Pixhawk usb port for serial communication, set this parameter to */dev/ttyACM0:57600*.

The parameter *thrust_max* is used for scaling the thrust reference from mpc controller, which is in Newtons, to range [0,1] expected by Pixhawk. It represents the maximum thrust reference that the mpc controller generates and it depends on the UAV mass. However, this parameter should be fine tuned during the switch to offboard mode control in order to ensure smoothless transition. If your vehicle suddenly goes up when you switch to offboard mode control, increase *thrust_max* and vice versa. E.g. for the f550 vehicle, whose mass is 2.3 kg, we set this parameter to 34. If you know the mass of the vehicle you are using, scale this parameter with the ratio of your vehicle mass and the f550 UAV mass (2.3 kg) and then fine tune the parameter, as described earlier.  

There is only one thing left before flying, initialize a multi-sensor fusion (msf) filter by using rqt_reconfigure
```
rosrun rqt_reconfigure rqt_reconfigure
```
Under *<namespace>/msf_optitrack_pose_sensor/pose_sensor* click the *core_init_filter* option, where <namespace> is the namespace of your msf node.

To run all packages required for flying with multi-marker tracking algorihm, use the following launch file:
```
  $ roslaunch multirotor_launch px4_msf_mpc_mavros_multimarker.launch
```
Again, before flying initialize the multi-sensor fusion filter.
