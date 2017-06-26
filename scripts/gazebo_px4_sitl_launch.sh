PX4_SRC_PATH='/home/thaus/development/px4_firmware'
echo $PX4_PATH
source $PX4_SRC_PATH/Tools/setup_gazebo.bash $PX4_SRC_PATH $PX4_SRC_PATH/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_SRC_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_SRC_PATH/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch ns:=sitl
