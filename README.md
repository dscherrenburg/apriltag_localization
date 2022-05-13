# apriltag_localization
Simulating the localization of the TiaGo-robot in a supermarket environment.

## Setup
#### Create catkin workspace
```
mkdir -p localization_ws/src                                    # Creating workspace folder
cd ~/localization_ws/src                                           # Navigate to workspace source folder
git clone https://github.com/Daan1289/apriltag_localization.git # Clone this github
catkin_create_pkg apriltag_localization std_msgs rospy roscpp   # Create rospackage
cd ~/localization_ws                                            # Navigate to workspace folder
catkin_make                                                     # Create catkin workspace
```
#### Cloning the needed Apriltag repositories
````
cd ~/localization_ws/src                                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git       # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git   # Clone Apriltag ROS wrapper
cd ~/localization_ws                                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y            # Install any missing packages
catkin build                                                  # Build all packages in the workspace (catkin_make_isolated will work also)
````
#### Generating the supermarket environment
```
cd ~/localization_ws/src
git clone https://github.com/levijn/BEP-Visual-Localization.git
vcs import --input retail_store_simulation/retail_store_simulation.rosinstall .
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup slam_toolbox joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller"
catkin build && source devel/setup.bash
```
