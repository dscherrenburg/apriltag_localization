# apriltag_localization
Simulating the localization of the TiaGo-robot in a supermarket environment.

## Setup
#### Create catkin workspace
```
mkdir -p localization_ws/src                                    # Creating workspace folder
cd ~/localization_ws/src                                        # Navigate to workspace source folder
git clone https://github.com/Daan1289/apriltag_localization.git # Clone this github
catkin_create_pkg apriltag_localization std_msgs rospy roscpp   # Create rospackage
```
#### Cloning the needed Apriltag repositories
```
cd ~/localization_ws/src                                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git       # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git   # Clone Apriltag ROS wrapper
cd ~/localization_ws                                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y            # Install any missing packages
```
#### Generating the supermarket environment
If not installed already, install the following tools:
```
sudo apt install python3-vcstool
```
```
sudo apt-get install ros-melodic-ros-numpy
```
```
cd ~/localization_ws/src
git clone https://github.com/levijn/BEP-Visual-Localization.git
mv BEP-Visual-Localization retail_store_simulation
vcs import --input retail_store_simulation/retail_store_simulation.rosinstall .
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup slam_toolbox joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller"
```
#### Downloading Apriltag models for Gazebo
```
cd ~/localization_ws/src/retail_store_simulation/models
git clone https://github.com/koide3/gazebo_apriltag.git
cd gazebo_apriltag/
git clone https://github.com/AprilRobotics/apriltag-imgs.git
```
#### Build the catkin workspace
```
cd ~/localization_ws
catkin build && source devel/setup.bash
```
#### Add the tags you want to use in the simulation
1. Open the file tags.yaml with filepath: /localization_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml
2. Add the tags to the standalone_tags list. 
3. Tags needed for this simulation:
```
standalone_tags:
  [
    {id: 0, size: 0.24},
    {id: 1, size: 0.24},
    {id: 2, size: 0.24},
    {id: 3, size: 0.24},
    {id: 4, size: 0.24},
    {id: 5, size: 0.24},
    {id: 6, size: 0.24},
    {id: 7, size: 0.24},
    {id: 8, size: 0.24},
    {id: 9, size: 0.24},
    {id: 10, size: 0.24},
    {id: 11, size: 0.24},
    {id: 12, size: 0.24},
    {id: 13, size: 0.24},
    {id: 14, size: 0.24},
    {id: 15, size: 0.24},
    {id: 16, size: 0.24},
    {id: 17, size: 0.24},
    {id: 18, size: 0.24},
    {id: 19, size: 0.24}
  ]
```

## Startup
#### Launching apriltag detection (Terminal 1)
```
cd ~/localization_ws
source devel/setup.bash
rosclean purge -y
roslaunch apriltag_localization localization.launch camera_name:=/xtion/rgb
```
#### Launching retailstore simulation (Terminal 2)
```
cd ~/localization_ws
source devel/setup.bash
roscd retail_store_simulation
source scripts/set_gazebo_env.sh
roslaunch retail_store_simulation apriltag.launch rviz:=true
```
1. Add Image display by adding a display in the Displays window.
   - Change the Image Topic to /tag_detections_image
2. Add TF display by adding a display in the Displays window


Not needed atm
###### Launching robot localization estimation (Terminal 3)
```
cd ~/localization_ws
source devel/setup.bash
```
