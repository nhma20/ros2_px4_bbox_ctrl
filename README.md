# Drone control with PX4 and ROS2 based on bounding boxes from CNN inference

Dereived from https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo

### Prerequisites
Tested with:
- Ubuntu 20.04.3 LTS
- ROS2 Foxy
- Gazebo 11.9.0
- px4_ros_com 29th Nov
- PX4 Autopilot v1.12.3



### Launch all
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

0. If offboard_control.cpp or other files have been edited, re-run ```install.sh``` script (add new files to script and CMakeLists.txt):
   ```sh
   cd ~/mmWave_ROS2_PX4_Gazebo/
   ( chmod +x ./install.sh )
   ./install.sh
   ```
    If same PX4 and px4_ros_com_ros2 roots:
    ```
    ./install.sh ~/PX4-Autopilot/ ~/px4_ros_com_ros2/
    ```
1. Launch PX4 SITL:
   ```sh
    cd ~/PX4-Autopilot/ 
    make px4_sitl_rtps gazebo
   ```
   Without Gazebo GUI:
   ```sh
    HEADLESS=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without drone following:
   ```sh
    PX4_NO_FOLLOW_MODE=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   After PX4 SITL fully launched, might need to manually start microRTPS client in same terminal:
   ```sh
    micrortps_client start -t UDP
   ```
   Will fail and return -1 if already running.
2. Optional: Open QGroundControl   
3. In a new terminal, start the fake bounding box publisher:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run px4_ros_com lidar_to_mmwave 
   ```
4. In another terminal, start the velocity vector calculator and publisher:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run px4_ros_com vel_ctrl_vec_pub
   ```
5. In a new terminal start microRTPS agent and offboard control:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   micrortps_agent start -t UDP & ros2 run px4_ros_com offboard_control 
   ```
6. Simulated drone in Gazebo should arm and takeoff, hover, do some maneuvers based on bbox in the fake bbox publisher, then land.



### MISC
1. Trajectory setpoint message:
   https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
2. Disabled param:
   pxh> param set NAV_RCL_ACT 0

   NAV_RCL_ACT: curr: 2 -> new: 0
   
4. Add any new ROS2 files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt

6. libignition-common3 error (after software update?) - Copy existing file and rename to match missing file
7. If gazebo does not open, try running ```gazebo --verbose``` to troubleshoot. ```killall gzserver``` should kill any gazebo instances. Else, retry in fresh terminal. Restart PC if all else fails.
11. In CMakeLists.txt add new msgs.
12. After running ```./build_ros2_workspace``` restart all affected executables.
13. iris.sdf (or other models) can be edited to include sensors, like 2D lidar.
14. Display simulated camera feed either with rviz2 or
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```
15. See local packages, and msgs, with: ```ros2 interface packages``` and e.g. ```ros2 interface package px4_msgs```
17. Drone spawn coordinates set in ~/PX4-Autopilot/Tools/sitl_run.sh ?
18. ```*** No rule to make target '/opt/ros/foxy/lib/libfastrtps.so.2.0.2', needed by 'libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so'.  Stop.```
Fixed by renaming closest libfastrtps.so.x.y.z to libfastrtps.so.2.0.2.
19. Dependency errors with PX4, like ```ninja: error: '/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.6.1', needed by 'libmav_msgs.so', missing and no known rule to make it``` may be solved by a PX4 reinstall (remember worlds, models, cmake files etc. must be also be reinstalled into new PX4).
20. If drone enters failsafe when starting offboard_control, ```param set COM_RCL_EXCEPT 4``` in the PX4 console may solve this. Else, try manually publish few setpoints to fmu/manual_control_setpoint/in and then start offboard mode.
21. Showing videos in readme: Just drag and drop your image/video from your local pc to github readme in editable mode.
22. If gradle not working, might have to downgrade Java (JDK) to 11: https://askubuntu.com/questions/1133216/downgrading-java-11-to-java-8




### TODO
0. :yellow_circle: Set up with "real" bounding box to tune PI controller


