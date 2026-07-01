镭神雷达启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch lslidar_c16_decoder lslidar_c16.launch
/lslidar_point_cloud

imu启动指令：
cd ~/fast_localization_ws
source devel/setup.bash
roslaunch imu_launch imu_msg1.launch
/IMU_data

海康相机启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch
/left_camera/image

fastlivo2启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo run_lslidar_C16.launch

rosbag record -O fastlivo2_test_01.bag /lslidar_point_cloud /IMU_data /left_camera/image