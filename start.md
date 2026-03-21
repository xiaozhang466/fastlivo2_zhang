镭神雷达启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch lslidar_c16_decoder lslidar_c16.launch

imu启动指令：
cd ~/fast_localization_ws
source devel/setup.bash
roslaunch imu_launch imu_msg1.launch

海康相机启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch

fastlivo2启动指令：
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo run_lslidar_C16.launch