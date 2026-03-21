# fastlivo2_ws

这是一个基于 ROS1 Catkin 的 FAST-LIVO2 工作区，已经包含：

- `src/FAST-LIVO2`：FAST-LIVO2 主程序
- `src/LIV_handhold/livox_ros_driver`：Livox 驱动
- `src/LIV_handhold/mvs_ros_driver`：海康 MVS 相机驱动
- `src/lslidar`：LSLiDAR C16 相关包
- `src/rpg_vikit`：FAST-LIVO2 依赖
- `fix_timestamp.py` / `check_last_timestamp.py`：时间戳修复与检查脚本

这份 README 的目标不是解释算法，而是给另一台新设备一份可直接照着执行的部署流程。优先覆盖两种场景：

1. 真机运行：Livox/LSLiDAR + 海康相机
2. 仅回放 rosbag：不接海康相机，也可以跳过 `mvs_ros_driver`

---

## 1. 推荐环境

- Ubuntu 20.04
- ROS Noetic

当前仓库默认配置中的核心话题如下：

- Livox 模式：
  - 图像：`/left_camera/image`
  - 激光：`/livox/lidar`
  - IMU：`/livox/imu`
- LSLiDAR C16 模式：
  - 图像：`/left_camera/image`
  - 激光：`/lslidar_point_cloud`
  - IMU：`/IMU_data`

---

## 2. 获取代码

下面假设工作区路径统一放在 `~/fastlivo2_ws`。如果你用别的目录，请自行替换。

```bash
git clone https://github.com/xiaozhang466/fastlivo2_zhang.git ~/fastlivo2_ws
cd ~/fastlivo2_ws
```

如果你不是通过 `git clone`，而是直接把当前仓库拷到新机器，也可以；但新机器首次编译时，最好不要依赖旧机器带过来的 `build/`、`devel/`。

---

## 3. 安装 ROS 和系统依赖

先安装基础工具和 ROS：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config \
  python3-rosdep python3-catkin-tools \
  ros-noetic-desktop-full
```

安装本项目常用依赖：

```bash
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-pcl-ros \
  ros-noetic-tf \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libboost-all-dev \
  libgoogle-glog-dev \
  libyaml-cpp-dev \
  libapr1-dev \
  libaprutil1-dev \
  libomp-dev
```

初始化 `rosdep`（新机器第一次装 ROS 时需要）：

```bash
sudo rosdep init || true
rosdep update
```

安装工作区中 ROS 包依赖：

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

说明：

- 本仓库已经自带 `src/rpg_vikit`，不需要再单独 clone。
- 如果你只打算离线回放 bag，可以先跳过海康相机驱动，后面编译时黑名单掉 `mvs_ros_driver`。

---

## 4. 安装 Sophus

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

上游旧提交 `a621ff` 编译失败

如果你强制使用 `a621ff`，并在 `so2.cpp` 看到：

- `lvalue required as left operand of assignment`

把下面两句：

- `unit_complex_.real() = 1.;`
- `unit_complex_.imag() = 0.;`

改成：

- `unit_complex_.real(1.);`
- `unit_complex_.imag(0.);`

然后重新编译。

---

## 5. 安装海康 MVS SDK

这是新机器最容易卡住的步骤。当前仓库中的 `mvs_ros_driver` 直接依赖海康官方 SDK：

- 头文件：`/opt/MVS/include/MvCameraControl.h`
- 动态库：
  - x86_64：`/opt/MVS/lib/64/libMvCameraControl.so`
  - aarch64：`/opt/MVS/lib/aarch64/libMvCameraControl.so`

如果这些文件不存在，`mvs_ros_driver` 无法编译。

### 5.1 下载并安装 MVS

从海康机器视觉官网或供应商页面下载 Linux 版安装包。官网有时不是直接提供 `.run`，而是先提供一层 `zip`，例如你现在下载到的 `MVS_Linux_STD_*.zip` 和 `MvCamCtrlSDK_STD_*.zip`。

对当前仓库，优先安装 `MVS_Linux_STD` 这个完整包；它通常已经包含运行和调试相机所需的 MVS 组件、头文件和动态库。`MvCamCtrlSDK_STD` 先保留备用，只有在完整包里缺少 `MvCameraControl.h` 或 `libMvCameraControl.so` 时再考虑补装。

按照你现在的解压结果，如果机器是 Ubuntu `x86_64`，优先安装 `MVS-4.6.1_x86_64_*.deb`。像 `i386` 这种 32 位安装包可以直接忽略。第二个压缩包里那批 `MvCamCtrlSDK_Runtime...` 文件先不要装；它们更像运行库补充包，不是这个项目当前最需要的主安装入口。

安装示例：

```bash
cd ~/Downloads
unzip MVS_Linux_STD_*.zip
cd MVS_Linux_STD_*
ls
sudo dpkg -i MVS-4.6.1_x86_64_*.deb
sudo apt -f install -y
```

如果你机器不是 `x86_64`，再换成对应架构的安装包。安装完成后，默认安装路径通常是 `/opt/MVS`。

### 5.2 检查 SDK 是否安装成功

```bash
ls /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib/64/libMvCameraControl.so
```

如果你是 ARM64 设备，请改查：

```bash
ls /opt/MVS/lib/aarch64/libMvCameraControl.so
```

### 5.3 配置运行时库路径

`mvs_ros_driver` 不仅要在编译期找到头文件，运行时也必须能找到 `libMvCameraControl.so`。

#### x86_64

```bash
echo "/opt/MVS/lib/64" | sudo tee /etc/ld.so.conf.d/mvs.conf
sudo ldconfig
ldconfig -p | grep MvCameraControl
```

#### aarch64

```bash
echo "/opt/MVS/lib/aarch64" | sudo tee /etc/ld.so.conf.d/mvs.conf
sudo ldconfig
ldconfig -p | grep MvCameraControl
```

### 5.4 安装 MVS 后检查 `libusb` 是否被覆盖

这一步很重要。部分 MVS 安装包会在 `/opt/MVS/lib/64` 下放入自己的 `libusb-1.0.so.0`。如果这个版本偏老，`PCL` 在链接 `libpcl_io.so` 时可能报：

- `undefined reference to libusb_set_option`

先检查当前系统实际会优先使用哪个 `libusb`：

```bash
ldconfig -p | grep libusb-1.0
nm -D /opt/MVS/lib/64/libusb-1.0.so.0 | grep libusb_set_option || true
nm -D /lib/x86_64-linux-gnu/libusb-1.0.so.0 | grep libusb_set_option
```

如果 `/opt/MVS/lib/64/libusb-1.0.so.0` 查不到 `libusb_set_option`，而系统库 `/lib/x86_64-linux-gnu/libusb-1.0.so.0` 能查到，就说明海康自带的旧版 `libusb` 抢到了系统库位置。处理方式是把旧库移出 `/opt/MVS/lib/64`，再把 MVS 目录里的 `libusb` 显式指向系统库：

```bash
sudo mkdir -p /opt/MVS/lib/64/backup
sudo mv /opt/MVS/lib/64/libusb-1.0.so.0.bak /opt/MVS/lib/64/backup/ 2>/dev/null || true
sudo mv /opt/MVS/lib/64/libusb-1.0.so.0 /opt/MVS/lib/64/backup/ 2>/dev/null || true
sudo mv /opt/MVS/lib/64/libusb-1.0.so /opt/MVS/lib/64/backup/ 2>/dev/null || true

sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so /opt/MVS/lib/64/libusb-1.0.so
sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /opt/MVS/lib/64/libusb-1.0.so.0
sudo ldconfig
```

然后再次确认：

```bash
readlink -f /opt/MVS/lib/64/libusb-1.0.so.0
nm -D /opt/MVS/lib/64/libusb-1.0.so.0 | grep libusb_set_option
```

如果 `readlink -f` 指向 `/lib/x86_64-linux-gnu/libusb-1.0.so.0`，并且 `nm` 能看到 `libusb_set_option`，说明冲突已经解除。

### 5.5 编译前检查相机配置

海康节点使用的配置文件在：

- [src/LIV_handhold/mvs_ros_driver/config/left_camera_trigger.yaml](src/LIV_handhold/mvs_ros_driver/config/left_camera_trigger.yaml)
- [src/LIV_handhold/mvs_ros_driver/config/right_camera_trigger.yaml](src/LIV_handhold/mvs_ros_driver/config/right_camera_trigger.yaml)

当前关键参数含义：

- `TopicName`
  - 默认左目是 `left_camera/image`
  - FAST-LIVO2 当前配置期待的是 `/left_camera/image`
- `TriggerEnable`
  - `1`：硬同步触发模式
  - `0`：关闭触发，便于先验证相机能否正常出图
- `SerialNumber`
  - 单相机时，驱动会直接选第 0 个设备，序列号是否填写不影响启动
  - 多相机时，必须改成实际相机序列号，否则会报找不到指定相机
- `ExposureTime` / `GainAuto` / `PixelFormat`
  - 根据你的相机型号和曝光需求调整

如果你只是想先确认新机器上相机驱动能不能跑起来，建议先把 `TriggerEnable` 临时设为 `0`，确认正常出图后再切回 `1`。

### 5.6 运行前验证海康驱动

先单独启动相机节点：

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch
```

另开一个终端检查：

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic list | grep left_camera
rostopic hz /left_camera/image
```

如果这里已经没有图像，先不要继续启动 FAST-LIVO2，先把海康驱动调通。

---

## 6. 编译工作区

### 6.1 先处理 `cv_bridge_DIR` 的路径问题

当前仓库的 [src/LIV_handhold/mvs_ros_driver/CMakeLists.txt](src/LIV_handhold/mvs_ros_driver/CMakeLists.txt) 里写死了这一行：

```cmake
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
```

但标准的 ROS Noetic apt 安装通常是：

```bash
/opt/ros/noetic/share/cv_bridge/cmake
```

所以如果新机器编译 `mvs_ros_driver` 时提示 `cv_bridge` 找不到，先把上面这一行改成：

```cmake
set(cv_bridge_DIR /opt/ros/noetic/share/cv_bridge/cmake)
```

或者直接删掉这行，让 `find_package(catkin ...)` 自己去找。

### 6.2 正常编译

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)
source devel/setup.bash
```

### 6.3 如果暂时不用海康相机

如果你现在只是想先跑 FAST-LIVO2 或回放 bag，而海康 MVS SDK 还没装好，可以先跳过 `mvs_ros_driver`：

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES="mvs_ros_driver" -j$(nproc)
source devel/setup.bash
```

---

## 7. 换机后的快速自检

完成编译后，先做最基础的包路径检查：

```bash
source /opt/ros/noetic/setup.bash
source ~/fastlivo2_ws/devel/setup.bash
rospack find fast_livo
rospack find mvs_ros_driver
rospack find livox_ros_driver
```

如果这些都能找到，说明工作区环境基本正常。

海康 SDK 运行时也要再确认一遍：

```bash
ldconfig -p | grep MvCameraControl
```

---

## 8. 真机运行流程

下面默认你使用的是 Livox Avia + 海康单相机。如果你用的是 LSLiDAR，看第 9 节。

### 8.1 终端 1：海康相机

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch
```

### 8.2 终端 2：Livox 驱动

如果串口权限有问题，先执行：

```bash
sudo chmod a+rw /dev/ttyUSB0
```

然后启动 Livox：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch livox_ros_driver livox_lidar_msg.launch
```

建议立刻检查：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 8.3 终端 3：FAST-LIVO2

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

### 8.4 真机模式下必须确认的事情

- 相机话题是否真的有数据：`rostopic hz /left_camera/image`
- Livox 话题是否真的有数据：
  - `rostopic hz /livox/lidar`
  - `rostopic hz /livox/imu`
- FAST-LIVO2 配置文件中的 topic 名称是否和驱动一致
- 海康相机如果启用了 `TriggerEnable: 1`，必须确认硬同步触发链路已经工作

说明：

- `mvs_ros_driver` 会尝试读取 `/home/$USER/timeshare` 共享时间戳文件。
- 如果日志里提示 `Timeshare file not found`，驱动会退回 `ros::Time::now()`，节点仍能运行，但硬同步时间戳不会生效。

---

## 9. LSLiDAR C16 流程

如果你用的是 LSLiDAR C16，对应 FAST-LIVO2 启动命令是：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo run_lslidar_C16.launch
```

启动 FAST-LIVO2 之前，请先确保你的激光驱动已经发布：

- `/lslidar_point_cloud`
- `/IMU_data`
- `/left_camera/image`

这些名称来自当前配置文件 [src/FAST-LIVO2/config/lslidar_C16.yaml](src/FAST-LIVO2/config/lslidar_C16.yaml)。

---

## 10. rosbag 回放流程

### 10.1 Livox Avia bag

终端 1：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

终端 2：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
rosbag play /path/to/your.bag --clock
```

### 10.2 LSLiDAR C16 bag

终端 1：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
roslaunch fast_livo run_lslidar_C16.launch
```

终端 2：

```bash
cd ~/fastlivo2_ws
source devel/setup.bash
rosbag play /path/to/your.bag --clock
```

如果 bag 里的话题名和当前 YAML 不一致，先改配置再启动。

---

## 11. 启动前必须检查的配置文件

### 11.1 Livox 模式

- [src/FAST-LIVO2/config/avia.yaml](src/FAST-LIVO2/config/avia.yaml)
  - `common.img_topic: /left_camera/image`
  - `common.lid_topic: /livox/lidar`
  - `common.imu_topic: /livox/imu`
  - `time_offset.img_time_offset: 0.1`

### 11.2 LSLiDAR 模式

- [src/FAST-LIVO2/config/lslidar_C16.yaml](src/FAST-LIVO2/config/lslidar_C16.yaml)
  - `common.img_topic: /left_camera/image`
  - `common.lid_topic: /lslidar_point_cloud`
  - `common.imu_topic: /IMU_data`
  - `time_offset.lidar_time_offset: -0.02`

### 11.3 相机内参

- [src/FAST-LIVO2/config/camera_pinhole.yaml](src/FAST-LIVO2/config/camera_pinhole.yaml)
  - 当前内参基于 `1280x1024`
  - 如果你修改了相机输出分辨率或 `image_scale`，内参也要同步调整

### 11.4 海康相机节点参数

- [src/LIV_handhold/mvs_ros_driver/config/left_camera_trigger.yaml](src/LIV_handhold/mvs_ros_driver/config/left_camera_trigger.yaml)
- [src/LIV_handhold/mvs_ros_driver/config/right_camera_trigger.yaml](src/LIV_handhold/mvs_ros_driver/config/right_camera_trigger.yaml)

重点检查：

- `TopicName`
- `TriggerEnable`
- `SerialNumber`
- `ExposureTime`
- `PixelFormat`

---

## 12. 时间戳工具

仓库中有两个辅助脚本：

- [fix_timestamp.py](fix_timestamp.py)：修复 bag 中指定话题的 `header.stamp`
- [check_last_timestamp.py](check_last_timestamp.py)：检查最后时间戳和相机重复帧情况

示例：

```bash
cd ~/fastlivo2_ws
source /opt/ros/noetic/setup.bash
python3 fix_timestamp.py input.bag output.bag
```

说明：

- `check_last_timestamp.py` 当前默认检查脚本里写死的路径。
- 如果你要检查其他 bag，先改脚本里的输入路径。

---

## 13. 常见问题

### 13.1 `fatal error: MvCameraControl.h: No such file or directory`

原因：海康 MVS SDK 没装好，或者不是默认安装到 `/opt/MVS`。

处理：

1. 确认 `/opt/MVS/include/MvCameraControl.h` 是否存在
2. 确认 `/opt/MVS/lib/64/libMvCameraControl.so` 或 `/opt/MVS/lib/aarch64/libMvCameraControl.so` 是否存在
3. 再重新编译工作区

### 13.2 运行时报 `error while loading shared libraries: libMvCameraControl.so`

原因：运行时动态库路径没配置。

处理：

```bash
echo "/opt/MVS/lib/64" | sudo tee /etc/ld.so.conf.d/mvs.conf
sudo ldconfig
ldconfig -p | grep MvCameraControl
```

ARM64 把 `64` 改成 `aarch64`。

### 13.3 编译 `FAST-LIVO2` 时出现 `undefined reference to libusb_set_option`

原因：安装 MVS 后，`/opt/MVS/lib/64` 里的旧版 `libusb-1.0.so.0` 抢在系统 `libusb` 前面被链接器使用，导致 `libpcl_io.so` 在链接 `fastlivo_mapping` 时找不到 `libusb_set_option`。

典型报错：

- `/usr/bin/ld: ... libpcl_io.so: undefined reference to libusb_set_option`

处理：

1. 检查 `/opt/MVS/lib/64/libusb-1.0.so.0` 和 `/lib/x86_64-linux-gnu/libusb-1.0.so.0` 哪个包含 `libusb_set_option`
2. 如果只有系统库包含该符号，把 `/opt/MVS/lib/64` 下旧的 `libusb` 文件移到 `backup/`，不要只是改名留在原目录里
3. 用软链接把 `/opt/MVS/lib/64/libusb-1.0.so` 和 `/opt/MVS/lib/64/libusb-1.0.so.0` 指向系统库
4. `sudo ldconfig` 后重新编译

注意：如果你只是把旧库改名成 `libusb-1.0.so.0.bak` 但仍留在 `/opt/MVS/lib/64`，`ldconfig` 可能会根据 `SONAME` 自动把 `libusb-1.0.so.0` 又重新链回这个旧库。所以旧库必须移出当前目录。

### 13.4 编译时报 `cv_bridge` 找不到

原因：`mvs_ros_driver/CMakeLists.txt` 把 `cv_bridge_DIR` 写死到了 `/usr/local/share/cv_bridge/cmake`，但标准 Noetic 在 `/opt/ros/noetic/share/cv_bridge/cmake`。

处理：

- 改 [src/LIV_handhold/mvs_ros_driver/CMakeLists.txt](src/LIV_handhold/mvs_ros_driver/CMakeLists.txt)
- 或删掉那行 `set(cv_bridge_DIR ...)`

### 13.5 日志提示 `Find No Devices!`

原因通常有这几类：

1. 相机本身没有被系统识别
2. GigE 相机不在同一网段
3. USB 线或供电有问题
4. MVS SDK 装了，但设备权限或链路没通

处理建议：

1. 先用海康 MVS 自带工具确认相机能被看到
2. GigE 相机确认电脑网卡和相机 IP 在同一网段
3. 先把 `TriggerEnable` 改成 `0`，排除触发链路问题

### 13.6 日志提示 `Can not find the camera with serial number ...`

原因：当前机器上枚举到了多个相机，但 YAML 里 `SerialNumber` 不是实际序列号。

处理：

1. 查看节点启动日志里打印出的设备序列号
2. 修改对应 YAML 里的 `SerialNumber`
3. 单相机时也建议把序列号改成真实值，避免以后接第二台相机后踩坑

### 13.7 日志提示 `Timeshare file not found`

原因：当前没有时间戳共享文件 `/home/$USER/timeshare`。

影响：

- 相机节点还能跑
- 但图像时间戳会退回系统时间，而不是硬同步时间戳

如果你现在的目标只是先把项目在新机器上跑起来，这条警告可以先接受；如果目标是复现硬同步精度，就必须继续把同步链路配好。

### 13.8 FAST-LIVO2 启动了，但没有轨迹或效果很差

优先检查：

1. 相机、LiDAR、IMU 话题名是否和 YAML 一致
2. [src/FAST-LIVO2/config/camera_pinhole.yaml](src/FAST-LIVO2/config/camera_pinhole.yaml) 内参是否匹配当前分辨率
3. 外参 `Rcl` / `Pcl` 是否正确
4. 时间偏移参数：
   - `img_time_offset`
   - `imu_time_offset`
   - `lidar_time_offset`

---

## 14. 建议的换机顺序

如果你想把排障成本降到最低，按下面顺序做：

1. 装 Ubuntu + ROS Noetic
2. 装 apt 依赖和 Sophus
3. 安装海康 MVS SDK，并确认 `/opt/MVS/include`、`/opt/MVS/lib/...` 都存在
4. 检查 MVS 是否把旧版 `libusb` 放进了 `/opt/MVS/lib/64`，必要时切回系统 `libusb`
5. 处理 `cv_bridge_DIR` 路径问题
6. 编译整个工作区
7. 只启动 `mvs_ros_driver`，先确认相机能出图
8. 只启动 LiDAR 驱动，确认 LiDAR/IMU 话题正常
9. 最后再启动 FAST-LIVO2

这样做的原因很简单：如果你一上来就同时拉起全部节点，问题会混在一起，不容易定位。

---

## 15. 备注

- 本仓库是完整 catkin 工作区，不是单独的 ROS package。
- `.bag` 数据通常体积较大，默认不会随代码一起管理。
- 如果你只想先验证算法流程，可以先用 bag 回放，把海康驱动放到最后再处理。
