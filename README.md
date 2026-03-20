# fastlivo2_zhang

这是一个基于 ROS1 Catkin 的 FAST-LIVO2 工作区，包含：
- FAST-LIVO2 主程序
- Livox/相机驱动相关包
- LSLiDAR C16 相关包
- rpg_vikit 依赖包
- 时间戳修复与检查脚本

目标：在另一台 Ubuntu 设备上，按本文步骤完成环境配置、编译并运行。

---

## 1. 推荐环境

- Ubuntu 20.04 + ROS Noetic（推荐）
- 也可尝试 Ubuntu 18.04 + ROS Melodic（需自行调整依赖包名）
- GCC/G++ >= 9（建议）

---

## 2. 获取代码

```bash
git clone https://github.com/xiaozhang466/fastlivo2_zhang.git
cd fastlivo2_zhang
```

---

## 3. 安装系统依赖

先安装基础工具与 ROS：

```bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config
sudo apt install -y ros-noetic-desktop-full python3-rosdep python3-catkin-tools
```

安装 C++ 依赖库：

```bash
sudo apt install -y \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libboost-all-dev \
  libgoogle-glog-dev \
  libyaml-cpp-dev \
  libapr1-dev libaprutil1-dev \
  libomp-dev
```

初始化 rosdep（首次机器需要）：

```bash
sudo rosdep init || true
rosdep update
```

在工作区自动安装 ROS 包依赖：

```bash
cd ~/fastlivo2_zhang
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. 安装 Sophus（必须）

本项目依赖 Sophus（与 FAST-LIVO2 上游一致，建议固定提交）。

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

> 如果后续编译提示找不到 Sophus，请优先检查这一步是否成功。

---

## 5. 编译工作区

```bash
cd ~/fastlivo2_zhang
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)
source devel/setup.bash
```

建议把环境加载写入 shell：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/fastlivo2_zhang/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 运行方式

### 6.1 Livox Avia 配置

```bash
cd ~/fastlivo2_zhang
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

然后播放 bag（另一个终端）：

```bash
source ~/fastlivo2_zhang/devel/setup.bash
rosbag play /path/to/your.bag --clock
```

### 6.2 LSLiDAR C16 配置

```bash
cd ~/fastlivo2_zhang
source devel/setup.bash
roslaunch fast_livo run_lslidar_C16.launch
```

---

## 7. 话题与参数配置

主要配置在：
- [src/FAST-LIVO2/config/avia.yaml](src/FAST-LIVO2/config/avia.yaml)
- [src/FAST-LIVO2/config/lslidar_C16.yaml](src/FAST-LIVO2/config/lslidar_C16.yaml)
- [src/FAST-LIVO2/config/camera_pinhole.yaml](src/FAST-LIVO2/config/camera_pinhole.yaml)

请确认以下话题与你的数据一致：
- `common.img_topic`
- `common.lid_topic`
- `common.imu_topic`

如果不一致，请修改 YAML 后重新启动。

---

## 8. 时间戳工具（可选）

本仓库提供两个脚本：
- [fix_timestamp.py](fix_timestamp.py)：修复 bag 中指定话题的 `header.stamp`
- [check_last_timestamp.py](check_last_timestamp.py)：检查末尾时间戳与相机重复帧

示例：

```bash
cd ~/fastlivo2_zhang
source /opt/ros/noetic/setup.bash
python3 fix_timestamp.py input.bag output.bag
```

> `check_last_timestamp.py` 当前默认检查脚本内写死路径；如需检查其他 bag，请先修改脚本内路径。

---

## 9. 快速自检（换机后必做）

```bash
source /opt/ros/noetic/setup.bash
source ~/fastlivo2_zhang/devel/setup.bash
rospack find fast_livo
roscd fast_livo
```

如果以上命令都正常，说明环境与包路径已正确加载。

---

## 10. 常见问题

1) 编译报错找不到 Sophus  
- 重新执行第 4 节安装步骤，并确认 `sudo ldconfig` 已执行。

2) 运行时报话题无数据  
- 用 `rostopic list`、`rostopic hz` 检查 bag 是否真的发布对应话题。  
- 对照第 7 节修改 YAML 的 topic 名称。

3) 没有轨迹或效果很差  
- 检查相机内参和外参（`camera_pinhole.yaml`、YAML 中 `Rcl/Pcl`）。  
- 检查时间偏移参数（`img_time_offset`、`imu_time_offset`、`lidar_time_offset`）。

---

## 11. 备注

- 本仓库已移除原有子仓库 `.git`，适合作为单一仓库维护。
- 数据集（`.bag`）通常体积较大，默认由 `.gitignore` 忽略，不会自动随代码同步。
