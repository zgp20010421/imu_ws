# IMU Utils — IMU Allan Variance 标定工具

基于 ROS 的 IMU 噪声参数标定工具，通过 **Allan 方差** 方法对陀螺仪 (Gyroscope) 和加速度计 (Accelerometer) 进行噪声建模，输出可用于 VIO / SLAM 系统（如 Kalibr、OpenVINS、VINS-Mono 等）的噪声参数。
## 功能概述

本工具通过以下流程完成 IMU 标定：

1. 订阅 ROS IMU 话题 (`sensor_msgs/Imu`)，静置采集 IMU 数据
2. 对陀螺仪三轴和加速度计三轴分别计算 Allan 方差
3. 通过 Ceres 拟合 Allan 方差曲线，提取五项噪声参数
4. 输出标定结果 YAML 文件（兼容 Kalibr / OpenVINS 格式）

标定输出的噪声参数含义：

| 参数 | 含义 | 单位 |
|------|------|------|
| `gyr_n` | 陀螺仪角度随机游走 (Angle Random Walk) | rad/s |
| `gyr_w` | 陀螺仪零偏不稳定性 (Bias Instability) | rad/s |
| `acc_n` | 加速度计速度随机游走 (Velocity Random Walk) | m/s² |
| `acc_w` | 加速度计零偏不稳定性 (Bias Instability) | m/s² |

## 项目结构

```
imu_ws/
├── src/
│   ├── imu_utils/                  # 主标定功能包
│   │   ├── include/
│   │   │   └── ros_utils.h         # ROS 工具函数
│   │   ├── src/
│   │   │   ├── imu_an.cpp          # 主程序入口
│   │   │   ├── utils.h / type.h    # 内部工具
│   │   │   ├── acc_lib/            # 加速度计 Allan 方差计算与拟合
│   │   │   │   ├── allan_acc.cpp / .h
│   │   │   │   └── fitallan_acc.cpp / .h
│   │   │   └── gyr_lib/            # 陀螺仪 Allan 方差计算与拟合
│   │   │       ├── allan_gyr.cpp / .h
│   │   │       └── fitallan_gyr.cpp / .h
│   │   ├── launch/                 # 各设备 launch 文件
│   │       ├── A3.launch
│   │       ├── d435i_imu_an.launch
│   │       ├── xsens.launch
│   │       └── ...
│   └── code_utils/                 # 通用工具库（依赖包）
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/code_utils/     # Eigen/OpenCV/数学工具
│       └── src/
```

## 环境依赖

| 依赖 | 版本要求 |
|------|----------|
| **ROS** | Noetic (Ubuntu 20.04) / Melodic (Ubuntu 18.04) |
| **Ceres Solver** | ≥ 1.14 |
| **Eigen3** | ≥ 3.3 |
| **OpenCV** | ≥ 3.0 |

### 安装依赖

```bash
# 1. 安装 Eigen3
sudo apt install libeigen3-dev

# 2. 安装 OpenCV（ROS Noetic 自带，通常无需额外安装）
sudo apt install libopencv-dev

# 3. 安装 Ceres Solver
sudo apt install libceres-dev
```

## 编译步骤

### 1. 初始化 catkin 工作空间（如尚未初始化）

```bash
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src
catkin_init_workspace
```

### 2. 放置代码到工作空间

将 `imu_utils` 和 `code_utils` 两个功能包放入 `src/` 目录下：

```bash
cd ~/imu_ws/src
# 如果是从 git 仓库获取：
# git clone <repo_url>/imu_utils.git
# git clone <repo_url>/code_utils.git
```

确认目录结构如下：

```
imu_ws/src/
├── imu_utils/
└── code_utils/
```

### 3. 安装依赖

```bash
cd ~/imu_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 编译工作空间

```bash
cd ~/imu_ws
catkin_make
```

编译成功后终端会显示类似如下信息：

```
[100%] Built target imu_an
```

### 5. 加载工作空间环境

```bash
source ~/imu_ws/devel/setup.bash
```

> **提示**：可将此行添加到 `~/.bashrc` 中，避免每次手动 source：
> ```bash
> echo "source ~/imu_ws/devel/setup.bash" >> ~/.bashrc
> ```

### 6. 验证编译结果

```bash
# 确认功能包可被识别
rospack find imu_utils
# 应输出: /home/<user>/imu_ws/src/imu_utils
```

## 数据录制

标定前需要**静置 IMU** 录制一段 rosbag 数据。建议时长 **2 小时以上**以获得更准确的结果。

```bash
# 查看 IMU 话题名称
rostopic list | grep imu

# 录制 IMU 数据（以 /camera/imu 话题为例，录制 2 小时）
rosbag record /camera/imu -O imu_data.bag -d 7200
```

> **注意事项**：
> - 录制过程中 IMU 必须**完全静止**，放在稳定平面上
> - 避免振动、温度变化等干扰
> - 录制时长越长，低频噪声参数（零偏不稳定性）越准确
> - 建议至少录制 **2 小时**，推荐 **4 小时**

## 运行标定

### 方式一：使用已有 Launch 文件

```bash
# 终端 1：启动 roscore
roscore

# 终端 2：播放 rosbag 数据
source ~/imu_ws/devel/setup.bash
rosbag play ~/imu_data.bag

# 终端 3：启动标定节点（以 d435i 为例）
source ~/imu_ws/devel/setup.bash
roslaunch imu_utils d435i_imu_an.launch
```

### 方式二：直接使用 rosrun

```bash
rosrun imu_utils imu_an \
  imu_topic:=/camera/imu \
  imu_name:=my_imu \
  data_save_path:=$(rospack find imu_utils)/data/ \
  max_time_min:=120 \
  max_cluster:=100
```

### 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `imu_topic` | string | 订阅的 IMU ROS 话题名 |
| `imu_name` | string | 自定义设备名称（用于输出文件命名） |
| `data_save_path` | string | 标定结果保存路径 |
| `max_time_min` | int | 数据采集时长上限（分钟），应与录制时长一致 |
| `max_cluster` | int | Allan 方差聚类数，一般设为 100 |

## 输出结果说明

标定完成后，在 `data_save_path` 目录下会生成以下文件：

### YAML 参数文件

`<imu_name>_imu_param.yaml` — 主要标定结果，示例：

```yaml
type: IMU
name: d435i
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 3.3831e-03
      gyr_w: 1.9441e-05
   x-axis:
      gyr_n: 5.7884e-03
      gyr_w: 2.8444e-05
   y-axis:
      gyr_n: 2.7484e-03
      gyr_w: 1.6629e-05
   z-axis:
      gyr_n: 1.6125e-03
      gyr_w: 1.3248e-05
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 1.3035e-02
      acc_w: 4.7889e-04
   x-axis:
      acc_n: 1.1670e-02
      acc_w: 2.7821e-04
   y-axis:
      acc_n: 1.5275e-02
      acc_w: 9.1471e-04
   z-axis:
      acc_n: 1.2160e-02
      acc_w: 2.4375e-04
```

### 原始数据文件

| 文件 | 说明 |
|------|------|
| `data_<name>_gyr_{x,y,z}.txt` | 陀螺仪三轴 Allan 偏差原始数据 |
| `data_<name>_acc_{x,y,z}.txt` | 加速度计三轴 Allan 偏差原始数据 |
| `data_<name>_gyr_t.txt` | 陀螺仪时间序列 |
| `data_<name>_acc_t.txt` | 加速度计时间序列 |
| `data_<name>_sim_gyr_{x,y,z}.txt` | 陀螺仪拟合曲线数据 |
| `data_<name>_sim_acc_{x,y,z}.txt` | 加速度计拟合曲线数据 |

可使用 Python / MATLAB 绘制 Allan 方差双对数图进行可视化分析。

## 自定义 Launch 文件

针对不同 IMU 设备，可参考已有 launch 文件创建自定义配置：

```xml
<!-- my_imu.launch -->
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <!-- 修改为你的 IMU 话题名 -->
        <param name="imu_topic" type="string" value="/imu/data"/>
        <!-- 自定义设备名称 -->
        <param name="imu_name" type="string" value="my_device"/>
        <!-- 结果保存路径 -->
        <param name="data_save_path" type="string" value="$(find imu_utils)/data/"/>
        <!-- 采集时长（分钟），需与 rosbag 时长一致 -->
        <param name="max_time_min" type="int" value="120"/>
        <!-- 聚类参数，一般 100 即可 -->
        <param name="max_cluster" type="int" value="100"/>
    </node>
</launch>
```

将文件保存至 `src/imu_utils/launch/my_imu.launch`，运行：

```bash
roslaunch imu_utils my_imu.launch
```

## 已标定设备示例

`data/` 目录中已包含以下设备的标定结果，可作为参考：

| 设备 | Launch 文件 | 标定时长 |
|------|------------|----------|
| DJI A3 | `A3.launch` | 120 min |
| Intel RealSense D435i | `d435i_imu_an.launch` | 120 min |
| Xsens MTi | `xsens.launch` | 200 min |
| Lord GX4 | `gx4.launch` | - |
| MPU 16448 | `16448.launch` | - |
| BMI160 | - | - |

## 常见问题

### 1. 编译报错找不到 Ceres

```
CMake Error: Could not find a package configuration file provided by "Ceres"
```

解决：安装 Ceres Solver

```bash
sudo apt install libceres-dev
```

或从源码编译安装：

```bash
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install
```

### 2. 编译报错找不到 Eigen3

```bash
sudo apt install libeigen3-dev
```

### 3. 标定节点启动后无数据输入

确认以下事项：
- `rosbag play` 正在运行且未播放完毕
- 话题名称与 launch 文件中的 `imu_topic` 一致
- 使用 `rostopic echo /your/imu/topic` 验证数据是否在发布

### 4. 标定结果不理想

- 增加录制时长（建议 ≥ 2 小时）
- 确保 IMU 在录制过程中完全静止
- 检查环境温度是否稳定
- 确认 IMU 数据频率足够（建议 ≥ 100 Hz）

## 致谢

本工具基于 [gaowenliang](https://github.com/gaowenliang414) 的 `imu_utils` 开发，采用 Allan 方差方法进行 IMU 噪声参数标定。

## License

MIT
