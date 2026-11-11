# Go2 传感器闭环与 ROS 2 联调

## 闭环数据流

MuJoCo 在 500 Hz 推进物理。250 Hz 的控制进程读取 IMU、12 个关节的位置、速度和执行力矩，再将足端与地面的法向接触力按 8 N 阈值转换为四个接触位。传入状态估计器的是接触位。控制器使用估计机身状态和关节测量运行既有 MPC 与全身力矩 QP。仿真机身真值只进入误差和终止条件计算。

| 文件 | 责任 |
|---|---|
| `base_estimation.py` | 用关节编码器数据和名义模型计算基座坐标系中的足端位置与关节引起的足端速度 |
| `cpp/base_estimator.cpp` | C++17/Eigen 实现姿态传播、惯性积分、接触去抖和足端运动学修正 |
| `run_sensor_control.py` | 执行估计、控制和物理循环，保存同时间戳估计与仿真真值 |
| `ros_bridge.py` | 系统 Python 3.10 上的 ROS 2 Humble 节点，将话题与项目 Python 3.11 进程连接 |
| `local_transport.py` | 本机 UDP 消息编解码、命令新鲜度和停步请求 |
| `check_ros_loop.py` | 作为独立 DDS 参与者发送速度，订阅并核验四个话题 |

估计初值取已知站立姿态与位置原点。姿态用陀螺仪积分，在至少三足可靠支撑且接近静止时用重力方向修正横滚和俯仰。连续三拍接触后足端才参与修正，离地立即取消。世界系竖直锚点使用已知平地高度与足球半径，水平锚点在每次落脚时用当时估计建立。惯性与足端残差用一阶增益和有界修正融合，没有协方差传播。偏航和水平位置缺少绝对观测，仍会累积漂移。

仿真在内存里将 IMU 放到基座原点，坐标轴与机身一致。测量加固定偏置与种子 2026 的高斯噪声，参数见每次运行的 summary.json。力矩来自执行器输出，接触位来自仿真接触求解。默认全程平地，基座添加固定载荷。ROS 时间戳取桥接节点发布时钟，状态消息另带仿真秒数。未发布 TF 或标定过的里程计协方差。

## 话题和停步

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 输入 | 首个正向指令选定本次速度，之后以同一速度持续续约 |
| `/go2/imu` | `sensor_msgs/Imu` | 输出 | 机身系角速度和比力，传感器 QoS |
| `/go2/joint_states` | `sensor_msgs/JointState` | 输出 | 12 关节的名字、角度、速度与执行力矩，传感器 QoS |
| `/go2/odom` | `nav_msgs/Odometry` | 输出 | odom 系位置姿态与基座系速度，可靠传输 |
| `/go2/status` | `std_msgs/String` | 输出 | JSON 状态含接触、负载估计、停步原因和拒绝计数，可靠传输 |

速度接口支持 0.02–0.10 m/s 的固定前进速度，已验收 0.10 m/s。发送全零 Twist 或连续 0.6 秒收不到有效续约，将锁存停步。控制状态机等待规划步态进入四足支撑再保持站立。进入 STOPPED 后需启动新回合才能重新行走。遥测以 25 Hz 采样，UDP 仅使用 `127.0.0.1`，默认端口 26371/26372。诊断发现本工作站 UDP 负载 1472 字节可达、1480 字节超时。回环消息用 zlib 压缩并限制在 1200 字节以内，汇总记录实际最大包长，本轮最大 874 字节。

验收脚本使用 [Fast DDS 的 UDPv4 单播配置](configs/fastdds_loopback.xml)，限制回环地址和最大消息缓冲 1200 字节，并在探针发现四个发布话题后才开始发车。两组验收使用不同的进程间端口，ROS Domain 默认 72。XML 的传输参数见 [Fast DDS 官方文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html)。

## 在当前工作站复现

进入 WSL 的 Ubuntu 22.04，使用项目 Linux 目录。C++ 构建依赖为 `g++`、`make`、`cmake`、`libeigen3-dev`。ROS 2 侧使用 Humble 的 `rclpy`、`rmw_fastrtps_cpp` 和标准消息包，由系统 Python 3.10 加载。项目控制进程仍使用独立的 Python 3.11 环境。新机先按 [ROS 2 Humble 官方安装说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst) 配置签名软件源，再安装对应包。

```bash
cd /root/robotrl_go2/go2-payload-control
./build.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv/bin/python run_sensor_control.py --load 0 --output reports/local_sensor/nominal
.venv/bin/python run_sensor_control.py --load 0.2 --output reports/local_sensor/loaded
./check_ros_loop.sh reports/local_sensor/ros_timeout timeout
./check_ros_loop.sh reports/local_sensor/ros_zero zero
.venv/bin/python check_sensor_results.py reports/local_sensor
```

每次实跑使用新的输出路径。已有的证据可以直接评分，无需再跑仿真。

```bash
.venv/bin/python check_sensor_results.py reports/sensor_v2
```

[验收数据](reports/sensor_v2/ACCEPTANCE.json) 记录门槛和逐工况结果。两组直接控制回合均为 12 秒。两组 ROS 回合均为 10 秒，包含独立 DDS 客户端记录。行走估计误差只统计第 4.5 秒后、控制器仍处于 RUN 的样本。机身真值速度与目标速度之差是跟踪误差，估计速度与机身真值之差是估计误差。两种指标对应不同问题。

该验收覆盖 WSL 中的平地前进、固定载荷与受控停步。机器人 SDK、真机传感器标定、实机通信及硬实时调度尚未覆盖。
