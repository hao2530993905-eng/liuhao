# SCREW07091444

当前版本用于验证 **UR5e + Damiao DM4310 电批 + 相机 + LeRobot** 自动螺丝装配系统中的机械臂运动、工具轴预插、力数据记录和示范数据采集。

当前推荐的基础 Demo **暂不接入视觉自动生成目标位姿，也不由 `apps/screw_demo/main.py` 的状态机控制电批旋转**。操作者手动提供孔前 TCP 位姿，机械臂先以 `moveJ` 到达孔前附近，再用键盘在工具坐标系 XY 平面内微调对准孔口，最后让机械臂沿工具坐标系 Z 轴低速预插，并记录 UR5e 的 TCP 力数据。

LeRobot 采集入口 `record_main_insert_dataset.py` 是另一条实验流程：它会记录 LeRobotDataset，并在确认后通过 `ScrewClient` 调用电批服务。因此，运行基础运动 Demo 和运行“带电批的数据采集流程”前，必须先确认自己使用的是哪一个入口。

## 项目结构

```text
SCREW07091444/
├── apps/
│   ├── screw_demo/
│   │   ├── main.py              # 当前推荐的人工对准 + 工具轴预插状态机
│   │   ├── config/default.json  # main.py 默认运动和力记录参数
│   │   ├── ur_driver.py         # UR5e RTDE 状态读取、moveJ、moveL、servoL
│   │   ├── screw_client.py      # DM4310 C++ 服务的 JSON/Socket 客户端
│   │   ├── teach.py              # 示教、键盘点动和位姿保存工具
│   │   └── tests/                # 分层硬件测试和客户端测试
│   ├── camera/
│   │   ├── ring.py               # 图像/点云采集、圆环检测、圆心和法向计算
│   │   └── camera_service.py     # 常驻 HTTP 相机服务
│   └── screw_driver/             # DM4310/OpenArm CAN C++ 服务端和监控程序
├── lerobot_robot_screw/
│   ├── src/lerobot_robot_screw/  # LeRobot robot package、数据特征和处理器
│   ├── scripts/recording/        # 真实机器人、相机并行和数据采集入口
│   ├── scripts/tools/            # 数据集、坐标、采样和可视化工具
│   └── tests/                    # LeRobot 包测试
├── scripts/benchmarks/           # 启动耗时和并行启动测试
├── artifacts/
│   ├── datasets/                 # LeRobot 数据集和原始 JSONL
│   ├── logs/                     # 力数据、运行日志和实验日志
│   ├── camera_test_data/         # 相机图片、PLY、poly 等中间结果
│   └── models/                   # 相机检测模型
├── archive/legacy/               # 旧版/重复脚本，仅供追溯
├── third_party/                  # 外部依赖，当前不修改
├── environment.yml
├── README.md
└── 项目说明.md
```

## 主要程序

### `apps/screw_demo/main.py`

当前最适合验证 UR5e 运动链路和工具坐标系的入口。状态机为：

```text
INIT
  -> MOVE_APPROACH_MOVEJ
  -> MANUAL_ALIGN
  -> PRE_INSERT_MOVEL
  -> DONE
```

实际行为如下：

1. 连接 UR5e RTDE，读取当前 TCP 位姿和关节角。
2. 等待操作者确认工作空间安全。
3. 使用逆解得到关节目标，并用 `moveJ` 到达人工提供的孔前 TCP 位姿。
4. 使用方向键在孔前位姿的 TCP 工具坐标系 X/Y 平面内执行小步长 `moveL` 微调。
5. 操作者按 `P` 后，沿工具坐标系 Z 轴以低速执行 `moveL` 预插。
6. 记录 `actual_q`、`actual_tcp_pose`、TCP wrench，以及工具轴方向上的轴向/横向力。
7. 预插完成后结束。

本入口**不会调用** `ScrewClient.forward()`、`reverse()` 或 `set_speed()`，所以不会启动 DM4310 旋转。它用于先验证 TCP、坐标变换、运动方向、插入深度和接触力数据。

### `apps/screw_demo/ur_driver.py`

UR5e 的 RTDE 封装。UR RTDE 的单位保持原始接口约定：

```text
actual_q:           6 个关节角，rad
actual_tcp_pose:    [x, y, z, rx, ry, rz]
                    x/y/z 为 m，rx/ry/rz 为旋转向量 rad
TCP force:          [Fx, Fy, Fz, Tx, Ty, Tz]
                    力为 N，力矩为 N*m
moveJ speed/acc:    rad/s、rad/s^2
moveL speed/acc:    m/s、m/s^2
```

### `apps/screw_demo/screw_client.py`

通过 TCP Socket 向 `apps/screw_driver` 提供的 C++ 服务发送 JSON 命令。支持状态查询、设定速度、正反转、保持、心跳、清故障和急停。它已经可以被 LeRobot 记录流程使用，但基础 `main.py` 当前不会自动调用它。

### `apps/camera/`

`ring.py` 负责一次相机采集和圆环几何处理；`camera_service.py` 将相机能力封装为常驻 HTTP 服务。相机并行采集入口可以读取一次检测结果，结合标定文件生成机器人目标位姿；在没有完成手眼标定前，不应把默认占位位姿当作真实自动定位结果。

### `lerobot_robot_screw/scripts/recording/`

- `record_main_insert_dataset.py`：记录人工对准、预插和电批状态的 LeRobotDataset；确认后会通过 `ScrewClient` 控制电批。
- `record_camera_insert_dataset.py`：相机引导的单流程入口。
- `record_camera_insert_dataset_parallel.py`：并行启动相机检测与记录流程，并把相机结果转换为接近位姿。
- `record_camera_insert_dataset_inner_outer_line.py`：内外圆环线位姿实验入口。

## 运行前准备

1. UR5e 控制柜已上电、机械臂已解抱闸，并处于允许 RTDE 控制的远程控制模式。
2. 上位机与 UR5e 网线互通，确认机械臂 IP；当前默认值是 `192.168.1.5`。
3. 已在 PolyScope 中正确设置 TCP。`actual_tcp_pose` 是当前 TCP 位姿；如果 TCP 仍是法兰或工具尖端偏置错误，所有对孔和预插位置都会产生系统误差。
4. 真实运动前清空工作空间，确认速度、加速度、插入深度和工具 Z 方向。
5. 只有运行 LeRobot 电批流程时，才需要启动 CAN、DM4310 驱动板和 C++ Socket 服务。
6. 使用相机流程时，确认相机 IP、模型权重、输出目录和相机到机器人坐标标定文件。

## 推荐运行流程

### 1. 先做 dry-run 参数检查

从项目根目录运行：

```bash
python /home/sjh/Desktop/liuhao-main/apps/screw_demo/main.py --help
python /home/sjh/Desktop/liuhao-main/apps/screw_demo/main.py --config /home/sjh/Desktop/liuhao-main/apps/screw_demo/config/default.json
```

dry-run 不会连接真实 UR5e，也不会产生真实运动。它用于确认参数解析、状态机分支和力记录逻辑。

### 2. 读取 UR5e 状态

建议先运行 `apps/screw_demo/tests/test_read_state.py`，确认可以读到关节和 TCP 数据，再进入运动测试。不要在尚未验证通信、TCP 和工作空间的情况下直接运行完整流程。

### 3. 运行人工对准 + 预插 Demo

`--approach-pose` 的命令行输入为 `[X Y Z RX RY RZ]`，其中 XYZ 使用米，旋转向量使用弧度：

```bash
python /home/sjh/Desktop/liuhao-main/apps/screw_demo/main.py \
  --enable-robot \
  --robot-host 192.168.1.5 \
  --approach-pose X Y Z RX RY RZ \
  --insert-sign 1 \
  --insert-depth 0.002 \
  --insert-speed 0.003 \
  --force-log-dir /home/sjh/Desktop/liuhao-main/artifacts/logs/screw_demo
```

`config/default.json` 当前默认把力日志写入 `/home/sjh/Desktop/liuhao-main/artifacts/logs/screw_demo`；真实实验仍建议显式确认该目录，避免误把实验产物写入源码目录。

### 4. 运行 LeRobot 数据采集流程

这个入口的 CLI 将 XYZ 距离参数写成毫米，脚本内部再转换成 UR RTDE 使用的米：

```bash
python /home/sjh/Desktop/liuhao-main/lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py \
  --enable-robot \
  --robot-host 192.168.1.5 \
  --approach-pose X_MM Y_MM Z_MM RX RY RZ \
  --insert-depth 2 \
  --insert-speed 3 \
  --insert-sign 1 \
  --screw-host 127.0.0.1 \
  --screw-port 5055 \
  --screw-speed-rpm 60 \
  --dataset-root /home/sjh/Desktop/liuhao-main/artifacts/datasets/real_insert_dataset \
  --repo-id local/real_screw_insert_001 \
  --fps 20
```

该入口会在操作者按 `P` 后启动电批并执行预插，结束时发送 `hold`。只有在已完成空载、低速、短时间的 DM4310 独立测试后，才允许在真实工件上使用。

### 5. 运行相机并行采集

分隔符 `--` 前的参数属于相机并行层，分隔符后的参数传给 LeRobot 记录器：

```bash
python /home/sjh/Desktop/liuhao-main/lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py \
  --camera-ip 192.168.1.66 \
  --target-ring outer \
  --calibration-json /home/sjh/Desktop/liuhao-main/artifacts/calibration/camera_to_robot.json \
  -- \
  --enable-robot \
  --robot-host 192.168.1.5 \
  --insert-depth 2 \
  --insert-speed 3 \
  --insert-sign 1 \
  --screw-host 127.0.0.1 \
  --screw-port 5055 \
  --screw-speed-rpm 60 \
  --dataset-root /home/sjh/Desktop/liuhao-main/artifacts/datasets/real_insert_dataset \
  --repo-id local/real_insert_dataset \
  --fps 20
```

如果使用常驻相机服务，可把 `--camera-service-url http://127.0.0.1:5060/capture` 加在分隔符前，并先运行：

```bash
python /home/sjh/Desktop/liuhao-main/apps/camera/camera_service.py --host 127.0.0.1 --port 5060
```

## 键盘操作

`apps/screw_demo/main.py` 和 `record_main_insert_dataset.py` 的人工阶段使用相同的交互约定：

| 按键 | 作用 |
|---|---|
| `Enter` | 确认当前安全检查或接受人工对准结果 |
| `←` / `→` | TCP 工具坐标系 `-X` / `+X` 微调 |
| `↓` / `↑` | TCP 工具坐标系 `-Y` / `+Y` 微调 |
| `P` | 确认并执行预插；在 LeRobot 流程中同时允许电批开始旋转 |
| `Q` | 中止当前流程 |

默认微调步长为 `0.5 mm`，最大横向累计偏移为 `10 mm`。每次按键都会生成一个受限的 `moveL` 目标，而不是直接修改关节角。

## 主要参数

基础 Demo 的常用参数：

```text
--config              JSON 默认配置文件
--robot-host          UR5e IP
--enable-robot        连接真实 UR5e；不加则使用 dry-run
--approach-pose       孔前 TCP 位姿，XYZ=m，旋转向量=rad
--jog-step            单次 XY 微调距离，m
--max-align-offset    最大 XY 偏移半径，m
--insert-depth        预插深度，m
--insert-sign         工具 Z 方向，+1 或 -1
--insert-speed        预插速度，m/s
--joint-speed/acc     moveJ 速度/加速度，rad/s、rad/s^2
--jog-speed            moveL 微调速度，m/s
--force-log-dir       TCP 力 CSV 输出目录
--no-force-log        关闭基础 Demo 的力数据记录
```

LeRobot 记录器的常用参数：

```text
--dataset-root        数据集输出目录
--repo-id             LeRobot 数据集 ID
--raw-output-path     原始 JSONL 输出路径
--fps                 记录频率
--screw-host/port     电批 Socket 服务地址
--screw-speed-rpm     目标电批转速
--insert-depth        以 mm 输入的预插深度
--insert-speed        以 mm/s 输入的预插速度
```

完整参数以各入口的 `--help` 输出为准：

```bash
python /home/sjh/Desktop/liuhao-main/apps/screw_demo/main.py --help
python /home/sjh/Desktop/liuhao-main/lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py --help
python /home/sjh/Desktop/liuhao-main/lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_parallel.py --help
```

## 电批服务

在 Ubuntu 控制机上构建并启动 C++ 服务。具体 CAN 驱动选项以 `apps/screw_driver/README_screw_monitor.md` 和服务端 `--help` 为准：

```bash
cd /home/sjh/Desktop/liuhao-main/apps/screw_driver
cmake -S /home/sjh/Desktop/liuhao-main/apps/screw_driver -B /home/sjh/Desktop/liuhao-main/apps/screw_driver/build -DUSE_OPENARM_CAN=ON
cmake --build /home/sjh/Desktop/liuhao-main/apps/screw_driver/build

# 先配置 CAN。默认服务使用 CAN-FD；如果硬件只配置 classic CAN，启动服务时加 --no-can-fd。
sudo /home/sjh/Desktop/liuhao-main/third_party/openarm_can-1.1.0/setup/configure_socketcan.sh can0 -fd
ip -details link show can0

/home/sjh/Desktop/liuhao-main/apps/screw_driver/build/screw_server --openarm --can can0 --send-id 0x06 --recv-id 0x16 --motor-type 1 --port 5055
```

如果提示 `bind failed on port 5055`，说明已有服务占用端口，先用 `ss -ltnp 'sport = :5055'` 找到旧进程，确认后停止旧服务再启动。

Socket 服务启动后，先用 `screw_client.py` 或对应测试脚本做低速、短时、无负载测试。电批服务有通信超时保护，默认 0.5 秒内没有新运动命令或 heartbeat 会自动 hold，所以持续转动测试必须周期性发送 heartbeat。

```bash
PYTHONPATH=/home/sjh/Desktop/liuhao-main/apps/screw_demo python -c '
import time
from screw_client import make_screw_client, assert_ok

with make_screw_client("127.0.0.1", 5055) as client:
    print(assert_ok(client.status(), "status_before"))
    print(assert_ok(client.set_speed(60), "set_speed_60"))
    for _ in range(10):
        time.sleep(0.1)
        print(assert_ok(client.heartbeat(), "heartbeat"))
    print(assert_ok(client.hold(), "hold"))
    print(assert_ok(client.status(), "status_after"))
'
```

## 当前限制

- `apps/screw_demo/main.py` 不接视觉，不自动估计孔位，不控制 DM4310 旋转。
- 相机到机器人坐标转换依赖手眼/外参标定；没有有效 `--calibration-json` 时，相机流程不能视为可靠的自动定位。
- 当前预插是固定深度和固定速度的 `moveL`，不是闭环力控，也没有基于力阈值的自动停止。
- 当前还没有完成真正的螺丝供料、批头与螺丝啮合检测、拧紧扭矩闭环和拧紧完成判定。
- LeRobot 采集已经覆盖机器人状态、TCP 力和电批状态接口，但数据质量仍依赖人工动作、时间同步和传感器标定。
- `full-auto` 只适合受控的启动耗时或接口实验；没有完成安全验证前，不要在真实机械臂上使用。

## 安全注意事项

真实 UR5e 运动前必须确认急停可用、TCP 正确、工具 Z 方向正确、目标位姿可达、周围没有人员和碰撞障碍。第一次测试使用空载、低速、短距离：建议先把 `--insert-depth` 设为 1--2 mm，并在每个确认点观察机械臂实际姿态。

如果发生异常，使用 `Q` 或 Ctrl+C 停止流程，并准备使用机器人控制柜急停。不要把 `actual_tcp_pose` 直接当作电批批头尖端位姿；批头相对法兰的安装偏置必须通过 TCP 标定建立。不要把基础 Demo 的“预插成功”理解为“螺丝已经拧紧”。

## 后续开发顺序

```text
TCP/工具坐标验证
-> DM4310 独立转动和故障处理
-> 预插接触检测和力控
-> 相机坐标标定与自动定位
-> 螺丝旋拧闭环
-> LeRobot 多模态示范数据
-> 模仿学习训练与部署
```

更完整的工程整理、环境、输出和测试说明见 [项目说明.md](项目说明.md)。
