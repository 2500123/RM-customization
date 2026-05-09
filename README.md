## 1. 项目简介

RoboMaster 2026 **英雄部署模式**下，官方图传和自定义客户端图传均被切断，但自定义数据流不受影响。

```
海康相机 → H.264 编码 → VideoPacket (ROS 2) → 串口 → 下位机 → 选手端 → 自定义客户端
```

| 特性 | 说明 |
|------|------|
| 链路 | 串口 (下位机 `0x0310`) → 裁判系统 MQTT `CustomByteBlock`，300B/pkt|
| 编码 | H.264 (x264)，`slower`  |
| 码率 | 目标 12 kB/s，硬限制 14 kB/s |

**项目结构**
```
Pacific_doorlock_sniper/
├── src/
│   ├── bringup/                     # ROS 2 launch
│   ├── hik_camera/                  # 海康相机驱动 (C++)
│   ├── doorlock_sniper/             # H.264 编码器 (C++)
│   └── doorlock_decoder/            # H.264 解码器 (Python)
├── tools/                           # 工具集
│   ├── custom_byteblock.proto       # Protobuf 定义
│   ├── custom_byteblock_pb.py       # Protobuf 辅助 (含回退)
│   ├── custom_byteblock_codec.py    # 片段头编解码
│   ├── serial_bridge.py             # 串口桥接 (生产链路)
│   ├── ros2_mqtt_bridge.py          # MQTT 桥接 (本地测试)
│   ├── mqtt_custombyteblock_sender.py  # 独立发送端 (无 ROS)
│   ├── pyqt_custombyteblock_viewer.py  # PyQt 接收端
│   └── udp_hevc_receiver.py         # UDP HEVC 接收器
├── deploy/                          # 开机自启部署文件
└── build/ install/ log/
```
## 2. 链路架构

```
场上机器人                                    自定义客户端 PC
┌────────────────────────────────────┐      ┌─────────────────────────┐
│ 海康相机                            │      │                         │
│   ↓ USB                            │      │ pyqt_custombyteblock_   │
│ hik_camera → video_encoder         │      │   viewer.py             │
│              ↓ H.264, 280B/pkt     │      │   ↑ MQTT subscribe      │
│       serial_bridge.py             │      │   CustomByteBlock       │
│              ↓ 串口帧              │      │   --mode h264_stream    │
│         下位机 (STM32)              │      │                         │
│          收到 cmd=0x0310 后         │      │                         │
│          MQTT publish CustomByteBlock│    │                         │
│              ↓                     │      │                         │
│         裁判系统 MQTT Broker        │ ───▶ │ 192.168.12.1:3333       │
│         192.168.12.1:3333          │ 网线 │                         │
└────────────────────────────────────┘      └─────────────────────────┘
```
## 3. 环境准备

**系统要求：** Ubuntu 22.04+ · ROS 2 Humble · 海康 MVS SDK (`/opt/MVS/include` + `/opt/MVS/lib/64`)

```bash
# 基础依赖
sudo apt install -y mosquitto build-essential cmake pkg-config \
    python3-colcon-common-extensions python3-rosdep \
    python3-opencv python3-av python3-paho-mqtt \
    python3-protobuf protobuf-compiler libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav

pip3 install --user PyQt5 pyserial

# ROS 依赖 + 编译 protobuf
rosdep install --from-paths src --ignore-src -r -y
cd tools && protoc --python_out=. custom_byteblock.proto && cd ..
```
> Mosquitto 仅本地测试需要。Protobuf 编译为可选，`pb.py` 自动回退。

## 4. 使用方式

### 4.1 生产链路（比赛用）

```bash
#1: ROS 管线 (相机 + H.264 编码)
colcon build && source install/setup.bash
ros2 launch bringup sniper.launch.py

#2: 串口桥接
source install/setup.bash
python3 tools/serial_bridge.py     --device /dev/stm32_uart     --baud 921600 --robot-id 1     --no-keyframe-filter     --send-rate 45 --redundancy 1     --print-stats

#3:接收
python3 tools/pyqt_custombyteblock_viewer.py \
    --mode h264_stream --host 192.168.12.1 --robot-id 1 --print-stats
```

### 4.3 本地测试

```bash
# 终端 1: MQTT Broker
mosquitto -p 3333

# 终端 2: ROS 管线
source install/setup.bash
ros2 launch bringup sniper.launch.py

# 终端 3: MQTT 桥接 (替代串口)
source install/setup.bash
python3 tools/ros2_mqtt_bridge.py --robot-id 1 --host 127.0.0.1 --print-stats

# 终端 4: 接收端
python3 tools/pyqt_custombyteblock_viewer.py \
    --mode h264_stream --host 127.0.0.1 --robot-id 1 --print-stats
```
### 4.5 UDP HEVC 接收

```bash
python3 tools/pyqt_custombyteblock_viewer.py --mode hevc_udp
```
## 5. ROS 功能包

| 功能包 | 节点 | 说明 |
|--------|------|------|
| `hik_camera` | `hik_camera` | 海康相机 → `/image_raw` |
| `doorlock_sniper` | `video_encoder` | H.264 编码 + 运动检测 → `/video_stream` (280B) |

`sniper.launch.py` 可调参数见文件内注释。相机代码源自 [rm-vision](https://github.com/chenjunnn/rm_vision)。
## 6. 部署

开机自启 systemd 服务文件及部署教程见 `deploy/README.md`。

## 7. 数据格式

**MQTT CustomByteBlock:** `0x0A + varint(len) + data`，data 为 `8B 片段头 + 280B H.264 + 12B 补零` = **300B**，protobuf 编码后约 303B。

**串口帧:** 5B 帧头 `[A5][2C 01][seq][CRC8]` + cmd_id `[10 03]` + data `300B` = **307B**，cmd_id=0x0310。

**UDP HEVC:** 8B 帧头 `frame_id:2B frag_idx:2B total_len:4B` (大端) + HEVC 数据。
 
 ping 192.168.12.1 
 nc -zv 192.168.12.1 3333
 