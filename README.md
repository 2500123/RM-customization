## 1. 项目简介

RoboMaster 2026 **英雄部署模式**下，官方图传和自定义客户端图传均被切断，但 **`CustomByteBlock` 自定义数据流不受影响**。

场上小电脑无法连接热点/网线，只能通过 **串口→下位机→裁判系统** 传输数据。本项目用该链路实现低带宽落点图传：

```
海康相机 → H.264 编码 → VideoPacket (ROS 2) → 串口 → 下位机 → 裁判系统 MQTT → 自定义客户端 PC
```

| 特性 | 说明 |
|------|------|
| 链路 | 串口 (下位机 `0x0310`) → 裁判系统 MQTT `CustomByteBlock`，300B/pkt，burst ~15 chunk/帧 |
| 编码 | H.264 (x264)，`fast` 平衡延迟与压缩 |
| 码率 | 目标 10 kB/s，硬限制 14 kB/s |
| 关键帧模式 | SPS/PPS 优先发送，P 帧过滤，节流可调 |

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
│   ├── serial_bridge.py             # ★ 串口桥接 (生产链路)
│   ├── ros2_mqtt_bridge.py          # MQTT 桥接 (本地测试)
│   ├── mqtt_custombyteblock_sender.py  # 独立发送端 (无 ROS)
│   ├── pyqt_custombyteblock_viewer.py  # PyQt 接收端
│   └── udp_hevc_receiver.py         # UDP HEVC 接收器
├── deploy/                          # 开机自启部署文件
└── build/ install/ log/
```

---

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

> **场上小电脑无法连网**，数据路径: 小电脑 → 串口 → 下位机 → 裁判系统 → MQTT → 自定义客户端。
>
> **本地测试** 用 `ros2_mqtt_bridge.py` 替代串口链路，小电脑直接连 mosquitto。见 [4.3 本地测试](#43-本地测试mqtt-直连)。

---

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

---

## 4. 使用方式

### 4.1 生产链路（比赛用）

```bash
# 终端 1: ROS 管线 (相机 + H.264 编码)
colcon build && source install/setup.bash
ros2 launch bringup sniper.launch.py

# 终端 2: 串口桥接 (ROS → 下位机, 关键帧模式)
# --send-rate 串口速率上限  --chunk-delay-ms burst 内 chunk 间隔
# --keyframe-interval 节流间隔 (0=不节流 ~5fps)
source install/setup.bash
python3 tools/serial_bridge.py \
    --device /dev/ttyACM0 --baud 921600 --robot-id 1 \
    --send-rate 40 --redundancy 1 --chunk-delay-ms 20 \
    --keyframe-interval 0.25 --print-stats

#### 4.1.1 提高画质/帧率（发送端 CPU 有余量时）

>
> 生产默认用 `serial_bridge.py` 的“关键帧过滤”模式：主要发送 **SPS/PPS + IDR(I 帧)**，大量 **P 帧会被丢弃**，因此帧率会被 `--keyframe-interval` 直接限制（常见 ~4fps）。

**方案 A：保持带宽不变，显著提高 fps（推荐优先尝试）**

- 关闭关键帧过滤，改为“全量发送”（SPS/PPS + I/P 全部发），让 H.264 的帧间预测真正生效：

```bash
python3 tools/serial_bridge.py \
    --device /dev/ttyACM0 --baud 921600 --robot-id 1 \
    --no-keyframe-filter \
    --send-rate 45 --redundancy 1 \
    --print-stats
```
```bash
python3 tools/serial_bridge.py \
    --device /dev/ttyCH341USB0 \
    --baud 921600 --robot-id 1 \
    --no-keyframe-filter \
    --send-rate 45 --redundancy 1 \
    --print-stats
```

说明：

- `--send-rate` 单位是 **pkt/s**（每包约 307B），`45 pkt/s ≈ 13.8 kB/s`，刚好贴近 `sniper.launch.py` 默认的 `hard_max_bitrate_kbytes=14`。
- 若观察到 `skipped` 增长（限速跳包），可把 `--send-rate` 小幅上调；若下位机/裁判系统处理不过来导致丢包，反向下调。

**方案 B：用更多 CPU 换同码率更好画质（压缩效率提升）**

- 在 [src/bringup/launch/sniper.launch.py](src/bringup/launch/sniper.launch.py) 里把 `x264_preset` 从 `fast` 调慢到 `medium/slow`（CPU 会上升，画质一般会更稳）：
    - 画质优先：`slow`
    - 均衡：`medium`

同时建议把编码器的 `output_fps` 调到一个“链路能承载”的值（例如 10~20，当前 launch 默认 20）：

- 当前编码器内部有 `bandwidth_limit_kbytes` 限速窗口；当 `output_fps` 过高但带宽固定时，编码器会把更多帧编码出来，但最终会被限速窗口挡住并在队列超时后丢弃（既浪费 CPU，也会引入画面不稳定）。
- 将 `output_fps` 设为可承载范围后，再用更慢的 `x264_preset` 去换压缩效率，收益更直接。

**方案 C：在带宽允许时，同时拉高帧率与清晰度**

- 先用方案 A 解除“只发关键帧”的限制；
- 再按链路余量小幅上调编码码率：把 [src/bringup/launch/sniper.launch.py](src/bringup/launch/sniper.launch.py) 里的 `target_bitrate_kbytes` 从 `10.0` 提到 `12.0~14.0`；
- 同时确保编码器侧 `bandwidth_limit_kbytes` 与发送端 `--send-rate` 不成为瓶颈（目标是让编码器日志里的 `TX stats avg` 接近但不超过上限）。

**经验法则**

- 想“更清晰”但带宽不变：优先调慢 `x264_preset`（fast→medium/slow）。
- 想“更流畅”：优先关闭关键帧过滤（`--no-keyframe-filter`），比盲目加码率更有效。
- 想“两者都上去”：只能在链路允许的前提下提高 `target_bitrate_kbytes / send-rate`。
```

**串口帧格式 (cmd=0x0310，机器人自定义数据上传):**

```
[SOF:0xA5] [data_length:2B LE] [seq:1B] [CRC8:1B] [cmd_id:2B=0x0310 LE] [data:N B]
```
> 帧头 5B 遵循官方协议 V1.3.0，CRC8 多项式 x^8+x^5+x^4+1 初始 0xFF，仅校验前 4 字节

> **下位机固件需实现**: 收到 `cmd_id=0x0310` 后将 `data` 作为 `CustomByteBlock.data`，通过 MQTT publish 到 `CustomByteBlock` topic。

### 4.2 接收端（自定义客户端 PC）

```bash
python3 tools/pyqt_custombyteblock_viewer.py \
    --mode h264_stream --host 192.168.12.1 --robot-id 1 --print-stats
```

### 4.3 本地测试（MQTT 直连）

无下位机环境时，小电脑通过 MQTT 直接发 → mosquitto → viewer:

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

### 4.4 无 ROS 测试

```bash
# 独立发送端 (摄像头采集 + 编码 + MQTT，一条命令)
python3 tools/mqtt_custombyteblock_sender.py \
    --mode h264_camera --camera 0 --width 300 --height 300 \
    --h264-bitrate 80000 --h264-preset ultrafast --cam-fps 15 \
    --send-hz 50 --robot-id 1 --host 127.0.0.1 --print-stats
```

### 4.5 UDP HEVC 接收（非英雄模式）

```bash
python3 tools/pyqt_custombyteblock_viewer.py --mode hevc_udp
#python3 tools/udp_hevc_receiver.py --display --save-dir ./frames
```

---

## 5. ROS 功能包

| 功能包 | 节点 | 说明 |
|--------|------|------|
| `hik_camera` | `hik_camera` | 海康相机 → `/image_raw` |
| `doorlock_sniper` | `video_encoder` | H.264 编码 + 运动检测 → `/video_stream` (280B) |
| `doorlock_decoder` | `video_decoder` | PyAV 解码 + OpenCV 显示 |

`sniper.launch.py` 可调参数见文件内注释。相机代码源自 [rm-vision](https://github.com/chenjunnn/rm_vision)。

---

## 6. 部署

开机自启 systemd 服务文件及部署教程见 `deploy/README.md`。

## 7. 数据格式

**MQTT CustomByteBlock:** `0x0A + varint(len) + data`，data 为 `8B 片段头 + 280B H.264 + 12B 补零` = **300B**，protobuf 编码后约 303B。

**串口帧:** 5B 帧头 `[A5][2C 01][seq][CRC8]` + cmd_id `[10 03]` + data `300B` = **307B**，cmd_id=0x0310。
> PC 侧补零到 300B，MCU 直转无需再补

**UDP HEVC:** 8B 帧头 `frame_id:2B frag_idx:2B total_len:4B` (大端) + HEVC 数据。
 
 ping 192.168.12.1 
 nc -zv 192.168.12.1 3333
 