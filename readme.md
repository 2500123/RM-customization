# RoboMaster Sniper — 快速参考

---

## 比赛操作

### 场景 A：英雄部署模式（图传被切 → MQTT CustomByteBlock）★ 主要场景

**发送端（小电脑，已配 systemd 自启则无需手动操作）**

```bash
# 终端 1: ROS 管线（相机 + H.264 编码）
source install/setup.bash
ros2 launch bringup sniper.launch.py

# 终端 2: 串口桥接（ROS → 下位机）
source install/setup.bash
python3 tools/serial_bridge.py --device /dev/ttyACM0 --baud 230400 --robot-id 1 --print-stats
```

**接收端（自定义客户端 PC）**

```bash
python3 tools/pyqt_custombyteblock_viewer.py \
    --mode h264_stream --host 192.168.12.1 --robot-id 1 --print-stats
```

### 场景 B：正常模式（图传正常 → UDP HEVC）

**发送端**：无需额外操作，机器人官方固件自动推送。

**接收端（命令行版，轻量）**

```bash
python3 tools/udp_hevc_receiver.py --display
```

**接收端（PyQt GUI 版）**

```bash
python3 tools/pyqt_custombyteblock_viewer.py --mode hevc_udp
```

> 比赛时两个 receiver 可以同时跑，正常时看 UDP HEVC，进英雄模式自动切看 MQTT。

---

## 本地测试

### 1. MQTT H.264（模拟英雄部署模式）

```bash
# 终端 1: MQTT Broker
mosquitto -p 3333

# 终端 2: ROS 管线
source install/setup.bash
ros2 launch bringup sniper.launch.py

# 终端 3: MQTT 桥接（替代串口）
source install/setup.bash
python3 tools/ros2_mqtt_bridge.py --robot-id 1 --host 127.0.0.1 --print-stats

# 终端 4: 接收端
python3 tools/pyqt_custombyteblock_viewer.py \
    --mode h264_stream --host 127.0.0.1 --robot-id 1 --print-stats
```

### 2. UDP HEVC（模拟正常图传）

```bash
# 本地测试 (裸流模式)
python3 tools/udp_hevc_receiver.py --raw --display

# 比赛接收 (RM 官方图传，带 8B 帧头)
python3 tools/udp_hevc_receiver.py --display
```

### 3. 独立发送端（无 ROS，USB 摄像头）

```bash
python3 tools/mqtt_custombyteblock_sender.py \
    --mode h264_camera --camera 0 --width 300 --height 300 \
    --h264-bitrate 80000 --h264-preset ultrafast --cam-fps 15 \
    --send-hz 50 --robot-id 1 --host 127.0.0.1 --print-stats
```

---

## 工具速查

| 工具 | 用途 | 需要 ROS |
|------|------|---------|
| `serial_bridge.py` | 串口桥接 (生产) | ✅ |
| `ros2_mqtt_bridge.py` | MQTT 桥接 (本地测试) | ✅ |
| `mqtt_custombyteblock_sender.py` | 独立发送端 | ❌ |
| `pyqt_custombyteblock_viewer.py` | PyQt 接收 (两种模式) | ❌ |
| `udp_hevc_receiver.py` | 命令行 HEVC 接收 | ❌ |

| PyQt viewer 模式 | 数据源 | 赛事场景 |
|------------------|--------|---------|
| `--mode h264_stream` | MQTT CustomByteBlock | 英雄部署模式 |
| `--mode hevc_udp` | UDP :3334 HEVC | 正常模式 |

---

## 部署（systemd 开机自启）

### 发送端（小电脑）

```
deploy/
├── rm-sniper-ros.service       # ROS 管线自启
├── rm-sniper-serial.service    # 串口桥接自启 (依赖 ROS 先就绪)
└── start-ros-pipeline.sh       # ROS launch 启动脚本
```

```bash
cd deploy
chmod +x start-ros-pipeline.sh
sudo cp rm-sniper-ros.service rm-sniper-serial.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rm-sniper-ros rm-sniper-serial
sudo systemctl start rm-sniper-ros rm-sniper-serial
```

### 接收端（自定义客户端 PC）

```
deploy/
└── rm-sniper-viewer.service    # PyQt GUI 自启
```

```bash
cd deploy
sudo cp rm-sniper-viewer.service /etc/systemd/user/
systemctl --user daemon-reload
systemctl --user enable rm-sniper-viewer.service
systemctl --user start rm-sniper-viewer.service
```

---

## 串口帧格式（下位机参考）

```
[0xA5] [data_len:2B LE=0x012C] [seq:1B] [CRC8:1B] [cmd_id:2B=0x0310 LE] [data:300B]  整帧307B
                                       ↑
                          8B 片段头 + 280B H.264 + 12B 补零
```

> 下位机收到 `cmd=0x0310` → `MQTT_Publish("CustomByteBlock", data, 288, 0)` 原样转发。
systemctl --user start rm-sniper-viewer.service
```

---

## 链路

```
小电脑 --串口--> 下位机 --内部--> 裁判系统 MQTT --网线--> 自定义客户端 PC
   ↑               ↑                    ↑                    ↑
serial_bridge  收到 cmd=0x0310    192.168.12.1:3333     pyqt viewer
               后 MQTT publish     CustomByteBlock
```

## 下位机固件

收到 `cmd_id=0x0310` 的串口帧后，`data` 字段原样 `MQTT_Publish("CustomByteBlock", data, len, 0)`。

详见 `README.md` 第 7 节及 `deploy/README.md`。