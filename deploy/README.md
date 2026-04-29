# RoboMaster Sniper — 部署文档

## 物理链路

```
小电脑 --串口--> 下位机(STM32) --内部--> 裁判系统(MQTT) --网线--> 自定义客户端 PC
         ↑                                   ↑                       ↑
  serial_bridge.py                    192.168.12.1:3333        pyqt viewer
  VideoPacket→串口帧                   CustomByteBlock
```

> 场上小电脑**无法连网**，只能串口通信。

---

## 文件说明

| 文件 | 部署位置 | 用途 |
|------|----------|------|
| `rm-sniper-ros.service` | 小电脑 | ROS 管线 (hik_camera + video_encoder) |
| `rm-sniper-serial.service` | 小电脑 | 串口桥接 (ROS → 下位机) |
| `rm-sniper-viewer.service` | 自定义客户端 PC | PyQt 接收端 |
| `start-ros-pipeline.sh` | 小电脑 | ROS launch 启动脚本 |

---

## 一、小电脑（发送端）部署

### 1.1 依赖

- ROS 2 Humble + 海康 MVS SDK
- 已 `colcon build`，项目路径 `/home/hyc/002/Pacific_doorlock_sniper`
- 串口: `pip3 install pyserial`

### 1.2 安装

```bash
cd /home/hyc/002/Pacific_doorlock_sniper/deploy
chmod +x start-ros-pipeline.sh

sudo cp rm-sniper-ros.service    /etc/systemd/system/
sudo cp rm-sniper-serial.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rm-sniper-ros rm-sniper-serial
```

### 1.3 修改串口号

```bash
sudo systemctl edit rm-sniper-serial
# 修改 --device /dev/ttyXXX 为实际串口
```

### 1.4 手动管理

```bash
sudo systemctl start rm-sniper-ros rm-sniper-serial
sudo systemctl status rm-sniper-ros rm-sniper-serial
journalctl -u rm-sniper-serial -f
```

---

## 二、自定义客户端 PC（接收端）部署

```bash
cd /home/hyc/002/Pacific_doorlock_sniper/deploy
sudo cp rm-sniper-viewer.service /etc/systemd/user/
systemctl --user daemon-reload
systemctl --user enable rm-sniper-viewer.service
systemctl --user start rm-sniper-viewer
```

---

## 三、串口帧格式

```
[SOF:0xA5] [data_length:2B LE] [seq:1B] [CRC8:1B] [cmd_id:2B=0x0310 LE] [data:N B]
```
> 帧头 5B 遵循官方协议 V1.3.0，PC 侧补零到 300B，整帧 = 5 + 2 + 300 = 307B

`data` 为 300B（8B 片段头 + 280B H.264 + 12B 补零），下位机直接作为 `CustomByteBlock.data` MQTT publish。

**下位机固件伪代码:**

```c
if (cmd_id == 0x0310) {
    MQTT_Publish("CustomByteBlock", data, data_len, 0);
}
```

---

## 四、本地测试（MQTT 直连，仅开发用）

无需下位机，小电脑直接连 MQTT Broker:

```bash
mosquitto -p 3333
ros2 launch bringup sniper.launch.py
python3 tools/ros2_mqtt_bridge.py --host 127.0.0.1 --print-stats
python3 tools/pyqt_custombyteblock_viewer.py --mode h264_stream --host 127.0.0.1
```
