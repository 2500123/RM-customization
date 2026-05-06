# RoboMaster Sniper — 部署文档

## 文件说明

| 文件 | 部署位置 | 用途 |
|------|----------|------|
| `rm-sniper-ros.service` | 小电脑 | ROS 管线 (hik_camera + video_encoder) |
| `rm-sniper-serial.service` | 小电脑 | 串口桥接 (ROS → 下位机) |
| `rm-sniper-viewer.service` | 自定义客户端 PC | PyQt 接收端 |
| `start-ros-pipeline.sh` | 小电脑 | ROS launch 启动脚本 |

---

## 一、小电脑（发送端）

### 安装

```bash
cd <项目路径>/deploy
chmod +x start-ros-pipeline.sh

sudo cp rm-sniper-ros.service    /etc/systemd/system/
sudo cp rm-sniper-serial.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rm-sniper-ros rm-sniper-serial
```

### 修改串口设备（推荐使用 udev symlink）

```bash
sudo systemctl edit rm-sniper-serial
# 修改 ExecStart 行中的 --device ...
# 推荐：用 udev 规则创建固定设备名（例如 /dev/stm32_uart），避免 ttyACM0/ttyACM1 重枚举导致服务写入失效。
```

### 手动启动

```bash
sudo systemctl start rm-sniper-ros rm-sniper-serial
```

### 查看状态 / 日志

```bash
sudo systemctl status rm-sniper-ros rm-sniper-serial
journalctl -u rm-sniper-ros -f
journalctl -u rm-sniper-serial -f
```

### 停止

```bash
sudo systemctl stop rm-sniper-serial rm-sniper-ros
```

### 取消开机自启

```bash
sudo systemctl disable rm-sniper-ros rm-sniper-serial
sudo rm /etc/systemd/system/rm-sniper-ros.service
sudo rm /etc/systemd/system/rm-sniper-serial.service
sudo systemctl daemon-reload
```

## 二、自定义客户端 PC（接收端）

### 安装

```bash
cd <项目路径>/deploy
sudo cp rm-sniper-viewer.service /etc/systemd/user/
systemctl --user daemon-reload
systemctl --user enable rm-sniper-viewer.service
systemctl --user start rm-sniper-viewer
```

### 停止

```bash
systemctl --user stop rm-sniper-viewer
```

### 取消开机自启

```bash
systemctl --user disable rm-sniper-viewer
rm /etc/systemd/user/rm-sniper-viewer.service
systemctl --user daemon-reload
```
