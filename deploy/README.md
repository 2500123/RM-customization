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
cd /deploy
chmod +x start-ros-pipeline.sh

sudo cp rm-sniper-ros.service    /etc/systemd/system/
sudo cp rm-sniper-serial.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rm-sniper-ros rm-sniper-serial
```

### 修改串口设备（推荐使用 udev symlink）

```bash
参考同济
串口设置
    1. 授予权限
        ```
        sudo usermod -a -G dialout $USER
        ```
    2. 获取端口 ID（serial, idVendor, idProduct）
        ```
        udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'
        ```
        将 /dev/ttyACM0 替换为实际设备名。
    3. 创建 udev 规则文件
        ```
        sudo touch /etc/udev/rules.d/99-usb-serial.rules
        ```
        然后在文件中写入如下内容（用真实 ID 替换示例，SYMLINK 是规则应用后固定的串口名）：
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="1234", ATTRS{serial}=="A1234567", SYMLINK+="gimbal"
        ```
    4. 重新加载 udev 规则
        ```
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        ```
    5. 检查结果
        ```
        ls -l /dev/gimbal
        # Expected output (example):
        # lrwxrwxrwx 1 root root 7 Jul 21 10:00 /dev/gimbal -> ttyACM0
```

### 手动启动

```bash
sudo systemctl start rm-sniper-ros rm-sniper-serial
```

### 查看状态 / 日志

```bash
sudo systemctl status rm-sniper-ros 
sudo systemctl status rm-sniper-ros
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

### 创建

```bash
mkdir -p ~/.config/autostart
nano ~/.config/autostart/rm-sniper-viewer.desktop

写入
[Desktop Entry]
Type=Application
Name=RoboMaster Sniper Viewer
Exec=/usr/bin/python3 /home/hyc/002/Pacific_doorlock_sniper/tools/pyqt_custombyteblock_viewer.py --mode h264_stream --host 192.168.12.1 --robot-id 1 --print-stats
X-GNOME-Autostart-enabled=true

```
 ### 停止
 
 ```bash
 pkill -f pyqt_custombyteblock_viewer.py
 ```

  ### 关闭

  ```bash
  rm ~/.config/autostart/rm-sniper-viewer.desktop
  ```
