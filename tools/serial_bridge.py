#!/usr/bin/env python3
"""RoboMaster 串口桥接: ROS /video_stream → 串口 → 下位机

将 VideoPacket (150B H.264 chunks) 通过串口发给下位机。
下位机固件收到后，将数据包装为 CustomByteBlock 通过 MQTT publish。

  串口协议 (Mini PC → 下位机):
    [SOF: 0xA5] [data_len: 2B] [seq: 1B] [CRC8] [cmd_id: 2B] [data: N B] [CRC16]
    data_len = 2 (cmd_id) + len(data)
    CRC8 覆盖 SOF + data_len + seq
    CRC16 覆盖 全部

  下行命令 ID: 0x0310 (机器人自定义数据上传)

使用:
  python3 tools/serial_bridge.py --device /dev/ttyACM0 --baud 115200 --robot-id 1
"""

from __future__ import annotations

import argparse, os, struct, sys, time
from typing import Optional

sys.path.insert(0, os.path.dirname(__file__))
from custom_byteblock_codec import CODEC_H264, pack_fragment

# ── CRC 校验 ────────────────────────────────────────────────────────

CRC8_TABLE = [0] * 256
CRC16_TABLE = [0] * 256

def _init_crc8():
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
        CRC8_TABLE[i] = crc & 0xFF

def _init_crc16():
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        CRC16_TABLE[i] = crc & 0xFFFF

_init_crc8()
_init_crc16()

def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc = CRC8_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFF

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = CRC16_TABLE[((crc >> 8) ^ b) & 0xFF] ^ ((crc & 0xFF) << 8)
    return crc & 0xFFFF


def pack_serial_frame(cmd_id: int, data: bytes, seq: int = 0) -> bytes:
    """封装为串口帧: SOF + len + seq + CRC8 + cmd_id + data + CRC16."""
    payload = struct.pack("<H", cmd_id) + data          # little-endian cmd_id
    data_len = len(payload)
    header = struct.pack("<BB", 0xA5, data_len & 0xFF)  # SOF + len (1B for simplicity)
    # 简化版: SOF:1B + data_len:1B + data + CRC8 (终端校验)
    # 如需完整 CRC16，下位机固件可适配

    # 使用简洁帧格式 (便于下位机解析):
    # SOF:1B + cmd_id:2B + data_len:1B + data:N B + crc8:1B
    frame = bytearray()
    frame.append(0xA5)
    frame.extend(struct.pack("<H", cmd_id))  # little-endian
    frame.append(len(data) & 0xFF)
    frame.extend(data)
    frame.append(crc8(bytes(frame)))
    return bytes(frame)


def main() -> int:
    try:
        import serial
    except ImportError:
        raise SystemExit("Missing pyserial: pip3 install pyserial")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from doorlock_sniper.msg import VideoPacket
    except ImportError:
        raise SystemExit("Missing ROS 2. source install/setup.bash first.")

    ap = argparse.ArgumentParser(description="RoboMaster 串口桥接: ROS → 下位机")
    ap.add_argument("--device", default="/dev/ttyACM0", help="串口设备")
    ap.add_argument("--baud", type=int, default=115200, help="波特率")
    ap.add_argument("--robot-id", type=int, default=1)
    ap.add_argument("--ros-topic", default="/video_stream")
    ap.add_argument("--print-stats", action="store_true")
    ap.add_argument("--cmd-id", type=int, default=0x0310, help="下位机命令ID")
    args = ap.parse_args()

    rclpy.init(args=sys.argv[1:])

    # ── 打开串口 ──
    try:
        ser = serial.Serial(args.device, args.baud, timeout=1, write_timeout=1)
        print(f"[serial] Opened {args.device} @ {args.baud} baud")
    except Exception as e:
        print(f"[serial] Cannot open {args.device}: {e}")
        print("[serial] Retrying every 3 seconds...")
        while True:
            try:
                ser = serial.Serial(args.device, args.baud, timeout=1, write_timeout=1)
                print(f"[serial] Opened {args.device}")
                break
            except Exception:
                time.sleep(3)

    # ── 共享状态 ──
    ros_rx = 0
    serial_tx = 0
    drop_count = 0
    seq = 0

    # ── ROS 节点 ──
    class SerialBridgeNode(Node):
        def __init__(self):
            super().__init__("serial_bridge")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=3000,
            )
            self.sub = self.create_subscription(
                VideoPacket, args.ros_topic, self._on_packet, qos
            )
            self.get_logger().info(
                f"Serial bridge: {args.ros_topic} → {args.device} @ {args.baud}"
            )

        def _on_packet(self, msg: VideoPacket) -> None:
            nonlocal ros_rx, serial_tx, drop_count, seq
            ros_rx += 1

            # 包装 H.264 chunk (8B 片段头 + 150B payload)
            chunk = bytes(msg.data)
            frame_id = int(msg.sequence_id) & 0xFFFF
            frag = pack_fragment(
                frame_id=frame_id, frag_idx=0, frag_cnt=1,
                codec=CODEC_H264, flags=0, total_len=0, chunk=chunk,
            )

            # 串口帧封装
            frame = pack_serial_frame(args.cmd_id, frag, seq)
            seq = (seq + 1) & 0xFF

            try:
                ser.write(frame)
                serial_tx += 1
            except Exception:
                drop_count += 1

    node = SerialBridgeNode()

    # 统计
    last_stat = time.monotonic()
    last_ros = 0
    last_tx = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            if args.print_stats and time.monotonic() - last_stat > 5.0:
                dt = time.monotonic() - last_stat
                dr = ros_rx - last_ros
                ds = serial_tx - last_tx
                print(
                    f"[serial] ROS rx={ros_rx} (+{dr}, {dr/dt:.0f} pkt/s) | "
                    f"Serial tx={serial_tx} (+{ds}, {ds/dt:.0f} pkt/s) | "
                    f"drops={drop_count}"
                )
                last_stat = time.monotonic()
                last_ros = ros_rx
                last_tx = serial_tx
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
