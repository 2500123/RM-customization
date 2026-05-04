#!/usr/bin/env python3
"""RoboMaster 串口桥接: ROS /video_stream → 串口 → 下位机

将 VideoPacket (280B H.264 chunks) 通过串口发给下位机。
下位机固件收到后，将数据包装为 CustomByteBlock 通过 MQTT publish。

  串口协议 (Mini PC → 下位机) 遵循官方文档 V1.3.0:
    frame_header: 5B  = SOF(0xA5) + data_length(2B LE) + seq(1B) + CRC8(1B)
    cmd_id: 2B LE (0x0310)
    data: 288B 片段 (8B 片段头 + 280B H.264 chunk) + 12B 补零 = 300B

        CRC16: 2B LE (对整帧 SOF~data 的校验)

    整帧长度 = frame_header 5B + cmd_id 2B + data 300B + CRC16 2B = 309 字节

使用:
  python3 tools/serial_bridge.py  --baud 921600 --robot-id 1
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(__file__))
from custom_byteblock_codec import CODEC_H264, pack_fragment

# ── H.264 NAL start-code matching ──────────────────────────────────────
_ST4 = b"\x00\x00\x00\x01"
_ST3 = b"\x00\x00\x01"


def _has_sps_pps(data: bytes) -> bool:
    """Return True if *data* contains an SPS (type 7) or PPS (type 8) NAL unit."""
    n = len(data)
    i = 0
    while i < n - 3:
        if i + 4 <= n and data[i:i + 4] == _ST4:
            if i + 5 <= n and (data[i + 4] & 0x1F) in (7, 8):
                return True
            i += 5
        elif data[i:i + 3] == _ST3:
            if i + 4 <= n and (data[i + 3] & 0x1F) in (7, 8):
                return True
            i += 4
        else:
            i += 1
    return False


def _has_any_nal(data: bytes) -> bool:
    """Check for any NAL start code in *data* (quick garbage filter)."""
    return _ST4 in data or _ST3 in data


def _first_non_kf_nal(data: bytes) -> bool:
    """Return True if first NAL unit in *data* is a non-keyframe slice (type 1-4)."""
    n = len(data)
    i = 0
    while i < n - 3:
        if i + 4 <= n and data[i:i + 4] == _ST4:
            if i + 5 <= n:
                nt = data[i + 4] & 0x1F
                return nt in (1, 2, 3, 4)
            return False
        elif data[i:i + 3] == _ST3:
            if i + 4 <= n:
                nt = data[i + 3] & 0x1F
                return nt in (1, 2, 3, 4)
            return False
        else:
            i += 1
    return False


# ----------------------------------------------------------------------
# CRC8 (x^8 + x^5 + x^4 + 1, init=0xFF) 查表法，官方附录一数据
# ----------------------------------------------------------------------
CRC8_TAB = bytes([
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
])

def crc8(data: bytes, init: int = 0xFF) -> int:
    """计算 CRC8，多项式 x^8+x^5+x^4+1，初始值 0xFF"""
    crc = init
    for byte in data:
        crc = CRC8_TAB[crc ^ byte]
    return crc


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    """计算 CRC16-CCITT (poly=0x1021, init=0xFFFF), RoboMaster 官方协议常用。"""
    crc = init
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_serial_frame(cmd_id: int, data: bytes, seq: int = 0) -> bytes:
    """完全遵循官方协议封装串口帧

    格式:
        SOF:       1B  (0xA5)
        data_length:2B  (小端, data 的长度；不包含 cmd_id)
        seq:        1B  (包序号)
        CRC8:       1B  (对前4字节的校验)
        cmd_id:     2B  (小端)
        data:       N B
        CRC16:      2B  (小端, 对整帧 SOF~data 的校验)
    """
    frame = bytearray()
    frame.append(0xA5)                                 # SOF
    frame.extend(struct.pack("<H", len(data)))          # data_length (data only)
    frame.append(seq & 0xFF)                            # seq
    # 计算前4字节的 CRC8 (SOF + data_length + seq)
    crc_val = crc8(frame[:4])                           # 前4字节
    frame.append(crc_val)                               # CRC8
    frame.extend(struct.pack("<H", cmd_id))             # cmd_id (小端)
    frame.extend(data)                                  # data
    # CRC16 覆盖从 SOF 到 data 末尾 (不含 CRC16 本身)
    frame.extend(struct.pack("<H", crc16(frame)))
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
    ap.add_argument("--baud", type=int, default=921600, help="波特率")
    ap.add_argument("--robot-id", type=int, default=1)
    ap.add_argument("--ros-topic", default="/video_stream")
    ap.add_argument("--print-stats", action="store_true")
    ap.add_argument("--cmd-id", type=int, default=0x0310, help="下位机命令ID")
    ap.add_argument("--send-rate", type=int, default=40,
                    help="串口发送速率上限 (pkt/s)，不超过 MCU 处理能力")
    ap.add_argument(
        "--keyframe-interval", type=float, default=1.0,
        help="Minimum seconds between keyframe bursts (0 = every keyframe)")
    ap.add_argument("--no-keyframe-filter", action="store_true",
                    help="Disable keyframe filtering (send all packets)")
    ap.add_argument("--redundancy", type=int, default=2,
                    help="每个关键帧分片重复发送次数 (≥1, 模拟 QoS 1)")
    ap.add_argument("--chunk-delay-ms", type=int, default=5,
                    help="Burst 内相邻 chunk 串口写入间隔 (ms)")
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
    seq_counter = 0
    skip_count = 0          # 因限速跳过的包数
    pframe_skip = 0         # 被过滤的 chunk 总数
    kf_sent = 0             # 实际发出的关键帧 burst 数
    sent_frame_counter = 0  # 实际发送 chunk 的连续编号
    last_kf_time = 0.0      # 上次发出关键帧 burst 的时刻
    in_burst = False        # 当前是否在关键帧突发内
    grace_chunks = 0        # burst 结束后多送的 chunk 数（完成跨 chunk 的 NAL 分片）
    min_interval = 1.0 / max(args.send_rate, 1)
    last_send_time = 0.0

    print(f"[serial] Send rate limit: {args.send_rate} pkt/s (min interval {min_interval*1000:.1f} ms)")
    print(f"[serial] Keyframe filter: {'OFF' if args.no_keyframe_filter else f'ON (interval {args.keyframe_interval}s)'}")
    print(f"[serial] Redundancy: {args.redundancy}x")
    print(f"[serial] Chunk delay: {args.chunk_delay_ms}ms")

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
            nonlocal ros_rx, serial_tx, drop_count, skip_count, seq_counter
            nonlocal last_send_time, pframe_skip, kf_sent, last_kf_time
            nonlocal sent_frame_counter, in_burst, grace_chunks
            ros_rx += 1

            chunk = bytes(msg.data)  # exactly 280B Annex‑B H.264

            # ── 关键帧模式 ──
            if not args.no_keyframe_filter:
                now = time.monotonic()

                is_sps_pps = _has_sps_pps(chunk)
                if is_sps_pps:
                    if args.keyframe_interval > 0:
                        if now - last_kf_time < args.keyframe_interval:
                            # 冷却期内：只发 SPS/PPS chunk，不开新 burst
                            pass
                        else:
                            in_burst = True
                            last_kf_time = now
                            kf_sent += 1
                    else:
                        in_burst = True
                        kf_sent += 1

                # SPS/PPS 永远发送，不受 burst/grace 限制
                if not is_sps_pps:
                    if not in_burst and grace_chunks <= 0:
                        pframe_skip += 1
                        return

                    if not in_burst:
                        grace_chunks -= 1

                    if in_burst and _first_non_kf_nal(chunk):
                        in_burst = False
                        grace_chunks = 1

                # burst 内的 chunk，直接放行
            else:
                now = time.monotonic()
                # 全量模式：限速
                # 重要：不要通过“跳包”来限速（会随机丢 280B chunk，极易破坏码流导致花屏/绿屏）。
                # 改为节流发送：必要时 sleep，保证码流连续性；代价是可能增加端到端延迟。
                dt = now - last_send_time
                if dt < min_interval:
                    time.sleep(min_interval - dt)
                    now = time.monotonic()
                last_send_time = now

            # 包装 H.264 chunk (8B 片段头 + 280B payload) → 288B
            # 使用发送端连续编号代替原始 sequence_id，接收端可准确检测丢包
            frame_id = sent_frame_counter & 0xFFFF
            sent_frame_counter += 1
            frag = pack_fragment(
                frame_id=frame_id, frag_idx=0, frag_cnt=1,
                codec=CODEC_H264, flags=0, total_len=280, chunk=chunk,
            )
            # 补零到 300B（data_length=300），MCU 直转无需再补
            if len(frag) < 300:
                frag = frag + b'\x00' * (300 - len(frag))

            # 串口帧封装（传入当前 seq，每次递增，循环使用即可）
            frame = pack_serial_frame(args.cmd_id, frag, seq=seq_counter)
            seq_counter = (seq_counter + 1) & 0xFF  # seq 范围 0-255

            # ── 冗余发送 (模拟 QoS 1): 同一帧发多次 ──
            for _ in range(args.redundancy):
                if _ > 0:
                    time.sleep(0.001)  # 1ms 间隔，避免 MCU 串口 FIFO 溢出
                try:
                    ser.write(frame)
                    serial_tx += 1
                except Exception:
                    drop_count += 1

            # ── Burst 内 chunk 间隔: 串口传输本身约 3ms, 额外延迟让 MCU 完成 MQTT publish ──
            if not args.no_keyframe_filter:
                time.sleep(args.chunk_delay_ms / 1000.0)

    node = SerialBridgeNode()

    # 统计
    last_stat = time.monotonic()
    last_ros = 0
    last_tx = 0
    last_pframe = 0
    last_bursts = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            if args.print_stats and time.monotonic() - last_stat > 5.0:
                dt = time.monotonic() - last_stat
                dr = ros_rx - last_ros
                ds = serial_tx - last_tx
                dp = pframe_skip - last_pframe
                db = kf_sent - last_bursts
                print(
                    f"[serial] ROS rx={ros_rx} (+{dr}, {dr/dt:.0f} pkt/s) | "
                    f"Serial tx={serial_tx} (+{ds}, {ds/dt:.0f} pkt/s) | "
                    f"drops={drop_count} skipped={skip_count} | "
                    f"P-skip={pframe_skip} (+{dp}) kf-sent={kf_sent} (+{db})"
                )
                last_stat = time.monotonic()
                last_ros = ros_rx
                last_tx = serial_tx
                last_pframe = pframe_skip
                last_bursts = kf_sent
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())