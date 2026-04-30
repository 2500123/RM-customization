#!/usr/bin/env python3
"""RoboMaster ROS 2 → MQTT 桥接节点 (运行在小电脑/机器人端)

订阅 /video_stream (VideoPacket H.264 分片), 封装为 CustomByteBlock protobuf,
通过 MQTT 发布到机器人端 Broker (192.168.12.1:3333)。

  链路:
    小电脑 ROS 2 (hik_camera + video_encoder)
      → /video_stream (VideoPacket, 150 bytes H.264 chunks)
      → 本桥接节点
      → MQTT CustomByteBlock (300 bytes max, 含 8B 片段头)
      → 自定义客户端 PC (192.168.12.2, pyqt viewer)

  英雄部署模式:
    官方图传被切断, CustomByteBlock 数据流不受影响。
    本桥接节点确保视频编码数据通过自定义数据链路到达自定义客户端。

使用:
  # 先 source ROS 2 环境
  source install/setup.bash
  python3 tools/ros2_mqtt_bridge.py --robot-id 1 --print-stats
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from typing import Optional

# ── 优先使用 protobuf 库 ──────────────────────────────────────────────
try:
    sys.path.insert(0, os.path.dirname(__file__))
    from custom_byteblock_pb import serialize_cbb, using_protobuf_library  # type: ignore
except Exception:
    def _encode_varint(value: int) -> bytes:
        out = bytearray()
        while True:
            b = value & 0x7F
            value >>= 7
            if value:
                out.append(b | 0x80)
            else:
                out.append(b)
                break
        return bytes(out)

    def serialize_cbb(data: bytes) -> bytes:
        return b"\x0A" + _encode_varint(len(data)) + data

    def using_protobuf_library() -> bool:
        return False


# ── fragment header (same as custom_byteblock_codec.py) ───────────────

_HEADER = struct.Struct("!HBBBBH")
HEADER_LEN = _HEADER.size
CODEC_H264 = 2

# RoboMaster 官方限制
MAX_CUSTOM_BYTEBLOCK_BYTES = 300


def pack_h264_fragment(frame_id: int, chunk: bytes) -> bytes:
    """Pack a single H.264 chunk into a fragment (frag_cnt=1, total_len=0)."""
    return _HEADER.pack(frame_id, 0, 1, CODEC_H264, 0, 0) + chunk


# ── bridge node ───────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="RoboMaster ROS 2 /video_stream → MQTT CustomByteBlock 桥接节点",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", default="192.168.12.1", help="MQTT Broker IP (机器人端)")
    ap.add_argument("--port", type=int, default=3333, help="MQTT 端口")
    ap.add_argument("--topic", default="CustomByteBlock")
    ap.add_argument("--robot-id", type=int, default=1, help="机器人 ID 编号 (作为 MQTT clientID)")
    ap.add_argument("--ros-topic", default="/video_stream")
    ap.add_argument("--print-stats", action="store_true")
    ap.add_argument("--stats-interval", type=float, default=5.0)
    args = ap.parse_args()

    # ── MQTT ──
    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except Exception as e:
        raise SystemExit(
            "Missing paho-mqtt. Install with:\n"
            "  sudo apt install -y python3-paho-mqtt\n"
            f"Original error: {e}"
        )

    # ── ROS 2 ──
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from doorlock_sniper.msg import VideoPacket  # type: ignore
    except Exception as e:
        raise SystemExit(
            "Missing ROS 2 / doorlock_sniper dependencies.\n"
            "Make sure to source install/setup.bash first.\n"
            f"Original error: {e}"
        )

    rclpy.init(args=sys.argv[1:])

    # ── shared state ──
    ros_rx = 0
    mqtt_tx = 0
    drop_count = 0
    oversized_count = 0

    client_id = f"rm-bridge-{args.robot_id}"

    print(f"[bridge] Robot ID={args.robot_id}")
    print(f"[bridge] ROS {args.ros_topic} → MQTT {args.host}:{args.port}/{args.topic}")
    print(f"[bridge] Proto: {'protobuf library' if using_protobuf_library() else 'manual varint'}")
    print(f"[bridge] VideoPacket payload: 280 bytes → fragment: 288 bytes → CustomByteBlock wire: ~291 bytes (< {MAX_CUSTOM_BYTEBLOCK_BYTES}B)")

    # MQTT client
    client = mqtt.Client(client_id=client_id)
    client.connect(args.host, args.port, keepalive=10)
    client.loop_start()

    # ROS node
    class BridgeNode(Node):
        def __init__(self):
            super().__init__("ros2_mqtt_bridge")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=3000,
            )
            self.sub = self.create_subscription(
                VideoPacket, args.ros_topic, self._on_packet, qos
            )
            self.get_logger().info(
                f"Bridge started: {args.ros_topic} → MQTT {args.host}:{args.port}/{args.topic}"
            )

        def _on_packet(self, msg: VideoPacket) -> None:
            nonlocal ros_rx, mqtt_tx, drop_count, oversized_count
            ros_rx += 1

            chunk = bytes(msg.data)  # 150 bytes of H.264
            # Wrap in fragment header (frame_id = low 16 bits of sequence_id)
            frame_id = int(msg.sequence_id) & 0xFFFF
            frag = pack_h264_fragment(frame_id, chunk)

            # 检查是否超过官方限制 (158 bytes 远小于 300 bytes limit)
            if len(frag) > MAX_CUSTOM_BYTEBLOCK_BYTES:
                oversized_count += 1
                return

            pb = serialize_cbb(frag)

            result = client.publish(args.topic, payload=pb, qos=0, retain=False)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                mqtt_tx += 1
            else:
                drop_count += 1

    node = BridgeNode()

    # Stats timer
    last_stat = time.monotonic()
    last_ros_rx = 0
    last_mqtt_tx = 0
    last_drop = 0

    def print_stats():
        nonlocal last_stat, last_ros_rx, last_mqtt_tx, last_drop
        now = time.monotonic()
        dt = now - last_stat
        if dt <= 0:
            return
        dr = ros_rx - last_ros_rx
        dm = mqtt_tx - last_mqtt_tx
        dd = drop_count - last_drop
        rate = dr / dt
        bw = (dm * 291) / dt  # ~291 bytes per MQTT message
        print(
            f"[bridge] ROS rx={ros_rx} (+{dr}, {rate:.0f} pkt/s) | "
            f"MQTT tx={mqtt_tx} (+{dm}, {bw:.0f} B/s) | "
            f"drops={drop_count} (+{dd}) oversize={oversized_count}"
        )
        last_stat = now
        last_ros_rx = ros_rx
        last_mqtt_tx = mqtt_tx
        last_drop = drop_count

    try:
        if args.print_stats:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=args.stats_interval)
                print_stats()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        try:
            client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
