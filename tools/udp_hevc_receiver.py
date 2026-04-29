#!/usr/bin/env python3
"""RoboMaster UDP HEVC 视频流接收器 — 命令行版 (无 GUI)

监听 UDP 3334 端口, 接收 HEVC 码流。

两种模式:
  (默认)  RM 官方图传 — 8B 帧头 (frame_id:2B + frag_idx:2B + total_len:4B) + HEVC data
  --raw  剥 8B 帧头逐片喂 — 本地测试用 (发端是 RM 格式但不拼帧)

  链路规格:
    编码格式: HEVC (H.265)
    端口:     UDP 3334
    来源:     机器人 (192.168.12.1) → 自定义客户端 (192.168.12.2)

使用:
  # 接收官方图传 (默认)
  python3 tools/udp_hevc_receiver.py --display

  # 本地 ffmpeg 测试 (裸流)
  python3 tools/udp_hevc_receiver.py --raw --display

  # 仅统计
  python3 tools/udp_hevc_receiver.py --stats-only
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time
from collections import deque
from typing import Dict, Optional, Tuple

# ── UDP HEVC 帧头 ────────────────────────────────────────────────────
_HEADER = struct.Struct("!HHI")  # frame_id:2B, frag_idx:2B, total_len:4B
HEADER_LEN = _HEADER.size


def unpack_hevc(data: bytes) -> Tuple[int, int, int, bytes]:
    if len(data) < HEADER_LEN:
        raise ValueError(f"packet too short: {len(data)} < {HEADER_LEN}")
    frame_id, frag_idx, total_len = _HEADER.unpack_from(data, 0)
    return frame_id, frag_idx, total_len, data[HEADER_LEN:]


# ── HEVC 帧重组 ──────────────────────────────────────────────────────

class HevcReassembler:
    def __init__(self, timeout_s: float = 1.0):
        self._timeout = timeout_s
        self._frames: Dict[int, dict] = {}

    def push(self, frame_id: int, frag_idx: int, total_len: int, chunk: bytes) -> Optional[bytes]:
        now = time.monotonic()
        key = frame_id

        # GC
        expired = [k for k, v in self._frames.items() if (now - v["ts"]) > self._timeout]
        for k in expired:
            del self._frames[k]

        entry = self._frames.get(key)
        if entry is None:
            entry = {"total": total_len, "chunks": {}, "ts": now}
            self._frames[key] = entry

        if frag_idx not in entry["chunks"]:
            entry["chunks"][frag_idx] = chunk
        entry["ts"] = now
        entry["total"] = total_len  # update in case total_len changes

        # Try assembly: if we have enough bytes
        assembled = b"".join(entry["chunks"][i] for i in sorted(entry["chunks"]))
        if len(assembled) >= entry["total"]:
            del self._frames[key]
            return assembled[: entry["total"]]
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="RoboMaster UDP HEVC 视频流接收器")
    ap.add_argument("--port", type=int, default=3334, help="UDP 监听端口")
    ap.add_argument("--bind", default="", help="绑定地址 (空=所有网卡, 比赛用 192.168.12.2)")
    ap.add_argument("--display", action="store_true", help="OpenCV 显示解码画面")
    ap.add_argument("--display-scale", type=int, default=1, help="显示缩放倍数")
    ap.add_argument("--stats-only", action="store_true", help="仅统计, 不解码")
    ap.add_argument("--raw", action="store_true", help="裸 HEVC 流 (跳过 8B 帧头, 本地测试用)")
    ap.add_argument("--save-dir", default="", help="保存解码帧到此目录 (PNG)")
    ap.add_argument("--print-stats-interval", type=float, default=2.0, help="统计打印间隔(秒)")
    args = ap.parse_args()

    # ── HEVC 解码器 ──
    codec = None
    display = args.display and not args.stats_only

    if not args.stats_only:
        try:
            import av  # type: ignore
            codec = av.CodecContext.create("hevc", "r")
            codec.thread_type = "FRAME"
            codec.flags |= av.codec.context.Flags.LOW_DELAY
            print("[hevc] PyAV HEVC decoder initialized")
        except Exception as e:
            print(f"[hevc] PyAV HEVC not available: {e}")
            print("[hevc] Falling back to --stats-only mode")
            args.stats_only = True

    if display:
        try:
            import cv2  # type: ignore
        except Exception:
            print("[hevc] OpenCV not available, disabling display")
            display = False

    # ── 保存目录 ──
    save_dir = args.save_dir.strip() or None
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # ── UDP Socket ──
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)
    print(f"[hevc] Listening on UDP {args.bind}:{args.port}")

    # ── Stats ──
    pkts_rx = 0
    pkts_bad = 0
    frames_ok = 0
    frames_bad = 0
    last_stat = time.monotonic()
    last_pkts = 0
    last_frames = 0

    reasm = HevcReassembler(timeout_s=1.0)

    raw_mode = args.raw
    if raw_mode:
        print("[hevc] --raw mode: stripping 8B RM header, feeding chunks to decoder")

    if display:
        cv2.namedWindow("RoboMaster HEVC", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RoboMaster HEVC", 640, 480)

    frame_save_idx = 0

    print("[hevc] Running... (Ctrl+C to stop)")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue

            pkts_rx += 1

            if raw_mode:
                # 剥掉 8B RM 帧头, 逐片喂解码器 (PyAV 内部缓冲拼帧)
                if len(data) > 8:
                    assembled = data[8:]
                else:
                    continue
            else:
                try:
                    frame_id, frag_idx, total_len, chunk = unpack_hevc(data)
                except Exception:
                    pkts_bad += 1
                    continue

                assembled = reasm.push(frame_id, frag_idx, total_len, chunk)
                if assembled is None:
                    continue

            # ── Decode ──
            if args.stats_only or codec is None:
                frames_ok += 1
                continue

            try:
                parsed = codec.parse(assembled)
                for pkt in parsed:
                    try:
                        frames = codec.decode(pkt)
                    except av.AVError:
                        frames_bad += 1
                        continue
                    for frame in frames:
                        if frame is None or frame.width == 0:
                            continue
                        arr = frame.to_ndarray(format="bgr24")
                        if arr is None or arr.size == 0:
                            continue
                        frames_ok += 1

                        if display:
                            h, w = arr.shape[:2]
                            scale = max(1, int(args.display_scale))
                            if scale > 1:
                                arr = cv2.resize(arr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
                            cv2.imshow("RoboMaster HEVC", arr)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                print("[hevc] User quit")
                                return 0

                        if save_dir:
                            frame_save_idx += 1
                            out_path = os.path.join(save_dir, f"hevc_{frame_save_idx:08d}.png")
                            cv2.imwrite(out_path, arr)
            except av.AVError:
                frames_bad += 1

            # ── Print stats ──
            now = time.monotonic()
            if now - last_stat >= args.print_stats_interval:
                dt = now - last_stat
                dp = pkts_rx - last_pkts
                df = frames_ok - last_frames
                print(
                    f"[hevc] pkts={pkts_rx} (+{dp}, {dp/dt:.0f} pkt/s) | "
                    f"bad_pkts={pkts_bad} | frames={frames_ok} (+{df}, {df/dt:.1f} fps) | "
                    f"bad_frames={frames_bad}"
                )
                last_stat = now
                last_pkts = pkts_rx
                last_frames = frames_ok

    except KeyboardInterrupt:
        print(f"\n[hevc] Stopped. pkts={pkts_rx} bad={pkts_bad} frames={frames_ok} bad_frames={frames_bad}")
    finally:
        if display:
            cv2.destroyAllWindows()
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
