#!/usr/bin/env python3
"""RoboMaster 图形化接收端

两个链路:
  MQTT CustomByteBlock (H.264) — 英雄模式可用 ✅
  UDP HEVC                 — 英雄模式被切 ❌

Modes:
  h264_stream    MQTT → H.264 解码显示 (配合 sender --mode h264_camera)
  hevc_udp       UDP :3334 → HEVC 解码显示
  stats_only     MQTT 仅统计

示例 (MQTT):
  python3 tools/pyqt_custombyteblock_viewer.py --mode h264_stream --robot-id 1
示例 (UDP):
  python3 tools/pyqt_custombyteblock_viewer.py --mode hevc_udp
"""

from __future__ import annotations

import argparse
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Optional, Tuple

# Optional: use OpenCV for higher-quality scaling (better than Qt's default in many cases)
try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

# ── 优先使用 protobuf 库 ──────────────────────────────────────────────
try:
    sys.path.insert(0, os.path.dirname(__file__))
    from custom_byteblock_pb import parse_cbb, using_protobuf_library  # type: ignore
except Exception:
    # 极简回退: 手动 varint 解析

    def _read_varint(buf: bytes, offset: int) -> Tuple[int, int]:
        result = 0
        shift = 0
        while True:
            if offset >= len(buf):
                raise ValueError("truncated varint")
            b = buf[offset]
            offset += 1
            result |= (b & 0x7F) << shift
            if (b & 0x80) == 0:
                return result, offset
            shift += 7
            if shift > 63:
                raise ValueError("varint too long")

    def parse_cbb(protobuf_payload: bytes) -> bytes:
        off = 0
        data_val: Optional[bytes] = None
        while off < len(protobuf_payload):
            key, off = _read_varint(protobuf_payload, off)
            field_no = key >> 3
            wire = key & 0x07
            if wire == 0:
                _, off = _read_varint(protobuf_payload, off)
            elif wire == 1:
                off += 8
            elif wire == 2:
                ln, off = _read_varint(protobuf_payload, off)
                if off + ln > len(protobuf_payload):
                    raise ValueError("truncated len-delimited")
                val = protobuf_payload[off : off + ln]
                off += ln
                if field_no == 1:
                    data_val = val
            elif wire == 5:
                off += 4
            else:
                raise ValueError(f"unsupported wire type: {wire}")
        return data_val or b""

    def using_protobuf_library() -> bool:
        return False


# ── UDP HEVC 帧头 ──
_UDP_HEVC_HEADER = struct.Struct("!HHI")  # frame_id:2B, frag_idx:2B, total_len:4B
UDP_HEVC_HEADER_LEN = _UDP_HEVC_HEADER.size  # 8 bytes


def unpack_udp_hevc_header(data: bytes) -> Tuple[int, int, int, bytes]:
    """解析 UDP HEVC 包: 返回 (frame_id, frag_idx, total_len, payload)."""
    if len(data) < UDP_HEVC_HEADER_LEN:
        raise ValueError(f"UDP HEVC packet too short: {len(data)} < {UDP_HEVC_HEADER_LEN}")
    frame_id, frag_idx, total_len = _UDP_HEVC_HEADER.unpack_from(data, 0)
    return frame_id, frag_idx, total_len, data[UDP_HEVC_HEADER_LEN:]


def _import_qt():
    def _maybe_reexec_with_python3() -> None:
        if os.environ.get("CBB_QT_NO_REEXEC") == "1" or os.environ.get("CBB_QT_REEXECED") == "1":
            return
        py3 = shutil.which("python3")
        if not py3:
            return
        if shutil.which(sys.executable) == py3:
            return

        for probe in [
            "import PyQt5,sys;sys.exit(0)",
            "import PySide6,sys;sys.exit(0)",
            "import PyQt6,sys;sys.exit(0)",
        ]:
            if subprocess.run([py3, "-c", probe], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                env = dict(os.environ)
                env["CBB_QT_REEXECED"] = "1"
                os.execvpe(py3, [py3, *sys.argv], env)

    try:
        from PyQt5 import QtCore, QtGui, QtWidgets  # type: ignore
        return QtWidgets, QtGui, QtCore, "PyQt5"
    except Exception:
        try:
            from PySide6 import QtCore, QtGui, QtWidgets  # type: ignore
            return QtWidgets, QtGui, QtCore, "PySide6"
        except Exception:
            try:
                from PyQt6 import QtCore, QtGui, QtWidgets  # type: ignore
                return QtWidgets, QtGui, QtCore, "PyQt6"
            except Exception as e:
                try:
                    _maybe_reexec_with_python3()
                except Exception:
                    pass
                raise SystemExit(
                    "Missing Qt Python bindings for THIS interpreter.\n\n"
                    f"Python: {sys.executable}\n"
                    f"Version: {sys.version.split()[0]}\n\n"
                    "Use python3 or install binding for this interpreter.\n"
                    "  python3 tools/pyqt_custombyteblock_viewer.py ...\n"
                    "  python -m pip install --user PyQt5\n"
                    "  python -m pip install --user PySide6\n\n"
                    f"Original import error: {e}\n"
                )


# ── H.264 NAL start-code scan (SPS=7, PPS=8) ───────────────────────────
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


def main() -> int:
    ap = argparse.ArgumentParser(
        description="RoboMaster 自定义数据流图形化接收端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", default="192.168.12.1", help="MQTT Broker IP (机器人端)")
    ap.add_argument("--port", type=int, default=3333, help="MQTT 端口")
    ap.add_argument("--topic", default="CustomByteBlock")
    ap.add_argument("--robot-id", type=int, default=1, help="机器人 ID 编号 (作为 MQTT clientID)")
    ap.add_argument("--udp-port", type=int, default=3334, help="UDP HEVC 视频监听端口 (for hevc_udp mode)")
    ap.add_argument("--window", default="RoboMaster CustomBlock Viewer")
    ap.add_argument("--mode", choices=["h264_stream", "hevc_udp", "stats_only"], default="h264_stream")
    ap.add_argument("--print-stats", action="store_true")
    args = ap.parse_args()

    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except Exception as e:
        raise SystemExit(f"Missing paho-mqtt: {e}")

    try:
        from custom_byteblock_codec import CODEC_H264, unpack_fragment  # type: ignore
    except Exception:
        sys.path.insert(0, os.path.dirname(__file__))
        from custom_byteblock_codec import CODEC_H264, unpack_fragment  # type: ignore

    # ── PyAV 解码器 ──
    _h264_codec = None
    _hevc_codec = None
    _hevc_reasm: dict = {}

    if args.mode in ("h264_stream", "hevc_udp"):
        try:
            import av  # type: ignore
            try:
                av.logging.set_level(av.logging.ERROR)  # suppress "no frame!" noise
            except Exception:
                pass
            if args.mode == "h264_stream":
                _h264_codec = av.CodecContext.create("h264", "r")
                _h264_codec.thread_type = "FRAME"
                _h264_codec.flags |= av.codec.context.Flags.LOW_DELAY
            elif args.mode == "hevc_udp":
                _hevc_codec = av.CodecContext.create("hevc", "r")
                _hevc_codec.thread_type = "FRAME"
                _hevc_codec.flags |= av.codec.context.Flags.LOW_DELAY
        except Exception as e:
            if args.mode == "hevc_udp":
                raise SystemExit(
                    "hevc_udp mode requires PyAV with HEVC support. Install with:\n"
                    "  sudo apt install -y python3-av\n"
                    f"Original error: {e}"
                )
            raise SystemExit(
                "h264_stream mode requires PyAV. Install with:\n"
                "  sudo apt install -y python3-av\n"
                f"Original error: {e}"
            )

    QtWidgets, QtGui, QtCore, binding = _import_qt()

    if hasattr(QtCore.Qt, "AlignmentFlag"):
        align_center = QtCore.Qt.AlignmentFlag.AlignCenter
        keep_aspect = QtCore.Qt.AspectRatioMode.KeepAspectRatio
        smooth_tx = QtCore.Qt.TransformationMode.SmoothTransformation
        text_selectable = QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
    else:
        align_center = QtCore.Qt.AlignCenter
        keep_aspect = QtCore.Qt.KeepAspectRatio
        smooth_tx = QtCore.Qt.SmoothTransformation
        text_selectable = QtCore.Qt.TextSelectableByMouse

    qsize_expanding = QtWidgets.QSizePolicy.Expanding if hasattr(QtWidgets.QSizePolicy, "Expanding") else QtWidgets.QSizePolicy.Policy.Expanding

    class _ImageView(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self._qimg = None
            self._text = "(waiting for frames...)"
            self._last_out_wh = (0, 0)
            self.setMinimumSize(1, 1)
            self.setSizePolicy(qsize_expanding, qsize_expanding)

        def set_text(self, text: str) -> None:
            self._text = str(text)
            self._qimg = None
            self._last_out_wh = (0, 0)
            self.update()

        def set_image(self, qimg) -> None:
            self._qimg = qimg
            self._text = ""
            self.update()

        def last_out_wh(self) -> Tuple[int, int]:
            return self._last_out_wh

        def paintEvent(self, event):  # type: ignore
            painter = QtGui.QPainter(self)
            try:
                painter.fillRect(self.rect(), self.palette().brush(QtGui.QPalette.Window))
            except Exception:
                pass

            if self._qimg is None or self._qimg.isNull():
                painter.setPen(self.palette().color(QtGui.QPalette.Text))
                painter.drawText(self.rect(), align_center, self._text)
                return

            src_w = int(self._qimg.width())
            src_h = int(self._qimg.height())
            view_w = max(1, int(self.width()))
            view_h = max(1, int(self.height()))

            scale = min(view_w / max(src_w, 1), view_h / max(src_h, 1))
            out_w = max(1, int(round(src_w * scale)))
            out_h = max(1, int(round(src_h * scale)))
            self._last_out_wh = (out_w, out_h)

            x = (view_w - out_w) // 2
            y = (view_h - out_h) // 2
            target = QtCore.QRect(int(x), int(y), int(out_w), int(out_h))

            try:
                painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform, True)
            except Exception:
                try:
                    painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
                except Exception:
                    pass
            painter.drawImage(target, self._qimg)

            # 绘制十字准心
            pen = QtGui.QPen(QtGui.QColor(0, 255, 0, 180), 0.5)
            painter.setPen(pen)
            cx, cy = x + out_w // 2, y + out_h // 2
            gap = 0
            length = 300
            painter.drawLine(cx, cy - gap, cx, cy - length)
            painter.drawLine(cx, cy + gap, cx, cy + length)
            painter.drawLine(cx - gap, cy, cx - length, cy)
            painter.drawLine(cx + gap, cy, cx + length, cy)

    client_id = "1"

    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QWidget()
    window.setWindowTitle(f"{args.window} [Robot {args.robot_id}] [{binding}]")

    btn_refresh = QtWidgets.QPushButton("刷新")
    image_view = _ImageView()

    status_label = QtWidgets.QLabel("")
    status_label.setTextInteractionFlags(text_selectable)

    top_row = QtWidgets.QHBoxLayout()
    top_row.addWidget(btn_refresh)
    top_row.addStretch(1)

    layout = QtWidgets.QVBoxLayout(window)
    layout.addLayout(top_row, 0)
    layout.addWidget(image_view, 1)
    layout.addWidget(status_label, 0)

    window.resize(960, 720)
    window.show()

    rx_msgs = bad_msgs = 0
    h264_frames = 0
    h264_parse_errors = 0
    h264_last_frame_time = 0.0
    h264_stall_resets = 0
    hevc_frames = 0
    hevc_packets = 0
    hevc_parse_errors = 0
    last_status_ts = 0.0
    last_img: Optional[object] = None
    last_frame_bgr = None
    last_data_len = 0
    last_data_head = b""
    last_src_wh: Tuple[int, int] = (0, 0)
    last_out_wh: Tuple[int, int] = (0, 0)
    # ── 丢包 / 延迟诊断 ──
    last_frame_id: Optional[int] = None
    gap_count = 0
    lost_pkts = 0

    payload_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=2000)

    def _qimage_from_bgr(arr) -> object:
        h, w, ch = arr.shape
        if ch != 3:
            raise ValueError(f"expected 3 channels, got {ch}")
        fmt_bgr = getattr(QtGui.QImage.Format, "Format_BGR888", None)
        if fmt_bgr is not None:
            qimg = QtGui.QImage(arr.data, w, h, w * ch, fmt_bgr)
            return qimg.copy()
        # Fallback for older Qt bindings: convert to RGB
        rgb = arr[:, :, ::-1].copy()
        fmt_rgb = getattr(QtGui.QImage.Format, "Format_RGB888", None)
        if fmt_rgb is None:
            raise ValueError("Qt binding missing Format_RGB888")
        qimg = QtGui.QImage(rgb.data, w, h, w * 3, fmt_rgb)
        return qimg.copy()

    def _fit_size(src_w: int, src_h: int, dst_w: int, dst_h: int) -> Tuple[int, int]:
        if src_w <= 0 or src_h <= 0 or dst_w <= 0 or dst_h <= 0:
            return 0, 0
        scale = min(dst_w / src_w, dst_h / src_h)
        out_w = max(1, int(round(src_w * scale)))
        out_h = max(1, int(round(src_h * scale)))
        return out_w, out_h

    def redraw() -> None:
        """Refresh the displayed image.

        Resizing is handled by the widget's paintEvent, so this only needs to
        push the latest frame into the view.
        """
        nonlocal last_img
        if last_img is None or last_img.isNull():
            return
        image_view.set_image(last_img)

    def update_status(force: bool = False) -> None:
        nonlocal last_status_ts
        now = time.monotonic()
        if not force and (now - last_status_ts) < 0.5:
            return
        last_status_ts = now
        if args.mode == "h264_stream":
            loss_pct = f"{(lost_pkts / max(rx_msgs, 1) * 100):.1f}%" if rx_msgs > 0 else "0%"
            try:
                sz = image_view.size()
                view_wh = (int(sz.width()), int(sz.height()))
            except Exception:
                view_wh = (-1, -1)
            try:
                last_out_wh = image_view.last_out_wh()
            except Exception:
                last_out_wh = (0, 0)
            status_label.setText(
                f"MQTT {args.host}:{args.port} topic={args.topic} | "
                f"rx={rx_msgs} bad={bad_msgs} lost={lost_pkts}({loss_pct}) gaps={gap_count} "
                f"frames={h264_frames} errs={h264_parse_errors} stall={h264_stall_resets} | "
                f"src={last_src_wh[0]}x{last_src_wh[1]} view={view_wh[0]}x{view_wh[1]} out={last_out_wh[0]}x{last_out_wh[1]} | "
                f"data={last_data_len}B"
            )
        elif args.mode == "hevc_udp":
            status_label.setText(
                f"UDP :{args.udp_port} mode=hevc_udp | "
                f"pkts={hevc_packets} bad={bad_msgs} frames={hevc_frames} parse_errs={hevc_parse_errors} | "
                f"proto={'pb' if using_protobuf_library() else 'manual'}"
            )
        else:  # stats_only
            status_label.setText(
                f"MQTT {args.host}:{args.port} topic={args.topic} mode={args.mode} | "
                f"rx={rx_msgs} bad={bad_msgs} | "
                f"data_len={last_data_len} head={last_data_head.hex()}"
            )

    def reset_view() -> None:
        nonlocal rx_msgs, bad_msgs, h264_frames, h264_parse_errors
        nonlocal h264_last_frame_time, h264_stall_resets
        nonlocal hevc_frames, hevc_packets, hevc_parse_errors
        nonlocal last_img, last_frame_bgr, last_data_len, last_data_head
        nonlocal last_frame_id, gap_count, lost_pkts
        rx_msgs = bad_msgs = 0
        h264_frames = 0
        h264_parse_errors = 0
        h264_last_frame_time = 0.0
        h264_stall_resets = 0
        hevc_frames = 0
        hevc_packets = 0
        hevc_parse_errors = 0
        last_img = None
        last_frame_bgr = None
        last_data_len = 0
        last_data_head = b""
        last_frame_id = None
        gap_count = lost_pkts = 0
        try:
            while True:
                payload_queue.get_nowait()
        except queue.Empty:
            pass
        _hevc_reasm.clear()
        if _h264_codec is not None:
            try:
                _h264_codec.flush_buffers()
            except Exception:
                pass
        if _hevc_codec is not None:
            try:
                _hevc_codec.flush_buffers()
            except Exception:
                pass
        image_view.set_text("(waiting for frames...)")
        update_status(force=True)

    btn_refresh.clicked.connect(reset_view)

    # Resizing is handled by the custom paintEvent in _ImageView.

    def on_connect(client, userdata, flags, rc):
        client.subscribe(args.topic)

    def on_message(client, userdata, msg):
        try:
            payload_queue.put_nowait(bytes(msg.payload))
        except queue.Full:
            pass

    # ── MQTT 客户端创建 ──
    client = mqtt.Client(client_id=client_id)
    client.on_connect = on_connect
    client.on_message = on_message

    def pump() -> None:
        nonlocal rx_msgs, bad_msgs, h264_frames, h264_parse_errors
        nonlocal h264_last_frame_time, h264_stall_resets
        nonlocal hevc_frames, hevc_packets, hevc_parse_errors
        nonlocal last_img, last_data_len, last_data_head
        nonlocal _h264_codec
        nonlocal last_frame_id, gap_count, lost_pkts
        drained = 0
        while drained < 200:
            try:
                pb = payload_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1

            try:
                # ── HEVC UDP mode: raw HEVC with 8-byte official header ──
                if args.mode == "hevc_udp":
                    hevc_packets += 1
                    try:
                        frame_id, frag_idx, total_len, chunk = unpack_udp_hevc_header(pb)
                    except Exception:
                        bad_msgs += 1
                        continue

                    last_data_len = len(pb)
                    last_data_head = pb[:8]

                    # Reassemble fragments
                    key = frame_id
                    entry = _hevc_reasm.get(key)
                    if entry is None:
                        entry = {"total": total_len, "chunks": {}, "ts": time.monotonic()}
                        _hevc_reasm[key] = entry
                    entry["chunks"][frag_idx] = chunk
                    entry["ts"] = time.monotonic()

                    # GC old entries
                    now = time.monotonic()
                    expired = [k for k, v in _hevc_reasm.items() if (now - v["ts"]) > 1.0]
                    for k in expired:
                        del _hevc_reasm[k]

                    # Check if frame is complete
                    if not entry["chunks"]:
                        continue
                    assembled = b"".join(entry["chunks"][i] for i in sorted(entry["chunks"]))
                    if len(assembled) >= entry["total"]:
                        assembled = assembled[: entry["total"]]
                        del _hevc_reasm[key]
                        # Decode HEVC
                        try:
                            parsed = _hevc_codec.parse(assembled)
                            for pkt in parsed:
                                try:
                                    frames = _hevc_codec.decode(pkt)
                                except av.AVError:
                                    hevc_parse_errors += 1
                                    continue
                                for frame in frames:
                                    if frame is None or frame.width == 0:
                                        continue
                                    arr = frame.to_ndarray(format="bgr24")
                                    if arr is None or arr.size == 0:
                                        continue
                                    hevc_frames += 1
                                    last_frame_bgr = arr.copy()
                                    last_src_wh = (int(frame.width), int(frame.height))
                                    last_img = _qimage_from_bgr(last_frame_bgr)
                                    redraw()
                        except av.AVError:
                            hevc_parse_errors += 1
                    continue

                # ── MQTT modes: 提取 protobuf 内部的 CustomByteBlock.data ──
                rx_msgs += 1

                # 先用 parse_cbb 剥掉 protobuf 外壳
                data = parse_cbb(pb)
                if not data:
                    continue
                last_data_len = len(data)
                last_data_head = data[:8]

                if args.mode == "stats_only":
                    continue

                # ── h264_stream ──
                if args.mode == "h264_stream":
                    try:
                        hdr, chunk = unpack_fragment(data)
                    except Exception:
                        bad_msgs += 1
                        lost_pkts += 1
                        continue
                    if hdr.codec != CODEC_H264:
                        bad_msgs += 1
                        lost_pkts += 1
                        print(f"DEBUG lost={lost_pkts} bad={bad_msgs} rx={rx_msgs}", flush=True)
                        continue

                    # ── 丢包检测 (bad 消息也视为丢失) ──
                    if last_frame_id is not None:
                        diff = (hdr.frame_id - last_frame_id) & 0xFFFF
                        if diff > 1:
                            gap_count += 1
                            lost_pkts += diff - 1
                    last_frame_id = hdr.frame_id

                    # ── 零填充去除 ──
                    if hdr.total_len > 0 and hdr.total_len <= len(chunk):
                        chunk = chunk[:hdr.total_len]
                    if not chunk:
                        continue

                    # ── SPS/PPS 出现 → 重建解码器 ──
                    if _h264_codec is None or _has_sps_pps(chunk):
                        try:
                            import av as _av
                            _h264_codec = _av.CodecContext.create("h264", "r")
                            _h264_codec.thread_type = "FRAME"
                            _h264_codec.flags |= _av.codec.context.Flags.LOW_DELAY
                        except Exception:
                            _h264_codec = None
                        if _h264_codec is None:
                            continue

                    # Feed raw chunk to PyAV — internal annex-B parser handles NAL assembly
                    try:
                        packets = _h264_codec.parse(chunk)
                    except Exception:
                        h264_parse_errors += 1
                        continue
                    try:
                        for pkt in packets:
                            try:
                                frames = _h264_codec.decode(pkt)
                            except av.AVError:
                                h264_parse_errors += 1
                                continue
                            for frame in frames:
                                if frame is None or frame.width == 0 or frame.height == 0:
                                    continue
                                arr = frame.to_ndarray(format="bgr24")
                                if arr is None or arr.size == 0:
                                    continue
                                h264_frames += 1
                                h264_last_frame_time = time.monotonic()
                                last_frame_bgr = arr.copy()
                                last_src_wh = (int(frame.width), int(frame.height))
                                last_img = _qimage_from_bgr(last_frame_bgr)
                                redraw()
                    except av.AVError:
                        h264_parse_errors += 1
                    continue
            except Exception:
                bad_msgs += 1

        # ── H.264 解码器 stall 检测与自动恢复 ──
        # 如果持续收到数据（rx_msgs 增长）但超过 3 秒没产出任何帧，
        # 说明解码器状态已被破坏，需要 flush + 重建。
        if (args.mode == "h264_stream" and _h264_codec is not None
                and h264_last_frame_time > 0 and rx_msgs > 10):
            now = time.monotonic()
            stall_sec = now - h264_last_frame_time
            if stall_sec > 3.0:
                print(f"[viewer] H.264 decoder stalled for {stall_sec:.1f}s (rx={rx_msgs} frames={h264_frames}), recreating...", flush=True)
                try:
                    import av
                    _h264_codec = av.CodecContext.create("h264", "r")
                    _h264_codec.thread_type = "FRAME"
                    _h264_codec.flags |= av.codec.context.Flags.LOW_DELAY
                except Exception:
                    pass
                h264_stall_resets += 1
                h264_last_frame_time = time.monotonic()  # prevent rapid re-trigger

        if args.print_stats:
            update_status()

    # ── UDP HEVC 监听线程 ──
    _udp_sock: Optional[socket.socket] = None
    _udp_running = False

    if args.mode == "hevc_udp":
        _udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        _udp_sock.bind(("0.0.0.0", args.udp_port))
        _udp_sock.settimeout(0.5)
        _udp_running = True

        def udp_listener():
            while _udp_running:
                try:
                    data, addr = _udp_sock.recvfrom(65535)
                    try:
                        payload_queue.put_nowait(data)
                    except queue.Full:
                        pass
                except socket.timeout:
                    continue
                except Exception:
                    break

        udp_thread = threading.Thread(target=udp_listener, daemon=True)
        udp_thread.start()
        print(f"[viewer] UDP HEVC listener started on :{args.udp_port}")
        status_label.setText(f"UDP :{args.udp_port} — 等待 HEVC 视频流...")

    # ── MQTT 连接 (非 hevc_udp 模式) ──
    if args.mode != "hevc_udp":
        try:
            client.connect(args.host, args.port, keepalive=10)
        except Exception as e:
            status_label.setText(f"MQTT connect failed: {e}")
            return app.exec()
        client.loop_start()
        print(f"[viewer] MQTT connected: {args.host}:{args.port}/{args.topic} clientID={client_id}")

    timer = QtCore.QTimer()
    timer.setInterval(15)
    timer.timeout.connect(pump)
    timer.start()

    update_status(force=True)
    try:
        rc = app.exec()
    finally:
        if args.mode != "hevc_udp":
            client.loop_stop()
            try:
                client.disconnect()
            except Exception:
                pass
        if _udp_sock is not None:
            _udp_running = False
            try:
                _udp_sock.close()
            except Exception:
                pass
    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())