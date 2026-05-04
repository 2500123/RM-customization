import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket
import av
import cv2
import threading
import queue
from pathlib import Path


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


def _nal_types_present(data: bytes) -> set[int]:
    """Return set of NAL unit types observed in Annex-B *data* (best-effort)."""
    types: set[int] = set()
    n = len(data)
    i = 0
    while i < n - 3:
        if i + 4 <= n and data[i:i + 4] == _ST4:
            if i + 5 <= n:
                types.add(data[i + 4] & 0x1F)
                if len(types) >= 4:
                    return types
            i += 4
        elif data[i:i + 3] == _ST3:
            if i + 4 <= n:
                types.add(data[i + 3] & 0x1F)
                if len(types) >= 4:
                    return types
            i += 3
        else:
            i += 1
    return types


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')

        # 参数
        self.declare_parameter('topic', '/video_stream')
        self.declare_parameter('display', True)
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_save_decoder', True)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')
        self.declare_parameter('reset_gap_packets', 200)
        self.declare_parameter('resync_buffer_max_bytes', 8192)

        topic = self.get_parameter('topic').value
        self.display = self.get_parameter('display').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.display_scale = max(1, int(self.get_parameter('display_scale').value))
        self.display_width = self.width * self.display_scale
        self.display_height = self.height * self.display_scale
        self.crosshair_offset_x = int(self.get_parameter('crosshair_offset_x').value)
        self.crosshair_offset_y = int(self.get_parameter('crosshair_offset_y').value)
        self.crosshair_width = max(1, int(self.get_parameter('crosshair_width').value))
        self.debug_dump_enable = bool(self.get_parameter('debug_dump_enable').value)
        self.debug_dump_every_n_frames = max(1, int(self.get_parameter('debug_dump_every_n_frames').value))
        self.debug_dump_save_decoder = bool(self.get_parameter('debug_dump_save_decoder').value)
        self.debug_dump_dir = Path(str(self.get_parameter('debug_dump_dir').value)) / 'decoder'
        self.reset_gap_packets = max(1, int(self.get_parameter('reset_gap_packets').value))
        self.resync_buffer_max_bytes = max(1024, int(self.get_parameter('resync_buffer_max_bytes').value))
        self.display_frame_counter = 0
        if self.debug_dump_enable and self.debug_dump_save_decoder:
            self.debug_dump_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Debug dump enabled: every {self.debug_dump_every_n_frames} frames -> {self.debug_dump_dir}'
            )
        elif self.debug_dump_enable:
            self.get_logger().warn('debug_dump_enable=true but debug_dump_save_decoder=false')

        # 流式解码器状态
        self.codec = None
        self._create_codec()
        self.frame_count = 0
        self.packet_count = 0
        self.parsed_packet_count = 0
        self.gap_count = 0
        self.last_seq = None
        self._need_headers = True
        self._pending = bytearray()

        # 显示队列
        if self.display:
            self.frame_queue = queue.Queue(maxsize=3)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3000
        )
        self.subscription = self.create_subscription(VideoPacket, topic, self._packet_callback, qos)
        self.get_logger().info(f'Decoder started: subscribing to {topic}')

    def _create_codec(self):
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'FRAME'
        self.codec.flags |= av.codec.context.Flags.LOW_DELAY
        try:
            av.logging.set_level(av.logging.ERROR)  # suppress PyAV "no frame!" noise
        except Exception:
            pass

    def _reset_codec(self, reason=''):
        self._create_codec()
        self._need_headers = True
        self._pending.clear()
        self.get_logger().warn(f'Reset codec ({reason})')

    def _handle_decoded_frame(self, frame):
        if frame is None or frame.width == 0 or frame.height == 0:
            return
        img = frame.to_ndarray(format='bgr24')
        if img is None or img.size == 0:
            return
        self.frame_count += 1
        if self.display:
            try:
                self.frame_queue.put_nowait(img)
            except queue.Full:
                self.get_logger().debug('Display queue full, dropping frame', throttle_duration_sec=5)
        elif self.frame_count % 60 == 0:
            self.get_logger().info(f'Decoded {self.frame_count} frames')

    def _packet_callback(self, msg):
        """Feed raw 280B chunks to PyAV — internal annex-B parser handles NAL assembly."""
        self.packet_count += 1

        # 丢包检测
        if self.last_seq is not None and msg.sequence_id <= self.last_seq:
            # publisher restart / out-of-order: reset so we don't decode across discontinuity
            self.gap_count += 1
            self.get_logger().warn(f'Sequence reset/out-of-order: {self.last_seq} -> {msg.sequence_id}, reset')
            self._reset_codec('sequence reset')
        elif self.last_seq is not None and msg.sequence_id > self.last_seq + self.reset_gap_packets:
            self.gap_count += 1
            self.get_logger().warn(f'Large gap: {self.last_seq} -> {msg.sequence_id}, reset')
            self._reset_codec('large sequence gap')
        elif self.last_seq is not None and msg.sequence_id != self.last_seq + 1:
            self.gap_count += 1
        self.last_seq = msg.sequence_id

        chunk = bytes(msg.data)  # 280B Annex-B

        # After reset/gap, buffer bytes until we have SPS+PPS+IDR, then feed.
        # This avoids "non-existing PPS" / green-screen storms when starting mid-stream.
        if self._need_headers:
            self._pending.extend(chunk)
            if len(self._pending) > self.resync_buffer_max_bytes:
                # Drop oldest bytes, try align to a start code.
                drop = len(self._pending) - self.resync_buffer_max_bytes
                buf = bytes(self._pending)
                cut = drop
                # search forward for next start code (best-effort)
                for j in range(drop, min(len(buf) - 4, drop + 512)):
                    if buf[j:j + 3] == _ST3 or buf[j:j + 4] == _ST4:
                        cut = j
                        break
                del self._pending[:cut]

            types = _nal_types_present(self._pending)
            have_headers = (7 in types) and (8 in types)
            have_idr = (5 in types)
            if not (have_headers and have_idr):
                return

            # Feed buffered data once we see a clean restart point.
            payload = bytes(self._pending)
            self._pending.clear()
            self._need_headers = False
            chunk = payload

        try:
            packets = self.codec.parse(chunk)
            self.parsed_packet_count += len(packets)
            for pkt in packets:
                try:
                    frames = self.codec.decode(pkt)
                except av.AVError:
                    continue
                for frame in frames:
                    self._handle_decoded_frame(frame)
        except av.AVError as e:
            self.get_logger().debug(f'Parse error: {e!s}')

        if self.packet_count % 600 == 0:
            self.get_logger().info(
                f'Rx packets={self.packet_count} parsed_h264={self.parsed_packet_count} '
                f'decoded_frames={self.frame_count} gaps={self.gap_count}')

    def _display_loop(self):
        """独立线程显示"""
        cv2.namedWindow('Doorlock Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Doorlock Decoder', self.display_width, self.display_height)
        while rclpy.ok():
            try:
                img = self.frame_queue.get(timeout=0.05)
                if img is None:
                    break
                if img.size > 0:
                    img_disp = cv2.resize(img, (self.display_width, self.display_height), interpolation=cv2.INTER_NEAREST)
                    self._draw_overlay(img_disp)
                    cv2.imshow('Doorlock Decoder', img_disp)
                    if self.debug_dump_enable and self.debug_dump_save_decoder:
                        self.display_frame_counter += 1
                        if self.display_frame_counter % self.debug_dump_every_n_frames == 0:
                            frame_id = f'{self.display_frame_counter:08d}'
                            out_path = self.debug_dump_dir / f'decoder_{frame_id}.png'
                            cv2.imwrite(str(out_path), img_disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info('User quit')
                        rclpy.shutdown()
                        break
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Display error: {e}')
                break
        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        """叠加准心与中心圆点。"""
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, w // 2 + self.crosshair_offset_x))
        cy = max(0, min(h - 1, h // 2 + self.crosshair_offset_y))
        crosshair_color = (230, 190, 235)
        cv2.line(img, (0, cy), (w - 1, cy), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        center_color = (170, 255, 170)
        center = (w // 2, h // 2)
        cv2.circle(img, center, 24, center_color, 1, cv2.LINE_AA)

    def destroy_node(self):
        if self.display:
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass
            if hasattr(self, 'display_thread'):
                self.display_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
