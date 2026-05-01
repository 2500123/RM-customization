#!/usr/bin/env python3
"""RoboMaster MQTT CustomByteBlock 发送端

通过官方自定义数据链路发送 H.264 编码的视频:

Modes:
  h264_camera   摄像头实时采集 → H.264 编码 → MQTT (一条命令)
  h264_file     H.264 文件循环回放 → MQTT (无摄像头测试)

示例:
  python3 tools/mqtt_custombyteblock_sender.py --mode h264_camera --camera 0 \
      --width 300 --height 300 --robot-id 1 --host 127.0.0.1 --print-stats
"""

from __future__ import annotations

import argparse, os, sys, time
from collections import deque
from typing import Deque, Optional

# ── protobuf ──
try:
    sys.path.insert(0, os.path.dirname(__file__))
    from custom_byteblock_pb import serialize_cbb, using_protobuf_library
except Exception:
    def _ev(v):
        out=bytearray()
        while True:
            b=v&0x7F; v>>=7
            if v: out.append(b|0x80)
            else: out.append(b); break
        return bytes(out)
    def serialize_cbb(d): return b"\x0A"+_ev(len(d))+d
    def using_protobuf_library(): return False



class H264LiveEncoder:
    """OpenCV BGR → YUV420p → PyAV x264 编码器 (Annex-B 输出)。"""
    def __init__(self, width=300, height=300, fps=15, bitrate=80_000, preset="ultrafast", gop=30):
        import cv2; import av
        self._w, self._h = int(width), int(height)
        self._w = self._w if self._w % 2 == 0 else self._w + 1
        self._h = self._h if self._h % 2 == 0 else self._h + 1
        self._c = av.CodecContext.create("h264","w")
        self._c.width=self._w; self._c.height=self._h
        self._c.pix_fmt="yuv420p"; self._c.framerate=int(fps)
        self._c.bit_rate=int(bitrate); self._c.gop_size=int(gop)
        self._c.options={"preset":str(preset),"tune":"zerolatency"}
        self._c.open()
    def encode(self, bgr):
        import cv2; import av
        bgr = cv2.resize(bgr, (self._w, self._h)) if bgr.shape[:2] != (self._h, self._w) else bgr
        yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
        h, w = self._h, self._w
        w2 = w // 2
        frame = av.VideoFrame(w, h, "yuv420p")
        for plane_idx, (arr, stride) in enumerate([
            (yuv[0:h, 0:w], w),
            (yuv[h:, 0:w2], w2),
            (yuv[h:, w2:w2*2], w2),
        ]):
            p = frame.planes[plane_idx]
            # 逐行复制，补齐行对齐
            buf = bytearray(p.buffer_size)
            for row in range(arr.shape[0]):
                off = row * p.line_size
                buf[off:off+stride] = arr[row].tobytes()
            p.update(bytes(buf))
        pkts = self._c.encode(frame)
        return [bytes(p) for p in pkts if p and p.size>0]
    def flush(self):
        return [bytes(p) for p in self._c.encode(None) if p and p.size>0]


def _open_hikvision_camera(cam_index=0):
    try:
        import ctypes,numpy as np,cv2
        p="/opt/MVS/Samples/64/Python/MvImport"
        if p not in sys.path: sys.path.insert(0,p)
        if "MVCAM_COMMON_RUNENV" not in os.environ: os.environ["MVCAM_COMMON_RUNENV"]="/opt/MVS/lib"
        from CameraParams_header import MV_FRAME_OUT,MV_CC_DEVICE_INFO_LIST
        from MvCameraControl_class import MvCamera,MV_USB_DEVICE,MV_OK
        dl=MV_CC_DEVICE_INFO_LIST()
        if MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE,dl)!=MV_OK or dl.nDeviceNum==0: return None
        if cam_index>=dl.nDeviceNum: return None
        cam=MvCamera()
        if cam.MV_CC_CreateHandle(dl.pDeviceInfo[cam_index])!=MV_OK: return None
        if cam.MV_CC_OpenDevice()!=MV_OK: cam.MV_CC_DestroyHandle(); return None
        cam.MV_CC_StartGrabbing(); of=MV_FRAME_OUT()
        if cam.MV_CC_GetImageBuffer(of,2000)!=MV_OK: cam.MV_CC_StopGrabbing(); cam.MV_CC_CloseDevice(); cam.MV_CC_DestroyHandle(); return None
        W,H=of.stFrameInfo.nWidth,of.stFrameInfo.nHeight; cam.MV_CC_FreeImageBuffer(of)
        class M:
            def __init__(s): s._cam,s._w,s._h=cam,W,H
            def read(s):
                of=MV_FRAME_OUT()
                if s._cam.MV_CC_GetImageBuffer(of,500)!=MV_OK: return False,None
                ba=np.ctypeslib.as_array(ctypes.cast(of.pBufAddr,ctypes.POINTER(ctypes.c_ubyte)),shape=(s._h,s._w)).copy()
                s._cam.MV_CC_FreeImageBuffer(of)
                return True,cv2.cvtColor(ba,cv2.COLOR_BayerRG2BGR)
            def release(s): s._cam.MV_CC_StopGrabbing(); s._cam.MV_CC_CloseDevice(); s._cam.MV_CC_DestroyHandle()
            def isOpened(s): return True
        return M()
    except: return None


def main()->int:
    ap=argparse.ArgumentParser(description="RoboMaster MQTT CustomByteBlock 发送端")
    ap.add_argument("--host",default="192.168.12.1"); ap.add_argument("--port",type=int,default=3333)
    ap.add_argument("--topic",default="CustomByteBlock"); ap.add_argument("--robot-id",type=int,default=1)
    ap.add_argument("--mode",choices=["h264_camera","h264_file"],default="h264_camera")
    ap.add_argument("--send-hz",type=float,default=50.0); ap.add_argument("--max-data-bytes",type=int,default=300)
    ap.add_argument("--camera",type=int,default=0); ap.add_argument("--width",type=int,default=300)
    ap.add_argument("--height",type=int,default=300); ap.add_argument("--h264-bitrate",type=int,default=80000)
    ap.add_argument("--h264-preset",default="ultrafast"); ap.add_argument("--cam-fps",type=int,default=15)
    ap.add_argument("--h264-gop",type=int,default=30); ap.add_argument("--h264-chunk-len",type=int,default=150)
    ap.add_argument("--h264-file",default=""); ap.add_argument("--print-stats",action="store_true")
    args=ap.parse_args()

    try: import paho.mqtt.client as mqtt
    except Exception as e: raise SystemExit(f"Missing paho-mqtt: {e}")
    sys.path.insert(0,os.path.dirname(__file__))
    from custom_byteblock_codec import CODEC_H264,pack_fragment

    h264_data=b""; h264_offset=0
    if args.mode=="h264_file":
        p=args.h264_file.strip() or None
        if not p: raise SystemExit("--mode h264_file requires --h264-file")
        with open(p,"rb") as f: h264_data=f.read()
        if not h264_data: raise SystemExit(f"Empty: {p}")
        print(f"[file] {len(h264_data)} bytes from {p}")

    h264_cap=None; h264_enc=None
    h264_chunk_buf:Deque[bytes]=deque(); h264_frame_count=0; h264_enc_bytes=0
    if args.mode=="h264_camera":
        ci=int(args.camera)

        # ── 等待海康 MVS 相机就绪（无限循环） ──
        while h264_cap is None:
            h264_cap = _open_hikvision_camera(ci)
            if h264_cap is not None:
                ok, probe = h264_cap.read()
                if not ok or probe is None:
                    h264_cap.release(); h264_cap = None
            if h264_cap is None:
                print("[camera] Waiting for Hikvision camera...", flush=True)
                time.sleep(2)

        print("[camera] Hikvision MVS SDK connected")
        H,W=probe.shape[:2]
        ew=int(args.width) if args.width>0 else (W if W>0 else 300)
        eh=int(args.height) if args.height>0 else (H if H>0 else 300)
        print(f"[camera] {W}x{H} native, encode {ew}x{eh}")
        h264_enc = H264LiveEncoder(ew,eh,int(args.cam_fps),int(args.h264_bitrate),str(args.h264_preset),int(args.h264_gop))
        print(f"[encoder] {ew}x{eh} @{args.cam_fps}fps {args.h264_bitrate//1000}kbps {args.h264_preset}")

    cid=f"rm-robot-{args.robot_id}"; md=int(args.max_data_bytes)
    if md>300: md=300
    print(f"[sender] R{args.robot_id} -> {args.host}:{args.port}/{args.topic}  "
          f"mode={args.mode} {args.send_hz}Hz proto={'lib' if using_protobuf_library() else 'manual'}")
    client=mqtt.Client(client_id=cid); client.connect(args.host,args.port,keepalive=10); client.loop_start()
    period=1.0/max(args.send_hz,0.1); pub_msgs=pub_chunks=frame_id=0; last_stat=time.monotonic()

    try:
        while True:
            t0=time.monotonic()
            if args.mode=="h264_file":
                cl=int(args.h264_chunk_len)
                if h264_offset+cl>len(h264_data): h264_offset=0
                raw=h264_data[h264_offset:h264_offset+cl]
                # 不再补零——补零会引入虚假 start code (00 00 00 01) 破坏 H.264 解析
                h264_offset+=cl
                data=pack_fragment(frame_id=frame_id,frag_idx=0,frag_cnt=1,codec=CODEC_H264,flags=0,total_len=len(raw),chunk=raw)
                frame_id=(frame_id+1)&0xFFFF; pub_chunks+=1
            else:
                cl=int(args.h264_chunk_len)
                if len(h264_chunk_buf)<max(int(args.send_hz*0.3),5):
                    ok,bgr=h264_cap.read()
                    if ok and bgr is not None:
                        if args.width>0 and args.height>0 and (bgr.shape[1]!=args.width or bgr.shape[0]!=args.height):
                            import cv2; bgr=cv2.resize(bgr,(int(args.width),int(args.height)),interpolation=cv2.INTER_AREA)
                        try:
                            pkts = h264_enc.encode(bgr)
                            if not pkts:
                                print(f"[encoder] WARN: encode returned 0 packets", flush=True)
                            for pkt in pkts:
                                h264_enc_bytes+=len(pkt)
                                for off in range(0,len(pkt),cl):
                                    c=pkt[off:off+cl]
                                    # 不再补零——补零会引入虚假 start code 破坏 H.264 解析
                                    h264_chunk_buf.append(pack_fragment(frame_id=frame_id,frag_idx=0,frag_cnt=1,codec=CODEC_H264,flags=0,total_len=len(c),chunk=c))
                                    pub_chunks+=1
                            h264_frame_count+=1; frame_id=(frame_id+1)&0xFFFF
                        except Exception as e:
                            print(f"[encoder] ERROR: {e}", flush=True)
                if h264_chunk_buf: data=h264_chunk_buf.popleft()
                else: time.sleep(0.002); continue
            if len(data)>md: data=data[:md]
            client.publish(args.topic,payload=serialize_cbb(data),qos=0,retain=False); pub_msgs+=1
            if args.print_stats and time.monotonic()-last_stat>2.0:
                last_stat=time.monotonic()
                print(f"msgs={pub_msgs} chunks={pub_chunks} " +
                      (f"frames={h264_frame_count} buf={len(h264_chunk_buf)}" if args.mode=="h264_camera" else ""))
            dt=time.monotonic()-t0
            if dt<period: time.sleep(period-dt)
    except KeyboardInterrupt: return 0
    finally:
        if h264_cap is not None: h264_cap.release()
        client.loop_stop()
        try: client.disconnect()
        except: pass

if __name__=="__main__": raise SystemExit(main())
