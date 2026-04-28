#!/usr/bin/env python3
"""H.264 fragment codec for CustomByteBlock.data (8-byte header + chunk).

Used by: sender, viewer, ros2_mqtt_bridge.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Tuple

CODEC_H264 = 2

_HEADER = struct.Struct("!HBBBBH")
HEADER_LEN = _HEADER.size


def pack_fragment(
    *,
    frame_id: int,
    frag_idx: int,
    frag_cnt: int,
    codec: int,
    flags: int,
    total_len: int,
    chunk: bytes,
) -> bytes:
    """Pack H.264 chunk into fragment (8B header + data)."""
    if not (0 <= frame_id <= 0xFFFF):
        raise ValueError("frame_id must be uint16")
    if frag_cnt <= 0 or frag_cnt > 0xFF:
        raise ValueError("frag_cnt must be 1..255")
    if not (0 <= frag_idx < frag_cnt):
        raise ValueError("frag_idx out of range")
    if not (0 <= codec <= 0xFF and 0 <= flags <= 0xFF):
        raise ValueError("codec/flags must be uint8")
    if not (0 <= total_len <= 0xFFFF):
        raise ValueError("total_len must be uint16")
    return _HEADER.pack(frame_id, frag_idx, frag_cnt, codec, flags, total_len) + chunk


@dataclass(frozen=True)
class FragmentHeader:
    frame_id: int
    frag_idx: int
    frag_cnt: int
    codec: int
    flags: int
    total_len: int


def unpack_fragment(data: bytes) -> Tuple[FragmentHeader, bytes]:
    """Unpack fragment into header + chunk."""
    if len(data) < HEADER_LEN:
        raise ValueError("fragment too small")
    frame_id, frag_idx, frag_cnt, codec, flags, total_len = _HEADER.unpack_from(data, 0)
    if frag_cnt == 0 or frag_idx >= frag_cnt:
        raise ValueError("invalid fragment index/count")
    return FragmentHeader(frame_id, frag_idx, frag_cnt, codec, flags, total_len), data[HEADER_LEN:]
