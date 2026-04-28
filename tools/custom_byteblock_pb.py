#!/usr/bin/env python3
"""Protobuf 序列化/反序列化辅助模块 (RoboMaster CustomByteBlock).

优先使用 protoc 编译生成的 Python 类，未编译时回退到手动 varint 编码。
两种方式生成的二进制流完全兼容。

使用方法:
    # 1. 先尝试编译 proto 文件 (可选但推荐):
    #    protoc --python_out=tools tools/custom_byteblock.proto
    #
    # 2. 在代码中导入:
    from custom_byteblock_pb import serialize_cbb, parse_cbb, serialize_cbbe, parse_cbbe
"""

from __future__ import annotations

from typing import Optional, Tuple

# ── 尝试导入编译后的 protobuf ──
_USE_PROTOBUF = False
_CustomByteBlock = None

try:
    from custom_byteblock_pb2 import CustomByteBlock as _CBB  # type: ignore
    _CustomByteBlock = _CBB
    _USE_PROTOBUF = True
except ImportError:
    try:
        from google.protobuf import descriptor_pb2, descriptor_pool, message_factory  # type: ignore

        _pool = descriptor_pool.Default()
        _file_proto = descriptor_pb2.FileDescriptorProto()
        _file_proto.name = "custom_byteblock.proto"
        _file_proto.package = "rm_custom"
        _file_proto.syntax = "proto3"

        _cbb_msg = _file_proto.message_type.add()
        _cbb_msg.name = "CustomByteBlock"
        _cbb_f1 = _cbb_msg.field.add()
        _cbb_f1.name = "data"
        _cbb_f1.number = 1
        _cbb_f1.type = descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
        _cbb_f1.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

        _serialized = _file_proto.SerializeToString()
        _pool.AddSerializedFile(_serialized)

        _factory = message_factory.MessageFactory(pool=_pool)
        _CustomByteBlock = _factory.GetPrototype(_pool.FindMessageTypeByName("rm_custom.CustomByteBlock"))
        _USE_PROTOBUF = True
    except ImportError:
        pass


# ── 手动 varint 编码 (回退方案) ──

def _encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("varint must be non-negative")
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


def _manual_serialize(data: bytes) -> bytes:
    """手动序列化 CustomByteBlock: field 1, wire type 2 => key = 0x0A"""
    return b"\x0A" + _encode_varint(len(data)) + data


def _manual_parse(protobuf_payload: bytes) -> bytes:
    """手动解析 CustomByteBlock, 提取 field 1 的 bytes."""
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


# ── 公开 API ─────────────────────────────────────────────────────────

def serialize_cbb(data: bytes) -> bytes:
    """序列化 CustomByteBlock 消息。

    Args:
        data: 要放入 data 字段的原始字节 (最大 300 bytes, 对应 2.4kbit 限制)

    Returns:
        Protobuf 序列化后的二进制流, 可直接通过 MQTT publish
    """
    if _USE_PROTOBUF and _CustomByteBlock is not None:
        msg = _CustomByteBlock()
        msg.data = data
        return msg.SerializeToString()
    return _manual_serialize(data)


def parse_cbb(protobuf_payload: bytes) -> bytes:
    """反序列化 CustomByteBlock 消息, 提取 data 字段。

    Args:
        protobuf_payload: MQTT 收到的原始二进制 payload

    Returns:
        data 字段的内容 (bytes), 可能为空
    """
    if _USE_PROTOBUF and _CustomByteBlock is not None:
        msg = _CustomByteBlock()
        msg.ParseFromString(protobuf_payload)
        return msg.data or b""
    return _manual_parse(protobuf_payload)


def using_protobuf_library() -> bool:
    """是否成功加载了 protobuf 库（而非手动 varint 回退）。"""
    return _USE_PROTOBUF
