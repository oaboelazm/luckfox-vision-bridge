from __future__ import annotations

import struct

from .crc import crc16_ccitt_false
from .types import GuidanceFrame, StatusFrame

GUIDANCE_HEADER = 0xAA55
STATUS_HEADER = 0x55AA
GUIDANCE_MSG_TYPE = 0x01
STATUS_MSG_TYPE = 0x81

GUIDANCE_FMT_NO_CRC = "<H B H I B f f f f f B"
GUIDANCE_FMT = "<H B H I B f f f f f B H"
STATUS_FMT_NO_CRC = "<H B H B B f B"
STATUS_FMT = "<H B H B B f B H"

GUIDANCE_SIZE = struct.calcsize(GUIDANCE_FMT)
STATUS_SIZE = struct.calcsize(STATUS_FMT)


class ProtocolError(Exception):
    pass


def pack_guidance(seq: int, frame: GuidanceFrame, emergency_hover: bool) -> bytes:
    flags = 0
    if frame.target_valid:
        flags |= 0x01
    if emergency_hover:
        flags |= 0x02

    payload = struct.pack(
        GUIDANCE_FMT_NO_CRC,
        GUIDANCE_HEADER,
        GUIDANCE_MSG_TYPE,
        seq & 0xFFFF,
        frame.timestamp_ms & 0xFFFFFFFF,
        frame.target_type & 0xFF,
        float(frame.dx_norm),
        float(frame.dy_norm),
        float(frame.z_est_m),
        float(frame.yaw_err_deg),
        float(frame.confidence),
        flags,
    )
    crc = crc16_ccitt_false(payload)
    return payload + struct.pack("<H", crc)


def unpack_guidance(packet: bytes) -> GuidanceFrame:
    if len(packet) != GUIDANCE_SIZE:
        raise ProtocolError(f"Invalid guidance length {len(packet)}")
    values = struct.unpack(GUIDANCE_FMT, packet)
    crc_expected = crc16_ccitt_false(packet[:-2])
    if values[-1] != crc_expected:
        raise ProtocolError("Guidance CRC mismatch")
    if values[0] != GUIDANCE_HEADER or values[1] != GUIDANCE_MSG_TYPE:
        raise ProtocolError("Guidance header/type mismatch")

    flags = values[11]
    return GuidanceFrame(
        timestamp_ms=values[3],
        target_type=values[4],
        dx_norm=values[5],
        dy_norm=values[6],
        z_est_m=values[7],
        yaw_err_deg=values[8],
        confidence=values[9],
        target_valid=bool(flags & 0x01),
    )


def pack_status(frame: StatusFrame) -> bytes:
    payload = struct.pack(
        STATUS_FMT_NO_CRC,
        STATUS_HEADER,
        STATUS_MSG_TYPE,
        frame.seq_echo & 0xFFFF,
        frame.mode & 0xFF,
        frame.control_state & 0xFF,
        float(frame.battery_v),
        frame.ack_code & 0xFF,
    )
    crc = crc16_ccitt_false(payload)
    return payload + struct.pack("<H", crc)


def unpack_status(packet: bytes) -> StatusFrame:
    if len(packet) != STATUS_SIZE:
        raise ProtocolError(f"Invalid status length {len(packet)}")
    values = struct.unpack(STATUS_FMT, packet)
    crc_expected = crc16_ccitt_false(packet[:-2])
    if values[-1] != crc_expected:
        raise ProtocolError("Status CRC mismatch")
    if values[0] != STATUS_HEADER or values[1] != STATUS_MSG_TYPE:
        raise ProtocolError("Status header/type mismatch")

    return StatusFrame(
        seq_echo=values[2],
        mode=values[3],
        control_state=values[4],
        battery_v=values[5],
        ack_code=values[6],
    )


def parse_fixed_packets(buffer: bytearray, expected_header: int, packet_size: int) -> list[bytes]:
    packets: list[bytes] = []
    while len(buffer) >= packet_size:
        header = int.from_bytes(buffer[:2], "little")
        if header != expected_header:
            del buffer[0]
            continue
        chunk = bytes(buffer[:packet_size])
        packets.append(chunk)
        del buffer[:packet_size]
    return packets
