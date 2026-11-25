import binascii
import struct

_BODY = struct.Struct("<2sBBHIhhh")
PACKET_SIZE = _BODY.size + 2


def decode_fast(packet):
    """Decode one complete fast packet; angles are returned in degrees."""
    if len(packet) != PACKET_SIZE:
        raise ValueError("bad fast packet length")

    sync, version, flags, seq, sample_ms, roll, pitch, yaw = (
        _BODY.unpack_from(packet)
    )
    if sync != b"\xa5\x5a" or version != 1 or flags & ~3:
        raise ValueError("bad fast packet header")

    expected = struct.unpack_from("<H", packet, _BODY.size)[0]
    actual = binascii.crc_hqx(packet[:_BODY.size], 0xffff)
    if actual != expected:
        raise ValueError("bad fast packet CRC")

    if abs(roll) > 18000 or abs(pitch) > 9000 or abs(yaw) > 18000:
        raise ValueError("bad fast packet angles")

    return {
        "fast_seq": seq,
        "device_sample_ms": sample_ms,
        "cal": flags & 1,
        "att_valid": bool(flags & 2),
        "rpy": (roll / 100.0, pitch / 100.0, yaw / 100.0),
    }
