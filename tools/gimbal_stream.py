from gimbal_fast_protocol import decode_fast, PACKET_SIZE

_SYNC = b"\xa5\x5a"


class GimbalStream:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while self.buffer:
            binary = self.buffer.find(_SYNC)
            text = self.buffer.find(b"N=")
            starts = [p for p in (binary, text) if p >= 0]
            if not starts:
                tail = bytes(self.buffer[-1:])
                self.buffer.clear()
                if tail in (b"\xa5", b"N"):
                    self.buffer.extend(tail)
                break
            del self.buffer[:min(starts)]
            if self.buffer.startswith(_SYNC):
                if len(self.buffer) < PACKET_SIZE:
                    break
                try:
                    frame = decode_fast(bytes(self.buffer[:PACKET_SIZE]))
                except ValueError:
                    del self.buffer[0]
                    continue
                del self.buffer[:PACKET_SIZE]
                records.append(("fast", frame))
                continue
            newline = self.buffer.find(b"\n")
            binary = self.buffer.find(_SYNC)
            if binary >= 0 and (newline < 0 or binary < newline):
                del self.buffer[:binary]
                continue
            if newline < 0:
                if len(self.buffer) > 768:
                    del self.buffer[0]
                    continue
                break
            raw = bytes(self.buffer[:newline])
            del self.buffer[:newline + 1]
            if len(raw) > 768:
                continue
            try:
                line = raw.rstrip(b"\r").decode("ascii")
            except UnicodeDecodeError:
                continue
            records.append(("status", line))
        return records
