import argparse
import time

import serial
from gimbal_stream import GimbalStream

parser = argparse.ArgumentParser()
parser.add_argument("command", choices=list("XPBZCS"))
parser.add_argument("--port", default="COM10")
parser.add_argument("--seconds", type=float, default=3.0)
args = parser.parse_args()

stream = GimbalStream()
with serial.Serial(
        args.port, 9600, timeout=0.2, write_timeout=1.0,
        xonxoff=False, rtscts=False, dsrdtr=False) as port:
    if port.write(args.command.encode("ascii")) != 1:
        raise SystemExit("Command write incomplete")
    port.flush()
    print(f"Sent {args.command}; verify telemetry below")
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        data = port.read(min(max(port.in_waiting, 1), 4096))
        for kind, payload in stream.feed(data):
            if kind == "status":
                print(payload, flush=True)
