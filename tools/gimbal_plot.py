import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
import serial

from gimbal_protocol import parse_line
from gimbal_record import CsvRecorder

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM10")
parser.add_argument("--baud", type=int, default=9600)
parser.add_argument("--record", help="New CSV file to create")
args = parser.parse_args()

try:
    port = serial.Serial(
        args.port, args.baud, timeout=0,
        xonxoff=False, rtscts=False, dsrdtr=False)
except serial.SerialException as exc:
    raise SystemExit(f"Cannot open {args.port}: {exc}")

try:
    recorder = CsvRecorder(args.record) if args.record else None
except OSError as exc:
    port.close()
    raise SystemExit(f"Cannot create recording: {exc}")

record_error = None

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
fig.subplots_adjust(top=0.78, hspace=0.25)
lines = []
for axis, name in zip(axes, ("Roll", "Pitch", "Yaw")):
    lines.append(axis.plot([], [], label=name)[0])
    axis.set_ylabel(f"{name} (deg)")
    axis.grid(True)
axes[-1].set_xlabel("PC elapsed time (s)")
status = fig.text(0.02, 0.98, "", va="top", fontsize=9)

history = deque(maxlen=300)
buffer = bytearray()
started = time.monotonic()
last_rx = last_progress = started
last_n = None
latest = None
ignored = 0
link_error = None


def axis_text(name, values):
    state, enabled, ready, feedback, pulse, fault = values
    angle = f"{feedback / 1000:.2f}" if ready else "NA"
    return (f"{name}: state={state} en={enabled} ref={ready} "
            f"angle={angle} deg pulse={pulse} us fault={fault}")

def refresh():
    global buffer, latest, last_rx, last_progress, last_n
    global ignored, link_error
    global record_error
    now = time.monotonic()

    if link_error is None:
        try:
            buffer.extend(port.read(min(port.in_waiting, 4096)))
            while b"\n" in buffer:
                raw, _, rest = buffer.partition(b"\n")
                buffer = bytearray(rest)
                try:
                    if len(raw) > 512:
                        raise ValueError("line too long")
                    frame = parse_line(raw)
                except (ValueError, KeyError, UnicodeError):
                    ignored += 1
                    continue
                latest = frame
                last_rx = now
                if recorder is not None and record_error is None:
                    try:
                        recorder.write(frame, raw, now - started)
                    except OSError as exc:
                        record_error = str(exc)
                        print(f"Recording failed: {exc}", flush=True)
                if frame["n"] != last_n:
                    if last_n is not None:
                        if frame["n"] < last_n:
                            history.clear()
                        elif now - last_progress > 3.0:
                            history.append((
                                now - started,
                                float("nan"),
                                float("nan"),
                                float("nan")))
                    last_n = frame["n"]
                    last_progress = now
                    history.append((now - started, *frame["rpy"]))
            if len(buffer) > 512:
                buffer.clear()
                ignored += 1
        except (serial.SerialException, OSError) as exc:
            link_error = str(exc)

    if link_error is not None:
        message = f"DISCONNECTED: {link_error}"
        color = "red"
    elif latest is None:
        message, color = "WAITING FOR TELEMETRY", "darkorange"
    else:
        stale = now - last_rx > 3.0
        stalled = now - last_progress > 3.0
        label = "NO DATA" if stale else (
            "SAMPLE STALLED" if stalled else "LIVE")
        color = "red" if stale or stalled else "black"
        message = (f"{label}  N={latest['n']} "
                   f"CAL={latest['cal']} ATT={latest['att']} "
                   f"ignored={ignored}\n"
                   + axis_text("P", latest["P"]) + "\n"
                   + axis_text("Y", latest["Y"]))

    if record_error is not None:
        message += f"\nREC ERROR: {record_error}"
        color = "red"
    elif recorder is not None:
        message += "\nREC ON"

    status.set_text(message)
    status.set_color(color)

    if history:
        columns = list(zip(*history))
        for index, axis in enumerate(axes):
            lines[index].set_data(columns[0], columns[index + 1])
            axis.relim()
            axis.autoscale_view()
    fig.canvas.draw_idle()

timer = fig.canvas.new_timer(interval=100)
timer.add_callback(refresh)
fig.canvas.mpl_connect("close_event", lambda event: timer.stop())
timer.start()

try:
    plt.show()
finally:
    timer.stop()
    port.close()
    if recorder is not None:
        recorder.close()
