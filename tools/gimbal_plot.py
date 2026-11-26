import argparse
import time

import matplotlib.pyplot as plt
import serial

from gimbal_protocol import parse_line
from gimbal_record import CsvRecorder
from gimbal_filter_control import FilterControl
from gimbal_target_control import TargetControl
from gimbal_stream import GimbalStream
from gimbal_fast_view import FastView
from gimbal_fast_record import FastCsvRecorder

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM10")
parser.add_argument("--baud", type=int, default=9600)
parser.add_argument("--record", help="New CSV file to create")
parser.add_argument(
    "--fast-record", help="New 20 Hz angle CSV file to create")
args = parser.parse_args()

try:
    port = serial.Serial(
        args.port, args.baud, timeout=0, write_timeout=0.2,
        xonxoff=False, rtscts=False, dsrdtr=False)
except serial.SerialException as exc:
    raise SystemExit(f"Cannot open {args.port}: {exc}")

recorder = None
fast_recorder = None
try:
    if args.record:
        recorder = CsvRecorder(args.record)
    if args.fast_record:
        fast_recorder = FastCsvRecorder(args.fast_record)
except OSError as exc:
    port.close()
    if recorder is not None:
        recorder.close()
    raise SystemExit(f"Cannot create recording: {exc}")

fast_record_error = None

record_error = None

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
fig.subplots_adjust(top=0.50, hspace=0.25)
lines = []
for axis, name in zip(axes, ("Roll", "Pitch", "Yaw")):
    lines.append(axis.plot([], [], label=name)[0])
    axis.set_ylabel(f"{name} (deg)")
    axis.grid(True)
axes[-1].set_xlabel("Device elapsed time (s)")
status = fig.text(0.02, 0.98, "", va="top", fontsize=9)

filter_control = FilterControl(fig, port)
p_target_control = TargetControl(fig, port, "P", 0.645, 0.695, with_stop=True)
y_target_control = TargetControl(fig, port, "Y", 0.555, 0.605)
p_target_control.peers.append(y_target_control)
y_target_control.peers.append(p_target_control)

stream = GimbalStream()
fast_view = FastView()
history = fast_view.history
started = time.monotonic()
last_rx = last_progress = started
last_n = None
latest = None
ignored = 0
link_error = None
target_error = None


def axis_text(name, values):
    state, enabled, ready, feedback, pulse, fault = values
    angle = f"{feedback / 1000:.2f}" if ready else "NA"
    return (f"{name}: state={state} en={enabled} ref={ready} "
            f"angle={angle} deg pulse={pulse} us fault={fault}")

def refresh():
    global latest, last_rx, last_progress, last_n
    global ignored, link_error, target_error
    global record_error
    global fast_record_error
    now = time.monotonic()

    if link_error is None:
        try:
            data = port.read(min(port.in_waiting, 4096))
            for kind, payload in stream.feed(data):
                if kind == "fast":
                    fast_view.accept(payload, now)
                    if (fast_recorder is not None
                            and fast_record_error is None):
                        try:
                            fast_recorder.write(
                                payload, now - started)
                        except OSError as exc:
                            fast_record_error = str(exc)
                            print(
                                f"Fast recording failed: {exc}",
                                flush=True)
                    continue

                raw = payload.encode("ascii")
                try:
                    frame = parse_line(raw)
                except (ValueError, KeyError, UnicodeError):
                    ignored += 1
                    continue

                latest = frame
                last_rx = now
                filter_control.accept(frame)
                if target_error is None:
                    try:
                        p_target_control.accept(frame)
                        y_target_control.accept(frame)
                    except Exception as exc:
                        target_error = f"{type(exc).__name__}: {exc}"
                        print(f"Target control failed: {exc}",
                              flush=True)

                if recorder is not None and record_error is None:
                    try:
                        recorder.write(frame, raw, now - started)
                    except OSError as exc:
                        record_error = str(exc)
                        print(f"Recording failed: {exc}", flush=True)

                if frame["n"] != last_n:
                    last_n = frame["n"]
                    last_progress = now
        except (serial.SerialException, OSError) as exc:
            link_error = str(exc)

    if link_error is not None:
        message = f"DISCONNECTED: {link_error}"
        color = "red"
    elif latest is None:
        message, color = "WAITING FOR TELEMETRY", "darkorange"
    else:
        stale = now - last_rx > 3.0
        fast_stalled = (
            fast_view.last_progress is None
            or now - fast_view.last_progress > 3.0)
        stalled = now - last_progress > 3.0 or fast_stalled
        label = "NO DATA" if stale else (
            "SAMPLE STALLED" if stalled else "LIVE")
        color = "red" if stale or stalled else "black"
        sample_ms = latest.get("device_sample_ms")
        device_time = "NA" if sample_ms is None else f"{sample_ms}ms"
        message = (f"{label}  N={latest['n']} T={device_time} "
                   f"CAL={latest['cal']} ATT={latest['att']} "
                   f"ignored={ignored}\n"
                   + axis_text("P", latest["P"]) + "\n"
                   + axis_text("Y", latest["Y"]))

    fast_dt = (
        "NA" if fast_view.dt_min is None
        else f"{fast_view.dt_min}..{fast_view.dt_max}ms")
    message += (
        f"\nFAST rx={fast_view.received} "
        f"missing={fast_view.missing} dt={fast_dt} "
        f"resets={fast_view.resets}")

    if record_error is not None:
        message += f"\nREC ERROR: {record_error}"
        color = "red"
    elif recorder is not None:
        message += "\nREC ON"

    if target_error is not None:
        message += f"\nTARGET ERROR: {target_error}"
        color = "red"

    if fast_record_error is not None:
        message += f"\nFAST REC ERROR: {fast_record_error}"
        color = "red"
    elif fast_recorder is not None:
        message += "\nFAST REC ON (20 Hz)"

    status.set_text(message)
    status.set_color(color)

    if history:
        columns = list(zip(*history))
        for index, axis in enumerate(axes):
            lines[index].set_data(columns[0], columns[index + 1])
            axis.relim()
            axis.autoscale_view()
    live = (
        latest is not None and link_error is None
        and now - last_rx <= 3.0
        and now - last_progress <= 3.0
        and fast_view.last_progress is not None
        and now - fast_view.last_progress <= 3.0)
    filter_control.update(now, live)
    if target_error is None:
        try:
            p_target_control.update(now, live)
            y_target_control.update(now, live)
        except Exception as exc:
            target_error = f"{type(exc).__name__}: {exc}"
            print(f"Target control failed: {exc}", flush=True)
    if target_error is not None:
        for control in (p_target_control, y_target_control):
            control.live = False
            for button in control.buttons:
                button.set_active(False)

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
    if fast_recorder is not None:
        fast_recorder.close()
