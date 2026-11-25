import argparse
import csv
import math
import statistics

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()
groups, current = [], []
previous = None

try:
    with open(args.filename, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            t = float(row["pc_elapsed_s"])
            n = int(row["n"])
            key = (int(row["filter_gain_milli"]),
                   int(row["filter_apply_count"]))
            angles = tuple(float(row[f"{axis}_deg"])
                           for axis in ("roll", "pitch", "yaw"))
            valid = (
                int(row["cal"]) == 1 and int(row["att"]) == 1
                and int(row["filter_apply_status"]) == 1
                and int(row["p_state"]) != 1
                and int(row["y_state"]) != 1
                and int(row["p_enabled"]) == 0
                and int(row["y_enabled"]) == 0)
            if not all(math.isfinite(v) for v in (t, *angles)):
                raise ValueError("Non-finite data")
            boundary = (
                previous is None or key != previous[2]
                or t <= previous[0] or t - previous[0] > 3.0
                or n <= previous[1])
            if (not valid or boundary) and current:
                groups.append(current)
                current = []
            if not valid:
                previous = None
                continue
            current.append((t, key[0], *angles))
            previous = (t, n, key)
except (OSError, KeyError, TypeError, ValueError) as exc:
    raise SystemExit(f"Cannot analyze recording: {exc}")

if current:
    groups.append(current)

usable = 0
for index, group in enumerate(groups, 1):
    samples = [r for r in group if r[0] - group[0][0] >= 10.0]
    if len(samples) < 20 or samples[-1][0] - samples[0][0] < 20:
        print(f"Segment {index}: too short after settling; skipped")
        continue

    duration = samples[-1][0] - samples[0][0]
    roll = [r[2] for r in samples]
    pitch = [r[3] for r in samples]
    yaw = [samples[0][4]]
    for before, after in zip(samples, samples[1:]):
        yaw.append(yaw[-1] + math.remainder(after[4] - before[4], 360))
    yaw_delta = yaw[-1] - yaw[0]

    print(
        f"Segment {index}: gain={samples[0][1] / 1000:g}, "
        f"n={len(samples)}, duration={duration:.1f}s\n"
        f"  Roll: pp={max(roll) - min(roll):.4f} deg, "
        f"std={statistics.stdev(roll):.4f} deg\n"
        f"  Pitch: pp={max(pitch) - min(pitch):.4f} deg, "
        f"std={statistics.stdev(pitch):.4f} deg\n"
        f"  Yaw: delta={yaw_delta:+.4f} deg, "
        f"average_rate={yaw_delta / duration:+.5f} deg/s")
    usable += 1

if not usable:
    raise SystemExit("No sufficiently long valid stationary segments")
