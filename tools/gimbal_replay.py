import argparse
import csv
import math

import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

times = []
angles = [[], [], []]
last_n = None
last_elapsed = None

try:
    with open(args.filename, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            elapsed = float(row["pc_elapsed_s"])
            n = int(row["n"])
            values = [
                float(row[name])
                for name in ("roll_deg", "pitch_deg", "yaw_deg")]
            if not all(math.isfinite(x) for x in [elapsed, *values]):
                raise ValueError("non-finite data")
            reset = last_n is not None and n < last_n
            gap = (
                last_elapsed is not None
                and elapsed - last_elapsed > 3.0
            )
            if reset or gap:
                times.append(elapsed)
                for values_list in angles:
                    values_list.append(float("nan"))
            times.append(elapsed)
            for values_list, value in zip(angles, values):
                values_list.append(value)
            last_n = n
            last_elapsed = elapsed
except (OSError, ValueError, KeyError, csv.Error) as exc:
    raise SystemExit(f"Cannot read recording: {exc}")

if not times:
    raise SystemExit("Recording contains no samples")

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
for axis, values, name in zip(
        axes, angles, ("Roll", "Pitch", "Yaw")):
    axis.plot(times, values)
    axis.set_ylabel(f"{name} (deg)")
    axis.grid(True)

axes[-1].set_xlabel("PC receive elapsed time (s)")
fig.suptitle(args.filename)
fig.tight_layout()
plt.show()
