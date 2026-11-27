import csv
import math
import sys
from itertools import islice
from pathlib import Path

WINDOW = 100

if len(sys.argv) != 2:
    raise SystemExit("Usage: python tools/first_window.py <raw-data-folder>")

source = Path(sys.argv[1]) / "wlk_7" / "sub_1.csv"
window = []


def clip(value, limit):
    return max(-limit, min(limit, value))


with source.open(encoding="utf-8-sig", newline="") as file:
    for row in islice(csv.DictReader(file), WINDOW):
        accel = [float(row[f"gravity.{axis}"]) +
                 float(row[f"userAcceleration.{axis}"]) for axis in "xyz"]
        gyro = [math.degrees(float(row[f"rotationRate.{axis}"])) for axis in "xyz"]
        if not all(math.isfinite(value) for value in accel + gyro):
            raise SystemExit(f"FAIL: non-finite value in {source}")
        accel_norm = math.hypot(*(clip(value, 8.0) for value in accel))
        gyro_norm = math.hypot(*(clip(value, 1000.0) for value in gyro))
        window.append((accel_norm, gyro_norm))

if len(window) != WINDOW:
    raise SystemExit(f"FAIL: got {len(window)} samples, expected {WINDOW}")

target = Path("p1_first_window.csv")
with target.open("w", encoding="utf-8", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["sample_index", "accel_norm_g", "gyro_norm_dps"])
    writer.writerows((i, *values) for i, values in enumerate(window))

print(f"WINDOW label=walk subject=1 samples={len(window)} channels=2 file={target}")
