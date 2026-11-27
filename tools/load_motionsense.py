"""Convert MotionSense A_DeviceMotion_data into the board's IMU CSV format.

Board units: accel +-8g -> 4096 LSB/g ; gyro +-1000 dps -> 32.8 LSB/(deg/s).
MotionSense: accel is gravity + userAcceleration in g, rotationRate in rad/s, 50 Hz.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

ROOT = Path(r"E:\imu-motion-ai\data\external\MotionSense\A_DeviceMotion_data")
OUT = Path(r"E:\imu-motion-ai\data\external\converted")

TRIALS = {"wlk": (7, 8, 15), "jog": (9, 16), "std": (6, 14), "sit": (5, 13)}
LABEL = {"wlk": "walk", "jog": "run", "std": "still", "sit": "still"}
PERIOD_MS = 20  # 50 Hz

ACC_LSB_PER_G = 4310.0  # 实测：正面朝上静置 1 g = 4310 LSB（±8 g 档）
GYRO_LSB_PER_RAD = 32.8 * 180.0 / math.pi


def convert(path: Path, out_dir: Path) -> tuple[str, int]:
    label = LABEL[path.parent.name.split("_")[0]]
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            ax = float(row["gravity.x"]) + float(row["userAcceleration.x"])
            ay = float(row["gravity.y"]) + float(row["userAcceleration.y"])
            az = float(row["gravity.z"]) + float(row["userAcceleration.z"])
            gx = float(row["rotationRate.x"]) * GYRO_LSB_PER_RAD
            gy = float(row["rotationRate.y"]) * GYRO_LSB_PER_RAD
            gz = float(row["rotationRate.z"]) * GYRO_LSB_PER_RAD
            rows.append(
                (
                    index * PERIOD_MS,
                    round(ax * ACC_LSB_PER_G),
                    round(ay * ACC_LSB_PER_G),
                    round(az * ACC_LSB_PER_G),
                    round(gx),
                    round(gy),
                    round(gz),
                )
            )
    if not rows:
        return label, 0
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / f"{path.parent.name}_{path.stem}_{label}.csv"
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "label"])
        writer.writerows((*row, label) for row in rows)
    return label, len(rows)


def main() -> int:
    if not ROOT.is_dir():
        raise SystemExit(f"FAIL: dataset root not found: {ROOT}")
    summary: dict[str, dict[str, int]] = {}
    files = 0
    for act, trials in TRIALS.items():
        for trial in trials:
            folder = ROOT / f"{act}_{trial}"
            if not folder.is_dir():
                print(f"WARN missing folder {folder.name}")
                continue
            for csv_path in sorted(folder.glob("sub_*.csv")):
                label, count = convert(csv_path, OUT)
                if count == 0:
                    continue
                files += 1
                bucket = summary.setdefault(label, {"rows": 0, "files": 0, "subjects": 0})
                bucket["rows"] += count
                bucket["files"] += 1
                subject = csv_path.stem.replace("sub_", "")
                bucket.setdefault("_ids", set()).add(subject)

    if not summary:
        raise SystemExit("FAIL: nothing converted")

    total = 0
    for label in sorted(summary):
        bucket = summary[label]
        ids = bucket.pop("_ids")
        bucket["subjects"] = len(ids)
        total += bucket["rows"]
        print(
            f"{label:5s} rows={bucket['rows']:7d} files={bucket['files']:3d} "
            f"subjects={bucket['subjects']:2d} "
            f"seconds={bucket['rows'] / 50.0:7.1f}"
        )
    print(f"TOTAL rows={total} files={files} out={OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
