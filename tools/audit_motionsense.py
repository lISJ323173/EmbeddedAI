import csv
import math
import sys
from collections import Counter
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("Usage: python tools/audit_motionsense.py <raw-data-folder>")

root = Path(sys.argv[1])
trials = {
    "wlk_7": "walk", "wlk_8": "walk", "wlk_15": "walk",
    "jog_9": "run", "jog_16": "run",
    "std_6": "still", "std_14": "still",
    "sit_5": "still", "sit_13": "still",
}
prefixes = ("gravity", "userAcceleration", "rotationRate")
cols = {f"{prefix}.{axis}" for prefix in prefixes for axis in "xyz"}
manifest, files, totals = [], Counter(), Counter()
train_rows, accel_hits, gyro_hits = Counter(), Counter(), Counter()

for subject in range(1, 25):
    split = "train" if subject <= 16 else "val" if subject <= 20 else "test"
    for trial, label in trials.items():
        relative = Path(trial) / f"sub_{subject}.csv"
        with (root / relative).open(encoding="utf-8-sig", newline="") as file:
            reader = csv.DictReader(file)
            if not cols.issubset(reader.fieldnames or []):
                raise SystemExit(f"FAIL missing columns: {relative}")
            n = 0
            for row in reader:
                n += 1
                if split == "train":
                    accel = [float(row[f"gravity.{a}"]) +
                             float(row[f"userAcceleration.{a}"]) for a in "xyz"]
                    gyro = [math.degrees(float(row[f"rotationRate.{a}"])) for a in "xyz"]
                    if not all(math.isfinite(v) for v in accel + gyro):
                        raise SystemExit(f"FAIL non-finite data: {relative} row {n}")
                    accel_hits[label] += max(map(abs, accel)) > 8.0
                    gyro_hits[label] += max(map(abs, gyro)) > 1000.0
        if n == 0:
            raise SystemExit(f"FAIL empty file: {relative}")
        files[split] += 1
        totals[label] += n
        if split == "train":
            train_rows[label] += n
        manifest.append([split, subject, label, relative.as_posix(), n])

with Path("motionsense_split.csv").open("w", encoding="utf-8", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["split", "subject", "label", "source", "rows"])
    writer.writerows(manifest)

print(f"FILES train={files['train']} val={files['val']} test={files['test']}")
for label in ("still", "walk", "run"):
    n = train_rows[label]
    print(f"{label}: rows={totals[label]} train_acc_over={accel_hits[label]/n:.2%} "
          f"train_gyro_over={gyro_hits[label]/n:.2%}")
