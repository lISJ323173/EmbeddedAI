import csv
import math
import sys
from collections import Counter
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("Usage: python tools/build_features.py <raw-data-folder>")

root = Path(sys.argv[1])
window, stride = 100, 50
names = [f"{channel}_{stat}" for channel in ("acc", "gyro")
         for stat in ("mean", "std", "max", "mean_delta")]


def norm(values, limit):
    if not all(math.isfinite(v) for v in values):
        raise SystemExit("FAIL: non-finite sensor value")
    return math.hypot(*(max(-limit, min(limit, v)) for v in values))


def summarize(values):
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / n)
    delta = sum(abs(b - a) for a, b in zip(values, values[1:])) / (n - 1)
    return mean, std, max(values), delta


output, counts = [], Counter()
with Path("motionsense_split.csv").open(encoding="utf-8-sig", newline="") as source:
    for item in csv.DictReader(source):
        acc, gyro = [], []
        with (root / item["source"]).open(encoding="utf-8-sig", newline="") as file:
            for row in csv.DictReader(file):
                a = [float(row[f"gravity.{k}"]) + float(row[f"userAcceleration.{k}"])
                     for k in "xyz"]
                g = [math.degrees(float(row[f"rotationRate.{k}"])) for k in "xyz"]
                acc.append(norm(a, 8.0))
                gyro.append(norm(g, 1000.0))
        if len(acc) != int(item["rows"]):
            raise SystemExit(f"FAIL: row count changed in {item['source']}")
        key = (item["split"], item["label"])
        for start in range(0, len(acc) - window + 1, stride):
            end = start + window
            features = (*summarize(acc[start:end]), *summarize(gyro[start:end]))
            output.append([*key, item["subject"], item["source"], start, *features])
            counts[key] += 1

with Path("motionsense_features.csv").open("w", encoding="utf-8", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["split", "label", "subject", "source", "start", *names])
    writer.writerows(output)

print(f"FEATURES rows={len(output)} inputs={len(names)}")
for split in ("train", "val", "test"):
    print(split, " ".join(f"{label}={counts[split, label]}"
                          for label in ("still", "walk", "run")))
