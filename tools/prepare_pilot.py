import csv
import sys
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("Usage: python tools/prepare_pilot.py data/pilot/pilot_still.txt")

source = Path(sys.argv[1])
labels = {"pilot_still": "still", "pilot_walk": "walk", "pilot_run": "run"}
if source.stem not in labels or source.suffix.lower() != ".txt":
    raise SystemExit("FAIL: use pilot_still.txt, pilot_walk.txt or pilot_run.txt")

rows = []
for number, raw in enumerate(source.read_text(encoding="utf-8-sig").splitlines(), 1):
    line = raw.strip()
    if not line.startswith("IMU,") or line.startswith("IMU,t_ms,"):
        continue
    fields = line.split(",")
    if len(fields) != 8:
        raise SystemExit(f"FAIL line {number}: {line}")
    try:
        rows.append(tuple(map(int, fields[1:])))
    except ValueError:
        raise SystemExit(f"FAIL line {number}: expected integers: {line}")

if len(rows) < 2:
    raise SystemExit("FAIL: no usable recording")
gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
if min(gaps) <= 0 or max(gaps) > 75:
    raise SystemExit(f"FAIL: min_gap={min(gaps)} ms max_gap={max(gaps)} ms")
rate = 1000 * len(gaps) / (rows[-1][0] - rows[0][0])
if not 19 <= rate <= 21:
    raise SystemExit(f"FAIL: rate={rate:.2f} Hz")

start, end = rows[0][0] + 2000, rows[-1][0] - 2000
kept = [row for row in rows if start <= row[0] <= end]
if len(kept) < 2 or kept[-1][0] - kept[0][0] < 15000:
    raise SystemExit("FAIL: need at least 15 seconds after trimming")

target = source.with_suffix(".csv")
label = labels[source.stem]
with target.open("w", newline="", encoding="utf-8") as output:
    writer = csv.writer(output)
    writer.writerow(["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "label"])
    writer.writerows((*row, label) for row in kept)
print(f"PASS label={label} rows={len(kept)} rate={rate:.2f} Hz file={target}")
