from pathlib import Path
from statistics import median

root = Path(__file__).resolve().parents[1]
path = root / "p1_timing.txt"
times = []

for number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
    line = raw.strip()
    if not line.startswith("IMU,") or line.startswith("IMU,t_ms,"):
        continue
    fields = line.split(",")
    if len(fields) != 8:
        raise SystemExit(f"FAIL line {number}: {line}")
    try:
        stamp, ax, ay, az, gx, gy, gz = map(int, fields[1:])
    except ValueError:
        raise SystemExit(f"FAIL line {number}: expected integers: {line}")
    times.append(stamp)

if len(times) < 200:
    raise SystemExit(f"FAIL samples={len(times)}, need at least 200")

gaps = [end - start for start, end in zip(times, times[1:])]
if min(gaps) <= 0:
    raise SystemExit(f"FAIL non-increasing timestamps: min_gap={min(gaps)} ms")

rate = 1000.0 * len(gaps) / (times[-1] - times[0])
print(f"TIMING samples={len(times)} rate={rate:.2f} Hz "
      f"min={min(gaps)} ms median={median(gaps):g} ms max={max(gaps)} ms")
if not 19.0 <= rate <= 21.0 or max(gaps) > 75:
    raise SystemExit("FAIL: rate or maximum gap outside pilot limits")
print("PASS: 20 Hz pilot timing")
