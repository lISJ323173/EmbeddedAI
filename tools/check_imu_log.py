from pathlib import Path
from statistics import median

ROOT = Path(__file__).resolve().parents[1]
AXES = ("ax", "ay", "az", "gx", "gy", "gz")


def load_samples(name):
    samples = []
    for line in (ROOT / name).read_text(encoding="utf-8-sig").splitlines():
        line = line.strip()
        if not line.startswith("IMU,") or line.startswith("IMU,ax,"):
            continue
        fields = line.split(",")
        if len(fields) != 7:
            raise SystemExit(f"{name}: invalid IMU row: {line}")
        try:
            samples.append(tuple(int(value) for value in fields[1:]))
        except ValueError:
            raise SystemExit(f"{name}: non-integer IMU value: {line}")
    if len(samples) < 20:
        raise SystemExit(f"{name}: only {len(samples)} samples, need 20")
    return samples


flat = load_samples("p0_flat.txt")
flip = load_samples("p0_flip.txt")
az_flat = median(row[2] for row in flat[-20:])
az_flip = median(row[2] for row in flip[-20:])

print(f"ROWS flat={len(flat)} flip={len(flip)}")
print(f"AZ flat={az_flat:+.0f} flipped={az_flip:+.0f}")
spans = [max(axis) - min(axis) for axis in zip(*flip)]
print("FLIP_SPAN", " ".join(f"{axis}={span}" for axis, span in zip(AXES, spans)))
if az_flat * az_flip >= 0:
    raise SystemExit("FAIL: az did not change sign")
print("PASS: six integer columns and az sign reversal")
