import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
cfg = json.loads((root / "artifacts/motion_preprocess.json").read_text(encoding="utf-8"))
names = [f"{channel}_{stat}" for channel in ("acc", "gyro")
         for stat in ("mean", "std", "max", "mean_delta")]
if cfg["features"] != names or cfg["labels"] != ["still", "walk", "run"]:
    raise SystemExit("FAIL: unexpected feature or label order")

lines = ["#pragma once", "", "namespace motion_model {"]
for key in ("mean", "std"):
    values = ", ".join(f"{value:.9e}f" for value in cfg[key])
    lines.append(f"inline constexpr float feature_{key}[8] = {{{values}}};")
lines += ["}", ""]
target = root / "config/motion_preprocess.hpp"
target.write_text("\n".join(lines), encoding="utf-8")
print(f"PREPROCESS OK {target}")
