import csv
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

root = Path(__file__).resolve().parents[1]
out = root / "artifacts"
cfg = json.loads((out / "motion_preprocess.json").read_text(encoding="utf-8"))
with (root / "motionsense_features.csv").open(encoding="utf-8-sig", newline="") as file:
    row = next(r for r in csv.DictReader(file)
               if r["split"] == "val" and r["label"] == "walk")

x = np.array([float(row[k]) for k in cfg["features"]], dtype=np.float32)
mean = np.array(cfg["mean"], dtype=np.float32)
std = np.array(cfg["std"], dtype=np.float32)
engine = tf.lite.Interpreter(model_path=str(out / "motion_int8.tflite"))
engine.allocate_tensors()
inp, result = engine.get_input_details()[0], engine.get_output_details()[0]
scale, zero = inp["quantization"]
quant = np.clip(np.rint(((x - mean) / std) / scale) + zero, -128, 127).astype(np.int8)
engine.set_tensor(inp["index"], quant[None, :])
engine.invoke()
scores = engine.get_tensor(result["index"])[0].tolist()

values = lambda items: ", ".join(map(str, items))
header = (
    "#pragma once\n#include <cstdint>\n"
    f"inline constexpr std::int8_t smoke_input[8] = {{{values(quant)}}};\n"
    f"inline constexpr std::int8_t smoke_expected[3] = {{{values(scores)}}};\n"
)
(root / "config/motion_smoke.hpp").write_text(header, encoding="utf-8")
print(f"PC_INFER true=walk scores={scores} class={int(np.argmax(scores))}")
