import csv
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

root = Path(__file__).resolve().parents[1]
out = root / "artifacts"
cfg = json.loads((out / "motion_preprocess.json").read_text(encoding="utf-8"))
with (root / "motionsense_features.csv").open(encoding="utf-8-sig", newline="") as file:
    rows = [row for row in csv.DictReader(file) if row["split"] == "test"]

labels = cfg["labels"]
y = np.array([labels.index(row["label"]) for row in rows], dtype=np.int32)
if np.bincount(y, minlength=3).tolist() != [2006, 1123, 448]:
    raise SystemExit("FAIL: unexpected test class counts")
x = np.array([[float(row[k]) for k in cfg["features"]] for row in rows], dtype=np.float32)
mean, std = (np.asarray(cfg[k], dtype=np.float32) for k in ("mean", "std"))
x = (x - mean) / std

engine = tf.lite.Interpreter(model_path=str(out / "motion_int8.tflite"))
scale, zero = engine.get_input_details()[0]["quantization"]
inputs = np.clip(np.rint(x / scale) + zero, -128, 127).astype(np.int8)
model = tf.keras.models.load_model(out / "motion_float.keras", compile=False)
scores = model(x, training=False).numpy()
if not np.isfinite(scores).all():
    raise SystemExit("FAIL: non-finite PC scores")

target = root / "config/motion_replay.hpp"
with target.open("w", encoding="utf-8") as file:
    file.write("#pragma once\n#include <cstdint>\n")
    file.write("struct ReplayCase { std::int8_t input[8]; std::uint8_t label; float pc[3]; };\n")
    file.write("inline constexpr ReplayCase motion_replay[] = {\n")
    for inp, label, fp in zip(inputs, y, scores):
        qi = ", ".join(map(str, inp))
        sf = ", ".join(f"{v:.9e}f" for v in fp)
        file.write(f"    {{{{{qi}}}, {label}, {{{sf}}}}},\n")
    file.write("};\n")

correct = int(np.count_nonzero(scores.argmax(axis=1) == y))
print(f"PC_TEST n={len(y)} float_correct={correct} float_acc={correct / len(y):.4%}")
