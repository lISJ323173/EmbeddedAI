import csv
import json
import tempfile
from pathlib import Path

import numpy as np
import tensorflow as tf

root = Path(__file__).resolve().parents[1]
out = root / "artifacts"
cfg = json.loads((out / "motion_preprocess.json").read_text(encoding="utf-8"))
labels = cfg["labels"]
groups = {"train": ([], []), "val": ([], [])}
with (root / "motionsense_features.csv").open(encoding="utf-8-sig", newline="") as file:
    for row in csv.DictReader(file):
        if row["split"] in groups:
            x, y = groups[row["split"]]
            x.append([float(row[name]) for name in cfg["features"]])
            y.append(labels.index(row["label"]))

mean, std = (np.asarray(cfg[key], dtype=np.float32) for key in ("mean", "std"))
(x_train, y_train), (x_val, y_val) = [
    ((np.asarray(x, dtype=np.float32) - mean) / std, np.asarray(y, dtype=np.int32))
    for x, y in groups.values()
]
rng = np.random.default_rng(42)
picked = np.concatenate([rng.permutation(np.flatnonzero(y_train == i))[:512]
                         for i in range(len(labels))])


def representative():
    for i in picked:
        yield [x_train[i:i + 1]]


model = tf.keras.models.load_model(out / "motion_float.keras", compile=False)
with tempfile.TemporaryDirectory() as temp:
    saved = Path(temp) / "saved_model"
    model.export(str(saved), format="tf_saved_model", verbose=False)
    converter = tf.lite.TFLiteConverter.from_saved_model(str(saved))
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = converter.inference_output_type = tf.int8
    binary = converter.convert()

engine = tf.lite.Interpreter(model_content=binary)
engine.allocate_tensors()
inp, result = engine.get_input_details()[0], engine.get_output_details()[0]
sx, zx = inp["quantization"]
sy, zy = result["quantization"]
if inp["dtype"] != np.int8 or result["dtype"] != np.int8 or min(sx, sy) <= 0:
    raise SystemExit("FAIL: expected valid int8 inputs and outputs")
rounded = np.rint(x_val / sx) + zx
quantized = np.clip(rounded, -128, 127).astype(np.int8)
scores = np.empty((len(x_val), len(labels)), dtype=np.float32)
for i, row in enumerate(quantized):
    engine.set_tensor(inp["index"], row[None, :])
    engine.invoke()
    scores[i] = (engine.get_tensor(result["index"])[0].astype(np.float32) - zy) * sy

fp = model(x_val, training=False).numpy()
p_float, p_int8 = fp.argmax(axis=1), scores.argmax(axis=1)
run = y_val == labels.index("run")
(out / "motion_int8.tflite").write_bytes(binary)
print(f"INT8 bytes={len(binary)} in={inp['quantization']} out={result['quantization']}")
print(f"VAL n={len(y_val)} float={np.mean(p_float == y_val):.4f} int8={np.mean(p_int8 == y_val):.4f} agree={np.mean(p_float == p_int8):.4f}")
print(f"RUN float={sum(p_float[run] == y_val[run])}/{sum(run)} int8={sum(p_int8[run] == y_val[run])}/{sum(run)}")
print(f"ERROR logit_mae={np.mean(np.abs(fp - scores)):.6f} input_clipped_rows={np.any(rounded != quantized, axis=1).sum()}")
