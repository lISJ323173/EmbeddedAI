import csv
import json
from pathlib import Path
import numpy as np
import tensorflow as tf

root = Path(__file__).resolve().parents[1]
labels = ["still", "walk", "run"]
names = [f"{axis}_{stat}" for axis in ("acc", "gyro")
         for stat in ("mean", "std", "max", "mean_delta")]
groups = {"train": ([], []), "val": ([], [])}

with (root / "motionsense_features.csv").open(encoding="utf-8-sig", newline="") as file:
    for row in csv.DictReader(file):
        if row["split"] in groups:
            features, targets = groups[row["split"]]
            features.append([float(row[name]) for name in names])
            targets.append(labels.index(row["label"]))

(x_train, y_train), (x_val, y_val) = [
    (np.asarray(x, dtype=np.float32), np.asarray(y, dtype=np.int32))
    for x, y in groups.values()
]
mean = x_train.mean(axis=0)
std = np.maximum(x_train.std(axis=0), 1e-6)
x_train = (x_train - mean) / std
x_val = (x_val - mean) / std
counts = np.bincount(y_train, minlength=3)
weights = {i: float(len(y_train) / (3 * n)) for i, n in enumerate(counts)}

tf.keras.utils.set_random_seed(42)
model = tf.keras.Sequential([
    tf.keras.Input(shape=(8,)),
    tf.keras.layers.Dense(16, activation="relu"),
    tf.keras.layers.Dense(3),
])
model.compile(optimizer="adam",
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=["accuracy"])
stop = tf.keras.callbacks.EarlyStopping(
    monitor="val_loss", patience=5, restore_best_weights=True)
model.fit(x_train, y_train, validation_data=(x_val, y_val),
          class_weight=weights, batch_size=128, epochs=40, callbacks=[stop])

pred = model.predict(x_val, verbose=0).argmax(axis=1)
matrix = tf.math.confusion_matrix(y_val, pred, num_classes=3).numpy()
print(f"MODEL params={model.count_params()} train={len(y_train)} val={len(y_val)}")
print(f"VAL accuracy={np.mean(pred == y_val):.4f} labels={labels}")
print("VAL confusion (rows=true, columns=predicted):")
print(matrix)

out = root / "artifacts"
out.mkdir(exist_ok=True)
model.save(out / "motion_float.keras")
config = dict(features=names, labels=labels, mean=mean.tolist(), std=std.tolist())
(out / "motion_preprocess.json").write_text(
    json.dumps(config, indent=2) + "\n", encoding="utf-8")
