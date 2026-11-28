from pathlib import Path

import numpy as np
from tensorflow.lite.python import schema_py_generated as fb

root = Path(__file__).resolve().parents[1]
binary = (root / "artifacts/motion_int8.tflite").read_bytes()
model = fb.Model.GetRootAsModel(binary, 0)
graph = model.Subgraphs(0)
if model.SubgraphsLength() != 1 or graph.OperatorsLength() != 2:
    raise SystemExit("FAIL: expected one subgraph with two operators")

lines = ["#pragma once", "#include <cstdint>", "", "namespace motion_model {"]


def emit(ctype, name, values, fmt=str):
    items = [fmt(v) for v in values]
    lines.append(f"inline constexpr {ctype} {name}[{len(items)}] = {{")
    for start in range(0, len(items), 12):
        lines.append("    " + ", ".join(items[start:start + 12]) + ",")
    lines.append("};")


def raw(tensor, dtype):
    data = model.Buffers(tensor.Buffer()).DataAsNumpy()
    return np.frombuffer(data.tobytes(), dtype=dtype)


payload = bytearray()
for i, (inputs, outputs) in enumerate(((8, 16), (16, 3))):
    op = graph.Operators(i)
    code = model.OperatorCodes(op.OpcodeIndex()).BuiltinCode()
    if code != fb.BuiltinOperator.FULLY_CONNECTED or op.InputsLength() != 3:
        raise SystemExit(f"FAIL layer {i + 1}: operator code={code}")
    table = op.BuiltinOptions()
    options = fb.FullyConnectedOptions()
    options.Init(table.Bytes, table.Pos)
    activation = (fb.ActivationFunctionType.RELU, fb.ActivationFunctionType.NONE)[i]
    if options.FusedActivationFunction() != activation or options.WeightsFormat() != 0:
        raise SystemExit(f"FAIL layer {i + 1}: activation or weight format")
    x, w, b = (graph.Tensors(op.Inputs(j)) for j in range(3))
    y = graph.Tensors(op.Outputs(0))
    if tuple(w.ShapeAsNumpy()) != (outputs, inputs) or b.Shape(0) != outputs:
        raise SystemExit(f"FAIL layer {i + 1}: unexpected parameter shapes")
    if (x.Type(), w.Type(), b.Type(), y.Type()) != (
        fb.TensorType.INT8, fb.TensorType.INT8, fb.TensorType.INT32, fb.TensorType.INT8
    ):
        raise SystemExit(f"FAIL layer {i + 1}: unexpected tensor types")
    qw = w.Quantization()
    if np.any(qw.ZeroPointAsNumpy() != 0) or qw.QuantizedDimension() != 0:
        raise SystemExit(f"FAIL layer {i + 1}: unsupported weight quantization")
    name = f"dense{i + 1}"
    emit("std::int8_t", name + "_weights", raw(w, "i1"))
    emit("std::int32_t", name + "_bias", raw(b, "<i4"))
    payload.extend(raw(w, "i1").tobytes())
    payload.extend(raw(b, "<i4").tobytes())
    emit("float", name + "_weight_scale", np.broadcast_to(qw.ScaleAsNumpy(), (outputs,)), lambda v: f"{v:.9e}f")
    weight_scales = np.broadcast_to(qw.ScaleAsNumpy(), (outputs,))
    ratio = float(x.Quantization().Scale(0)) / float(y.Quantization().Scale(0))
    q31 = np.floor(weight_scales.astype(np.float64) * ratio * (1 << 31) + 0.5)
    bound = np.abs(raw(b, "<i4").astype(np.int64))
    bound += 255 * np.abs(raw(w, "i1").astype(np.int64)).reshape(outputs, inputs).sum(axis=1)
    if not np.all((q31 >= 1) & (q31 <= 0x7FFFFFFF)) or np.any(bound > 0x7FFFFFFF):
        raise SystemExit(f"FAIL {name}: multiplier or accumulator outside supported range")
    emit("std::int32_t", name + "_q31", q31.astype(np.int32))
    for edge, tensor in (("input", x), ("output", y)):
        q = tensor.Quantization()
        lines.append(f"inline constexpr float {name}_{edge}_scale = {q.Scale(0):.9e}f;")
        lines.append(f"inline constexpr int {name}_{edge}_zero = {q.ZeroPoint(0)};")
    print(f"DENSE{i + 1} {inputs}->{outputs} weights={inputs * outputs} bias={outputs}")

target = root / "config/motion_model_data.hpp"
target.parent.mkdir(exist_ok=True)
target.write_text("\n".join([*lines, "}", ""]), encoding="utf-8")
print(f"EXPORT OK {target}")

fingerprint = 2166136261
for byte in payload:
    fingerprint = ((fingerprint ^ byte) * 16777619) & 0xFFFFFFFF
print(f"PC_MODEL bytes={len(payload)} hash={fingerprint:08X}")
