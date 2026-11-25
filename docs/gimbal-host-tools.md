# 上位机工具使用说明（2026-11-25）

只读上位机：实时三轴曲线 + CSV 记录 + 离线回放。不发送任何运动命令。

## 环境

- Python 3.14.7，虚拟环境在工程根目录 `.venv-gimbal`，无需激活，直接调用
  `.venv-gimbal/Scripts/python.exe`。
- 直接依赖见 `tools/gimbal_requirements.txt`（pyserial、matplotlib）。
- 锁定版本见 `tools/gimbal_requirements.freeze.txt`（12 个包，全部位于虚拟环境内）。
- 记录目录 `logs/`、虚拟环境 `.venv-gimbal/`、`__pycache__/` 均已在 `.gitignore` 中排除。

### 本机环境修复（仅本机，不入库）

基座 Python 装在盘根 `E:\`，其 `E:\Lib\sitecustomize.py` 会无条件把
`E:\Lib\site-packages` 追加进 `sys.path`，导致全局包泄漏进虚拟环境
（`pip freeze` 混入 `pypdf`、`qrcode` 等全局包，`Pillow` 也解析到全局副本）。

已将该文件改为仅在 `sys.prefix == sys.base_prefix`（即基础 Python）时追加，
虚拟环境不再受影响；原文件备份为 `E:\Lib\sitecustomize.py.before-gimbal.bak`。
基础 Python 的原有行为保持不变。

## 工具

| 文件 | 作用 |
|---|---|
| `tools/gimbal_protocol.py` | 遥测行解析，`parse_line(raw)` 返回字段字典 |
| `tools/gimbal_record.py` | `CsvRecorder`，以 `"x"` 模式新建 CSV（同名拒绝覆盖） |
| `tools/gimbal_plot.py` | 实时三轴曲线，可选 `--record` 同步记录 |
| `tools/gimbal_replay.py` | 离线回放整段记录，不占用串口 |

## 命令

```bash
# 只看实时曲线
.venv-gimbal/Scripts/python.exe tools/gimbal_plot.py --port COM10 --baud 9600

# 实时曲线 + 记录（文件名必须不存在）
.venv-gimbal/Scripts/python.exe tools/gimbal_plot.py --port COM10 --baud 9600 \
    --record logs/gimbal-20261125-01.csv

# 离线回放
.venv-gimbal/Scripts/python.exe tools/gimbal_replay.py logs/gimbal-20261125-01.csv
```

串口参数固定 9600 8N1，数据来自 HC-05 蓝牙透传（COM10）。关闭窗口会释放串口。

## 单位约定

| 数据位置 | 单位 |
|---|---|
| 固件 `RPY=`、`raw_line` 中的 RPY | mdeg |
| `parse_line()` 返回的 `frame["rpy"]` | 度 |
| 实时曲线、CSV 的 `*_deg`、离线曲线 | 度 |
| P/Y 的 `feedback_mdeg` | mdeg，另受 `ready` 约束 |

`parse_line()` 已完成 mdeg → 度的换算，记录模块直接写入 `frame["rpy"]`，
不得再次除以 1000。

## CSV 字段

共 21 列：

```
pc_time_utc, pc_elapsed_s, n, cal, att,
roll_deg, pitch_deg, yaw_deg,
p_state, p_enabled, p_ready, p_feedback_mdeg, p_pulse_us, p_last_fault,
y_state, y_enabled, y_ready, y_feedback_mdeg, y_pulse_us, y_last_fault,
raw_line
```

- `pc_time_utc`：电脑接收时间，UTC，毫秒精度。
- `pc_elapsed_s`：相对程序启动的接收时刻。
- `raw_line`：原始报告全文，便于逐字段复核。
- 每个通过校验的报告都记录一行，包括 `n` 不变但状态变化的报告。

## 缺口与断线

- 实时与离线都以 3 s 为阈值：相邻样本接收时间差超过 3 s 时插入 `NaN`，曲线断开，
  不再用直线跨过缺口。该阈值仅用于显示，不代表控制超时。
- `n` 下降（MCU 复位）同样断线：实时清空历史，离线插入 `NaN`。
- 遥测停发期间不产生任何数据行。
- 一次性补入多行时（蓝牙链路恢复后集中到达），这些行的 `pc_elapsed_s` 相同，
  时间列会重叠，需用 `n` 区分。

## 已知限制

- 遥测约 1 Hz，只能验证接收、解析与绘图，不能用于亚秒级响应时间测量。
- `ready=0` 时 CSV 中的 `*_feedback_mdeg` 可能保留停止前的旧值，
  分析时必须结合 `p_ready` / `y_ready` 判断有效性。
- 姿态 `ATT=2` 是粘滞失效（如调试器暂停导致单次采样周期超限），
  需发送 `B` 重新初始化，不属上位机问题。
