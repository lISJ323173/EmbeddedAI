# 云台上位机

## 环境与连接
- Windows，项目虚拟环境 .venv-gimbal。
- HC-05 蓝牙串口：当前 COM10，9600 8N1，无流控。
- 打开程序前关闭其他占用该串口的工具。
- requirements.freeze.txt 保存本次验收环境的依赖版本。

## 安装
在工程根目录执行：
    py -m venv .venv-gimbal
    .venv-gimbal/Scripts/python.exe -m pip install -r tools/gimbal_requirements.freeze.txt

## 实时显示
    .venv-gimbal/Scripts/python.exe tools/gimbal_plot.py --port COM10 --baud 9600

## 同时记录 CSV
    .venv-gimbal/Scripts/python.exe tools/gimbal_plot.py --port COM10 --baud 9600 --record logs/session-01.csv

## 离线绘图
    .venv-gimbal/Scripts/python.exe tools/gimbal_replay.py logs/session-01.csv

## 数据约定
- 固件 RPY 和 raw_line 中的 RPY：mdeg。
- 解析器返回的 rpy、曲线、CSV 的 *_deg：度。
- P/Y feedback_mdeg：mdeg，必须结合 ready 判断有效性。
- CSV 使用电脑接收时间，当前遥测约 1 Hz。
- 不能据此精确测量亚秒级控制响应。
- 超过 3 秒的接收缺口显示为断线。
- 同名 CSV 拒绝覆盖；停发期间不生成伪造数据行。

## 运行行为
- 程序只读，不发送运动命令。
- 关闭窗口释放串口，不会向云台发送 X。
- 失联显示 NO DATA 或 DISCONNECTED。
- 环境目录和 logs/ 默认不提交 Git。

## 已验证
- 三轴实时显示、状态显示、断开提示和串口释放。
- CSV 30 行，三轴单位逐行与原始报告核对通过。
- 实时与离线缺口断线，恢复后继续绘图。
- 当前环境三个主要依赖均从虚拟环境加载。

## 滤波参数调整
- Gain 0.5 / 2 / 5 对应命令字符 1 / 2 / 3。
- 参数为 correction_gain，不是 0～1 的混合比例。
- 蓝牙任务入队，采样任务校验并应用。
- K 为实际参数乘 1000；KS 为最近应用状态和成功应用计数。
- 上位机同时核对实际 K、成功状态和计数增量。
- 每次只允许一个待确认请求；超时不自动重发。
- 两轴停止、CAL/ATT=1/1、遥测有效时按钮可用。
- 复位后等待新遥测和校准完成。
- B 重新校准保留参数，复位恢复默认 2.0。

## 参数调节验收
- 三个按钮均生效，每次点击应用计数仅增加 1。
- 等待回读、失联或未校准时按钮禁用。
- CSV 扩展为 24 列。
- 73 行参数字段与同行原始 K/KS 比对，0 处不一致。
- B 前后 K=500、应用计数=7，均保持不变。
- 测试结束恢复 K=2000。
- ignored=0。
- 滤波效果比较及闭环运行中调参尚未验收。
