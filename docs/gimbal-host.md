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
