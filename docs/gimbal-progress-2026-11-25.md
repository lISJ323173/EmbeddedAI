# 开发记录：2026-11-25

## 双轴遥测验收通过
- 采样任务在两轴更新后生成快照，经报告队列发送。
- P/Y 字段顺序：
  state, enabled, ready, feedback_mdeg, pulse_us, last_fault。
- USART2：9600 8N1，约 1 Hz；缓冲区 384 字节，发送超时 500 ms。
- 运行中双轴 state/enabled/ready 均为 1/1/1。
- X 后双轴 state/enabled 均为 0/0。
- P 水平零点保留，Y 航向参考清除。
- 输出完整，report_drop_count=0，蓝牙 errors/busy=0/0。
- 两条记录 DT=10..10、MISS=0；累计 BAD=12，期间未增加。
- X 后 392 ms 观察到停止遥测，不代表精确停止延迟。
