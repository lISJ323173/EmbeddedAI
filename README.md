# framework


嵌入式基础框架
## 目录结构

| 目录 | 职责 |
|---|---|
| Core/ | 芯片相关：启动、中断、时钟、main |
| Drivers/ | ST 官方 CMSIS + HAL（只读） |
| bsp/ | 外设抽象接口（纯虚类） |
| port/ | 接口的芯片实现（唯一允许调 HAL 的地方） |
| device/ | 电机、IMU、遥控等物资驱动 |
| module/ | PID、滤波、协议等算法 |
| application/ | 业务状态机 |
| config/ | 引脚、ID、参数总表 |
| docs/ | 文档 |