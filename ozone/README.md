# GM6020 速度曲线观测

已在 Ozone V3.50 中下载新固件并运行，验证实际速度采样和 Timeline 显示。
当前默认控制禁用，反馈有效；尚未开启正弦闭环或进行 PID 实机整定。

## 后续在 Ozone 中使用

1. 使用 `cmake --preset Debug` 和 `cmake --build --preset Debug` 构建。
2. 打开本目录的 `GM6020.jdebug`。工程读取 `../build/project.elf` 的调试符号。
3. 准备实际调试时，再下载此 ELF 并运行。不能用新 ELF 的地址观察旧固件。
4. Data Sampling 已预设 `speed_feedback_rpm`、`speed_target_rpm`、
   `current_command_raw`、`feedback_valid`。在 Sampling Frequency 中选择 1 kHz
   （若当前探针不支持，选可用频率）。在 Timeline 中观察对应曲线。
5. 新固件默认 `control_enabled=0`，只接收反馈并发送零电流。
   首先确认 `feedback_count` 持续增长且 `feedback_valid=1`。
6. 后续允许电机运动时，在 Watched Data 中将 `control_enabled` 改为 `1`。
   改回 `0` 可解除驱动电流；这不是主动制动。

固件不依赖 printf、RTT 或 SWO；Ozone 通过 J-Link 对 RAM 全局变量采样。
若版本没有自动显示曲线，可在 Data Sampling 的 Add Expression 中手动加入上述变量。
每个变量均为 C 链接的全局 volatile 标量，不使用旧固件的固定地址。

## 控制配置

- CAN1，GM6020 ID 1；反馈 `0x205`，电流指令 `0x1FE`。
- TIM6 中断运行 1 kHz 速度环，目标为 `100*sin(2*pi*t/4)` rpm。
- 目标从零相位开始，禁用时目标归零；重新使能重新开始。
- 初始 PI 参数：Kp=4、Ki=8、Kd=0。误差单位 rpm，输出为原始电流指令；
  Ki 按秒积分，采样周期 0.001 s。参数在 `Tasks/src/ControlTask.cpp` 中。
- 电流限幅 ±1500，约 ±0.275 A。尚未实机整定或验证跟踪性能。
- 反馈超过 20 ms 未更新、bus-off 或发送拥塞时禁用控制，并清除积分；
  故障恢复后需要手动重新使能。反馈变量在失联时保持最后值，必须同时查看
  `feedback_valid` 或 `feedback_age_ms`，不能将保持值误认为新数据。
- 电流帧中 ID 2~4 的指令为零；当前仅针对单电机台架。

## 观测变量

| 变量 | 含义 |
| --- | --- |
| speed_feedback_rpm | CAN 反馈有符号转速，rpm |
| speed_target_rpm | 正弦目标转速，rpm |
| speed_error_rpm | 目标减反馈，rpm |
| current_command_raw | 原始电流指令；16384 对应 3 A |
| feedback_count | 有效 ID 1 反馈帧累计数 |
| feedback_age_ms | 距离最后一帧的毫秒数 |
| feedback_valid | 反馈是否新鲜有效 |
| control_enabled | 0 禁用、1 使能；复位默认 0 |
| control_tx_failures | 发送拥塞或入队失败次数 |

Ozone 配置依据：[SEGGER Ozone 用户手册](https://www.segger.com/downloads/jlink/UM08025_Ozone.pdf)
中的 Data Sampling、Timeline 和 Window.Add 章节。工程脚本尚未在 Ozone 界面中实测。
