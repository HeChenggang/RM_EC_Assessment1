#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

// 供 C 中断处理(stm32f4xx_it.c 及 HAL 弱回调)调用的接口
#ifdef __cplusplus
extern "C" {
#endif

// 上电初始化: 配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器
// 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void);

// 1kHz 控制入口, 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void);

/* Ozone symbols: rpm, raw current command (16384 = 3 A), milliseconds.
 * Default disabled: set control_enabled=1 only when ready to run the motor. */
extern volatile uint32_t control_enabled;
extern volatile float position_target_deg;
extern volatile float position_feedback_deg;
extern volatile float position_error_deg;
extern volatile float speed_target_rpm;
extern volatile float speed_feedback_rpm;
extern volatile float speed_error_rpm;
extern volatile float current_command_raw;
extern volatile uint32_t feedback_count;
extern volatile uint32_t feedback_age_ms;
extern volatile uint32_t feedback_valid;

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H
