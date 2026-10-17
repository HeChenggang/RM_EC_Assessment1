#include "ControlTask.h"
#include "Pid.h"
#include "can.h"
#include "tim.h"
#include <cmath>

extern "C" {
volatile uint32_t control_enabled = 0;
volatile uint32_t tuning_limit_ms = 0; // 0：持续运行，直到禁用控制或发生故障。
volatile uint32_t tuning_samples = 0;
volatile float tuning_error_square_sum = 0;
volatile float tuning_peak_error_deg = 0;
// 在线调参：在 control_enabled=0 时修改参数，然后重新使能以复位积分项。
volatile float position_kp = 10.0f;
volatile float position_ki = 0.0f;
volatile float position_kd = 0.0f;
volatile float velocity_feedforward_gain = 2.0f;
volatile float position_target_deg = 0;
volatile float position_feedback_deg = 0;
volatile float position_error_deg = 0;
volatile float speed_target_rpm = 0;
volatile float speed_feedback_rpm = 0;
volatile float speed_error_rpm = 0;
volatile float current_command_raw = 0;
volatile uint32_t feedback_count = 0;
volatile uint32_t feedback_age_ms = 0;
volatile uint32_t feedback_valid = 0;
volatile uint32_t control_tx_failures = 0;
}

namespace {
// 速度误差单位为 rpm，输出为电流指令原始值。
Pid speed_pid(28.0f, 0.8f, 0.04f, 1500.0f, -1500.0f);
// 位置误差单位为度，输出单位为 rpm；采用位置外环的初始比例增益。
Pid position_pid(10.0f, 0.0f, 0.0f, 60.0f, -60.0f);
constexpr float kAmplitudeDeg = 60.0f;
float position_center_deg = 0;
float active_feedforward_gain = 0;
bool was_enabled = false;
uint32_t enabled_ticks = 0;
uint16_t last_angle = 0;
bool have_angle = false;
constexpr uint32_t kPeriodTicks = 4000; // 控制频率为 1 kHz 时对应 4 秒
constexpr uint32_t kFeedbackTimeoutMs = 20;
volatile uint32_t last_feedback_tick = 0;
uint32_t phase_tick = 0;
}

extern "C" void CAN_ReceiveCallback(CAN_HandleTypeDef *hcan,
                                    const CAN_RxHeaderTypeDef *header,
                                    const uint8_t *data)
{
    if (hcan != &hcan1 || header->StdId != 0x205U || header->DLC != 8U ||
        header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA) {
        return;
    }
    const uint16_t raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    const uint16_t angle = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (angle >= 8192U) {
        return;
    }
    const uint32_t now = HAL_GetTick();
    // 首帧定义零点。反馈中断后无法恢复期间未知的转动圈数；
    // 因此重新设置局部坐标原点，而不假定电机沿最短路径运动。
    if (!have_angle || now - last_feedback_tick > kFeedbackTimeoutMs) {
        position_feedback_deg = 0;
        control_enabled = 0;
        have_angle = true;
    } else {
        int32_t delta = static_cast<int32_t>(angle) - last_angle;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        position_feedback_deg = position_feedback_deg + delta * (360.0f / 8192.0f);
    }
    last_angle = angle;
    const int32_t rpm = raw < 0x8000U ? raw : static_cast<int32_t>(raw) - 65536;
    speed_feedback_rpm = static_cast<float>(rpm);
    last_feedback_tick = HAL_GetTick();
    feedback_count = feedback_count + 1U;
}

extern "C" void ControlTaskInit(void)
{
    control_enabled = 0;
    speed_pid.reset();
    CAN_Start();
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

extern "C" void MainTask(void)
{
    // CAN 中断优先级更高，仅对共享字段进行原子快照读取。
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const float rpm = speed_feedback_rpm;
    const float position = position_feedback_deg;
    const uint32_t last_tick = last_feedback_tick;
    const uint32_t count = feedback_count;
    __set_PRIMASK(mask);
    feedback_age_ms = HAL_GetTick() - last_tick;
    feedback_valid = count != 0U && feedback_age_ms <= kFeedbackTimeoutMs;
    if (!feedback_valid || (CAN1->ESR & CAN_ESR_BOFF) != 0U) {
        control_enabled = 0; // 反馈丢失后需要显式重新使能。
    }

    if (control_enabled) {
        if (!was_enabled) {
            enabled_ticks = 0;
            tuning_samples = 0;
            tuning_error_square_sum = 0;
            tuning_peak_error_deg = 0;
            // 拒绝调试器输入的无效参数，仅在使能瞬间应用新的增益。
            const float kp = position_kp, ki = position_ki, kd = position_kd;
            const float ff = velocity_feedforward_gain;
            if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
                kp < 0.0f || kp > 10.0f || ki < 0.0f || ki > 10.0f ||
                kd < 0.0f || kd > 1.0f || !std::isfinite(ff) || ff < 0 || ff > 2) {
                control_enabled = 0;
                current_command_raw = 0;
                HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
                return;
            }
            position_pid.setParams(kp, ki, kd);
            active_feedforward_gain = ff;
            position_center_deg = position;
            phase_tick = 0;
            position_pid.reset();
            speed_pid.reset();
        }
        position_target_deg = position_center_deg + kAmplitudeDeg * std::sin(
            6.28318530718f * static_cast<float>(phase_tick) / kPeriodTicks);
        speed_target_rpm = position_pid.calc(position_target_deg, position);
        // 对轨迹进行解析求导：角速度（度/秒）除以 6 即为转速（rpm）。
        const float omega = 6.28318530718f / (kPeriodTicks * 0.001f);
        speed_target_rpm = speed_target_rpm + active_feedforward_gain *
            kAmplitudeDeg * omega / 6.0f * std::cos(
                6.28318530718f * static_cast<float>(phase_tick) / kPeriodTicks);
        if (speed_target_rpm > 60.0f) speed_target_rpm = 60.0f;
        if (speed_target_rpm < -60.0f) speed_target_rpm = -60.0f;
        phase_tick = (phase_tick + 1U) % kPeriodTicks;
        current_command_raw = speed_pid.calc(speed_target_rpm, rpm);
    } else {
        phase_tick = 0;
        speed_pid.reset();
        position_pid.reset();
        position_target_deg = position;
        speed_target_rpm = 0;
        current_command_raw = 0;
    }
    speed_error_rpm = speed_target_rpm - rpm;
    position_error_deg = position_target_deg - position;
    if (control_enabled) {
        if (enabled_ticks != UINT32_MAX) ++enabled_ticks;
        // 在第二个 4 秒周期内统计误差，用于比较稳态跟踪效果。
        if (enabled_ticks > 4000U && enabled_ticks <= 8000U) {
            tuning_samples = tuning_samples + 1U;
            tuning_error_square_sum = tuning_error_square_sum + position_error_deg * position_error_deg;
            const float error = std::fabs(position_error_deg);
            if (error > tuning_peak_error_deg) tuning_peak_error_deg = error;
        }
        if (tuning_limit_ms != 0U && enabled_ticks >= tuning_limit_ms) {
            control_enabled = 0;
            current_command_raw = 0;
        }
    }
    was_enabled = control_enabled != 0U;

    constexpr uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    if (HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) {
        // 总线故障时避免积压延迟的电流指令。
        HAL_CAN_AbortTxRequest(&hcan1, mailboxes);
        control_tx_failures = control_tx_failures + 1U;
        control_enabled = 0;
        was_enabled = false;
        current_command_raw = 0;
        speed_pid.reset();
        return;
    }
    const uint16_t raw = static_cast<uint16_t>(static_cast<int16_t>(current_command_raw));
    const uint8_t data[8] = {static_cast<uint8_t>(raw >> 8),
                             static_cast<uint8_t>(raw), 0, 0, 0, 0, 0, 0};
    // 电流模式：使用 CAN1 上 ID 为 1 的电机，不使用 Gm6020 的电压模式编码器。
    if (CAN_SendStd(&hcan1, 0x1FEU, data, 8U) != HAL_OK) {
        control_tx_failures = control_tx_failures + 1U;
        control_enabled = 0;
        was_enabled = false;
        current_command_raw = 0;
        speed_pid.reset();
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        MainTask();
    }
}
