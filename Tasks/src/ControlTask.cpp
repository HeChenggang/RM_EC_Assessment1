#include "ControlTask.h"
#include "Pid.h"
#include "can.h"
#include "tim.h"
#include <cmath>

extern "C" {
volatile uint32_t control_enabled = 0;
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
// Initial bench gains, not tuned on hardware. Error in rpm, output in raw current.
Pid speed_pid(28.0f, 0.8f, 0.04f, 1500.0f, -1500.0f);
// Position error in degrees, output in rpm; initial outer-loop P gain.
Pid position_pid(1.0f, 0.0f, 0.0f, 60.0f, -60.0f);
constexpr float kAmplitudeDeg = 30.0f;
float position_center_deg = 0;
bool was_enabled = false;
uint16_t last_angle = 0;
bool have_angle = false;
constexpr uint32_t kPeriodTicks = 4000; // 4 seconds at 1 kHz
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
    // First frame defines zero. After a gap, unknown turns cannot be recovered;
    // re-anchor the local coordinate instead of assuming a shortest-path motion.
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
    // CAN IRQ has higher priority. Snapshot only the shared fields atomically.
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
        control_enabled = 0; // Require explicit re-enable after feedback loss.
    }

    if (control_enabled) {
        if (!was_enabled) {
            position_center_deg = position;
            phase_tick = 0;
            position_pid.reset();
            speed_pid.reset();
        }
        position_target_deg = position_center_deg + kAmplitudeDeg * std::sin(
            6.28318530718f * static_cast<float>(phase_tick) / kPeriodTicks);
        speed_target_rpm = position_pid.calc(position_target_deg, position);
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
    was_enabled = control_enabled != 0U;

    constexpr uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    if (HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) {
        // Do not accumulate delayed current commands during a bus fault.
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
    // Current mode: CAN1 / ID1. The voltage-mode Gm6020 encoder is not used.
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
