/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* Call once after both MX_CANx_Init functions. */
void CAN_Start(void);
/* Nonblocking enqueue; HAL_OK means queued, HAL_BUSY means all mailboxes full.
 * Serialize callers on each bus (do not call concurrently from main and ISR). */
HAL_StatusTypeDef CAN_SendStd(CAN_HandleTypeDef *hcan, uint16_t id,
                              const uint8_t *data, uint8_t length);
/* Runs in RX interrupt context. Do not block; copy data before returning.
 * The default weak implementation discards frames. Override with C linkage. */
void CAN_ReceiveCallback(CAN_HandleTypeDef *hcan,
                         const CAN_RxHeaderTypeDef *header,
                         const uint8_t *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

