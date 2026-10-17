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

/* 两个 MX_CANx_Init 函数均执行完毕后调用一次。 */
void CAN_Start(void);
/* 非阻塞入队；HAL_OK 表示已入队，HAL_BUSY 表示所有发送邮箱均已满。
 * 同一总线上的调用须串行执行（不要在主循环与中断服务程序中并发调用）。 */
HAL_StatusTypeDef CAN_SendStd(CAN_HandleTypeDef *hcan, uint16_t id,
                              const uint8_t *data, uint8_t length);
/* 在接收中断上下文中执行。不得阻塞，需在返回前完成数据复制。
 * 默认弱实现会丢弃帧；重写时须使用 C 链接方式。 */
void CAN_ReceiveCallback(CAN_HandleTypeDef *hcan,
                         const CAN_RxHeaderTypeDef *header,
                         const uint8_t *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

