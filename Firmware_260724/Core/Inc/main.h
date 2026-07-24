/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//volatile int drdy_interrupt = 0;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// IR Camera Protocol Defines

// IMU Sensor Defines
#define BYTE_HEADER_CMD    0x24
#define BYTE_FOOTER_1      0x0D
#define BYTE_FOOTER_2      0x0A
#define CMD_TYPE_READ_REG  0x11
#define CMD_TYPE_GET_VERSION 0x20
#define CMD_TYPE_START_STREAMING 0x27
#define CMD_TYPE_STOP_STREAMING 0x28
#define CMD_TYPE_STREAM_DATA 0xAB
#define SIZE_CMD_READ_REGS 11
#define SIZE_CMD_COMMON    8
#define SIZE_RESP_READ_REGS_BASE 16
#define SIZE_RESP_GET_VERSION 20
#define SIZE_STREAM_PACKET 45
#define SIZE_STREAM_INIT_RESPONSE 46
#define IIM46234_WHO_AM_I  0xEA

/* IIM-4623x SPI timing model from datasheet 5.3.1.4 and ATMEL reference.
 * NSS setup/hold are local MCU chip-select guard times.
 * Twait/Tnext are protocol timings between transactions. */
#define IMU_NSS_SETUP_US          1u
#define IMU_NSS_HOLD_US           1u
#define IMU_NSS_HIGH_US           7u
#define IMU_CMD_TWAIT_US_NORMAL  50u
#define IMU_CMD_TWAIT_US_START  600u
#define IMU_CMD_TNEXT_US        300u
#define IMU_CMD_RX_SLACK          6u
#define IMU_RX_TOTAL             34u
#define IMU_SIZE_RESP_ACK        10u

int IMU_SPI_WriteSession(const uint8_t *tx, uint16_t len, uint32_t timeout_ms);
int IMU_SPI_ReadSession(uint8_t *rx, uint16_t len, uint32_t timeout_ms);
void IMU_SPI_ResetSession(void);


#define PARAM_ID_IR_NUC_COMMAND        0x0101
#define PARAM_ID_IR_NUC_INTERVAL       0x0102
#define PARAM_ID_IR_NUC_TRIGGER        0x0103
#define PARAM_ID_IR_IMAGE_FLIP         0x0202
#define PARAM_ID_IR_FRAME_RATE         0x0303
#define PARAM_ID_IR_ICE_GAIN_BIAS      0x0405
#define PARAM_ID_IR_ICE_LEVEL_BIAS     0x0607
#define PARAM_ID_IR_ICE_STRENGTH       0x0808
#define PARAM_ID_IR_PALETTE            0x0909
#define PARAM_ID_IR_POLARITY           0x1010
#define PARAM_ID_IR_D_ZOOM             0x1111
#define PARAM_ID_IR_TNR_ENABLE         0x1213
#define PARAM_ID_IR_TNR_THRESHOLD      0x5A5A

#define PARAM_ID_EO_POWER_RESET        0x1515
#define PARAM_ID_EO_D_ZOOM             0x1616
#define PARAM_ID_EO_WHITE_BALANCE      0x1717
#define PARAM_ID_EO_RED_GAIN           0x1818
#define PARAM_ID_EO_BLUE_GAIN          0x1919
#define PARAM_ID_EO_CHROMA_GAIN        0x2020
#define PARAM_ID_EO_AGC                0x2121
#define PARAM_ID_EO_AE_SHUTTER         0x2222
#define PARAM_ID_EO_BRIGHTNESS         0x2323
#define PARAM_ID_EO_BLC                0x2424
#define PARAM_ID_EO_HLC                0x2525
#define PARAM_ID_EO_ACE                0x2626
#define PARAM_ID_EO_DEFOG              0x2727
#define PARAM_ID_EO_DNR                0x2828
#define PARAM_ID_EO_GAMMA              0x2929
#define PARAM_ID_EO_SHARPNESS          0x3030
#define PARAM_ID_EO_MIRROR             0x3131
#define PARAM_ID_EO_IMAGE_FLIP         0x3232
#define PARAM_ID_EO_TRIGGER_MODE       0x3333
#define PARAM_ID_EO_TRIG_IN_POLARITY   0x3434
#define PARAM_ID_EO_STROBE_POLARITY    0x3535
#define PARAM_ID_EO_STROBE_MODE        0x3636
#define PARAM_ID_EO_EXT_TRIG_DELAY     0x3737
#define PARAM_ID_EO_STROBE_DELAY       0x3838
#define PARAM_ID_EO_STROBE_WIDTH       0x3939
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
