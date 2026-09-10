/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
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

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE BEGIN EXPORTED_DEFINES */

/* Select the motor's trapezoidal position/velocity/acceleration mode. */
#define USB_CDC_COMMAND_FLAG_STARTUP_TRAJECTORY  0x0001U
#define USB_CDC_GAIN_TEST_FLAG_ENABLE             0x01U

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

typedef struct
{
  float target_joint_pos[6];
  float kp_nm_per_rad;
  float kd_nms_per_rad;
  uint16_t seq;
  uint16_t flags;
} usb_cdc_command_t;

typedef struct
{
  uint8_t joint_index;
  uint8_t flags;
  uint16_t reserved;
  float target_position_deg;
  float kp_nm_per_rad;
  float kd_nms_per_rad;
  float max_torque_nm;
} usb_cdc_gain_test_command_t;

typedef struct
{
  float joint_pos[6];
  float joint_vel[6];
  float base_lin_vel[3];
  float base_ang_vel[3];
  float base_quat[4];
  float cmd[3];
  uint32_t timestamp_ms;
  uint16_t status;
} usb_cdc_state_t;

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

uint8_t USB_CDC_GetLatestCommand(usb_cdc_command_t *command);
uint8_t USB_CDC_GetLatestGainTestCommand(usb_cdc_gain_test_command_t *command);
uint8_t USB_CDC_SendState(const usb_cdc_state_t *state);

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

