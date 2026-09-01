/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

#define USB_CDC_MAGIC_0   'R'
#define USB_CDC_MAGIC_1   'B'
#define USB_CDC_MAGIC_2   '3'
#define USB_CDC_MAGIC_3   '2'

#define USB_CDC_VERSION   1U
#define USB_CDC_MSG_STATE     0x01U
#define USB_CDC_MSG_COMMAND   0x02U
#define USB_CDC_MSG_HEARTBEAT 0x03U
#define USB_CDC_MSG_GAIN_TEST 0x04U

#define USB_CDC_HEADER_SIZE   10U
#define USB_CDC_CRC_SIZE      2U
#define USB_CDC_MAX_FRAME_SIZE APP_RX_DATA_SIZE

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

static uint8_t rx_stream[APP_RX_DATA_SIZE];
static uint16_t rx_stream_len = 0;

static volatile uint8_t latest_command_valid = 0;
static usb_cdc_command_t latest_command;
static volatile uint8_t latest_gain_test_command_valid = 0;
static usb_cdc_gain_test_command_t latest_gain_test_command;

static volatile uint8_t tx_busy = 0;
static uint8_t tx_frame[APP_TX_DATA_SIZE];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

static uint16_t USB_CDC_Crc16Ccitt(const uint8_t *data, uint16_t length);
static void USB_CDC_ResetRxStream(void);
static void USB_CDC_PushRxBytes(const uint8_t *data, uint16_t length);
static void USB_CDC_CompactRxStream(uint16_t drop_count);
static void USB_CDC_TryParseRxStream(void);
static uint8_t USB_CDC_ParseCommandFrame(const uint8_t *payload, uint16_t payload_length, uint16_t seq);
static uint8_t USB_CDC_ParseGainTestFrame(const uint8_t *payload, uint16_t payload_length);
static uint8_t USB_CDC_PackStateFrame(const usb_cdc_state_t *state, uint8_t *frame, uint16_t *frame_length);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  USB_CDC_ResetRxStream();
  latest_command_valid = 0;
  latest_gain_test_command_valid = 0;
  tx_busy = 0;
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  if ((Buf != NULL) && (Len != NULL) && (*Len > 0U))
  {
    USB_CDC_PushRxBytes(Buf, (uint16_t)*Len);
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, Buf);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  tx_busy = 0;
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

uint8_t USB_CDC_GetLatestCommand(usb_cdc_command_t *command)
{
  if (command == NULL)
  {
    return 0U;
  }

  if (latest_command_valid == 0U)
  {
    return 0U;
  }

  __disable_irq();
  *command = latest_command;
  latest_command_valid = 0U;
  __enable_irq();

  return 1U;
}

uint8_t USB_CDC_GetLatestGainTestCommand(usb_cdc_gain_test_command_t *command)
{
  if ((command == NULL) || (latest_gain_test_command_valid == 0U))
  {
    return 0U;
  }

  __disable_irq();
  *command = latest_gain_test_command;
  latest_gain_test_command_valid = 0U;
  __enable_irq();

  return 1U;
}

uint8_t USB_CDC_SendState(const usb_cdc_state_t *state)
{
  uint16_t frame_length = 0U;

  if ((state == NULL) || (tx_busy != 0U))
  {
    return USBD_BUSY;
  }

  if (USB_CDC_PackStateFrame(state, tx_frame, &frame_length) == 0U)
  {
    return USBD_FAIL;
  }

  tx_busy = 1U;
  if (CDC_Transmit_HS(tx_frame, frame_length) != USBD_OK)
  {
    tx_busy = 0U;
    return USBD_BUSY;
  }

  return USBD_OK;
}

static uint16_t USB_CDC_Crc16Ccitt(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void USB_CDC_ResetRxStream(void)
{
  rx_stream_len = 0U;
  memset(rx_stream, 0, sizeof(rx_stream));
}

static void USB_CDC_CompactRxStream(uint16_t drop_count)
{
  if (drop_count >= rx_stream_len)
  {
    rx_stream_len = 0U;
    return;
  }

  memmove(rx_stream, &rx_stream[drop_count], rx_stream_len - drop_count);
  rx_stream_len = (uint16_t)(rx_stream_len - drop_count);
}

static void USB_CDC_PushRxBytes(const uint8_t *data, uint16_t length)
{
  if (length >= APP_RX_DATA_SIZE)
  {
    data = &data[length - APP_RX_DATA_SIZE];
    length = APP_RX_DATA_SIZE;
  }

  if ((uint32_t)rx_stream_len + length > APP_RX_DATA_SIZE)
  {
    uint16_t drop = (uint16_t)(((uint32_t)rx_stream_len + length) - APP_RX_DATA_SIZE);
    USB_CDC_CompactRxStream(drop);
  }

  memcpy(&rx_stream[rx_stream_len], data, length);
  rx_stream_len = (uint16_t)(rx_stream_len + length);
  USB_CDC_TryParseRxStream();
}

static void USB_CDC_TryParseRxStream(void)
{
  while (rx_stream_len >= USB_CDC_HEADER_SIZE)
  {
    uint16_t start = 0U;
    uint8_t found = 0U;

    for (; start + 3U < rx_stream_len; ++start)
    {
      if ((rx_stream[start] == (uint8_t)USB_CDC_MAGIC_0) &&
          (rx_stream[start + 1U] == (uint8_t)USB_CDC_MAGIC_1) &&
          (rx_stream[start + 2U] == (uint8_t)USB_CDC_MAGIC_2) &&
          (rx_stream[start + 3U] == (uint8_t)USB_CDC_MAGIC_3))
      {
        found = 1U;
        break;
      }
    }

    if (found == 0U)
    {
      if (rx_stream_len > 3U)
      {
        USB_CDC_CompactRxStream((uint16_t)(rx_stream_len - 3U));
      }
      break;
    }

    if (start > 0U)
    {
      USB_CDC_CompactRxStream(start);
      continue;
    }

    if (rx_stream_len < USB_CDC_HEADER_SIZE)
    {
      break;
    }

    uint8_t version = rx_stream[4];
    uint8_t msg_id = rx_stream[5];
    uint16_t seq = (uint16_t)rx_stream[6] | ((uint16_t)rx_stream[7] << 8);
    uint16_t payload_length = (uint16_t)rx_stream[8] | ((uint16_t)rx_stream[9] << 8);
    uint32_t frame_length = (uint32_t)USB_CDC_HEADER_SIZE + (uint32_t)payload_length + (uint32_t)USB_CDC_CRC_SIZE;

    if ((version != USB_CDC_VERSION) || (payload_length > (APP_RX_DATA_SIZE - USB_CDC_HEADER_SIZE - USB_CDC_CRC_SIZE)) ||
        (frame_length > rx_stream_len))
    {
      USB_CDC_CompactRxStream(1U);
      continue;
    }

    uint16_t received_crc = (uint16_t)rx_stream[USB_CDC_HEADER_SIZE + payload_length] |
                            ((uint16_t)rx_stream[USB_CDC_HEADER_SIZE + payload_length + 1U] << 8);
    uint16_t calculated_crc = USB_CDC_Crc16Ccitt(rx_stream, (uint16_t)(USB_CDC_HEADER_SIZE + payload_length));

    if (received_crc == calculated_crc)
    {
      const uint8_t *payload = &rx_stream[USB_CDC_HEADER_SIZE];
      if (msg_id == USB_CDC_MSG_COMMAND)
      {
        (void)USB_CDC_ParseCommandFrame(payload, payload_length, seq);
      }
      else if (msg_id == USB_CDC_MSG_GAIN_TEST)
      {
        (void)USB_CDC_ParseGainTestFrame(payload, payload_length);
      }
    }

    USB_CDC_CompactRxStream((uint16_t)frame_length);
  }
}

static uint8_t USB_CDC_ParseCommandFrame(const uint8_t *payload, uint16_t payload_length, uint16_t seq)
{
  UNUSED(seq);

  if ((payload == NULL) || (payload_length != 28U))
  {
    return 0U;
  }

  usb_cdc_command_t command;
  memcpy(command.target_joint_pos, payload, sizeof(command.target_joint_pos));
  command.seq = (uint16_t)payload[24] | ((uint16_t)payload[25] << 8);
  command.flags = (uint16_t)payload[26] | ((uint16_t)payload[27] << 8);

  __disable_irq();
  latest_command = command;
  latest_command_valid = 1U;
  __enable_irq();

  return 1U;
}

static uint8_t USB_CDC_ParseGainTestFrame(const uint8_t *payload, uint16_t payload_length)
{
  usb_cdc_gain_test_command_t command;

  if ((payload == NULL) || (payload_length != 20U))
  {
    return 0U;
  }

  command.joint_index = payload[0];
  command.flags = payload[1];
  command.reserved = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  memcpy(&command.target_position_deg, &payload[4], sizeof(float));
  memcpy(&command.kp_nm_per_rad, &payload[8], sizeof(float));
  memcpy(&command.kd_nms_per_rad, &payload[12], sizeof(float));
  memcpy(&command.max_torque_nm, &payload[16], sizeof(float));

  __disable_irq();
  latest_gain_test_command = command;
  latest_gain_test_command_valid = 1U;
  __enable_irq();

  return 1U;
}

static uint8_t USB_CDC_PackStateFrame(const usb_cdc_state_t *state, uint8_t *frame, uint16_t *frame_length)
{
  const uint16_t payload_length = 106U;
  const uint16_t total_length = (uint16_t)(USB_CDC_HEADER_SIZE + payload_length + USB_CDC_CRC_SIZE);
  uint16_t offset = 0U;
  uint16_t crc = 0U;

  if ((state == NULL) || (frame == NULL) || (frame_length == NULL) || (total_length > APP_TX_DATA_SIZE))
  {
    return 0U;
  }

  frame[0] = (uint8_t)USB_CDC_MAGIC_0;
  frame[1] = (uint8_t)USB_CDC_MAGIC_1;
  frame[2] = (uint8_t)USB_CDC_MAGIC_2;
  frame[3] = (uint8_t)USB_CDC_MAGIC_3;
  frame[4] = USB_CDC_VERSION;
  frame[5] = USB_CDC_MSG_STATE;
  frame[6] = 0U;
  frame[7] = 0U;
  frame[8] = (uint8_t)(payload_length & 0xFFU);
  frame[9] = (uint8_t)((payload_length >> 8) & 0xFFU);
  offset = USB_CDC_HEADER_SIZE;

#define USB_CDC_COPY_BYTES(src_ptr, byte_count) \
  do { \
    memcpy(&frame[offset], (src_ptr), (byte_count)); \
    offset = (uint16_t)(offset + (byte_count)); \
  } while (0)

  USB_CDC_COPY_BYTES(state->joint_pos, sizeof(state->joint_pos));
  USB_CDC_COPY_BYTES(state->joint_vel, sizeof(state->joint_vel));
  USB_CDC_COPY_BYTES(state->base_lin_vel, sizeof(state->base_lin_vel));
  USB_CDC_COPY_BYTES(state->base_ang_vel, sizeof(state->base_ang_vel));
  USB_CDC_COPY_BYTES(state->base_quat, sizeof(state->base_quat));
  USB_CDC_COPY_BYTES(state->cmd, sizeof(state->cmd));
  USB_CDC_COPY_BYTES(&state->timestamp_ms, sizeof(state->timestamp_ms));
  USB_CDC_COPY_BYTES(&state->status, sizeof(state->status));

#undef USB_CDC_COPY_BYTES

  crc = USB_CDC_Crc16Ccitt(frame, (uint16_t)(USB_CDC_HEADER_SIZE + payload_length));
  frame[offset] = (uint8_t)(crc & 0xFFU);
  frame[offset + 1U] = (uint8_t)((crc >> 8) & 0xFFU);
  *frame_length = total_length;

  return 1U;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
