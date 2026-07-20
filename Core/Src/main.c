/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include <string.h>
#include "motor.h"
#include "motor_control.h"
#include "motor_many.h"
#include "motor_config.h"
#include "test_motor.h"
#include "test_motor_many.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef huart1;

WWDG_HandleTypeDef hwwdg1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_WWDG1_Init(void);
/* USER CODE BEGIN PFP */

static void USB_ApplyCommandToMotors(const usb_cdc_command_t *command);
static void USB_ConfigureMotorTorqueLimits(void);
static void USB_SendRobotState(void);
static void IMU_UART_Start(void);
static void IMU_UART_Process(void);
static void IMU_SetOutputFrequency(uint8_t frequency_hz);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USB_STATE_PERIOD_MS 20U
#define USB_CONTROL_PERIOD_MS 20U
#define WWDG_REFRESH_PERIOD_MS 100U
/*
 * Match mjlab's output-side PD law:
 *   torque = (q_des - q) * Kp + (v_des - v) * Kd + torque_ff
 *
 * The motor protocol evaluates position and velocity in turns and turns/s, while
 * mjlab gains use radians and rad/s. The per-turn constants below are the matching
 * values after 2*pi conversion and M4438_30 int16 protocol quantization.
 */
#define USB_CONTROL_KP_NM_PER_RAD       25.2628551029f
#define USB_CONTROL_KD_NMS_PER_RAD      2.0076441141f
#define USB_CONTROL_KP_NM_PER_TURN      158.7312f
#define USB_CONTROL_KD_NMS_PER_TURN     12.6144f
#define USB_CONTROL_TARGET_VELOCITY     0.0f
#define USB_CONTROL_FEEDFORWARD_TORQUE  0.0f
#define USB_CONTROL_MAX_TORQUE          2.0f

#define IMU_FRAME_HEADER_1       0x7EU
#define IMU_FRAME_HEADER_2       0x23U
#define IMU_FUNC_RAW_DATA        0x04U
#define IMU_FUNC_QUATERNION      0x16U
#define IMU_FUNC_SET_FREQUENCY   0x60U
#define IMU_FRAME_MAX_LENGTH     64U
#define IMU_UART_RX_BUFFER_SIZE  512U
#define IMU_UART_RX_BUFFER_MASK  (IMU_UART_RX_BUFFER_SIZE - 1U)
#define IMU_OUTPUT_FREQUENCY_HZ  50U
#define IMU_DATA_TIMEOUT_MS      200U

#define IMU_STATUS_RAW_VALID     0x0001U
#define IMU_STATUS_QUAT_VALID    0x0002U
#define IMU_STATUS_RX_OVERFLOW   0x8000U

typedef struct
{
  float accel_g[3];
  float gyro_rad_s[3];
  float mag[3];
  float quat[4];
  uint32_t raw_timestamp_ms;
  uint32_t quat_timestamp_ms;
  uint16_t status;
} imu_data_t;

static usb_cdc_command_t g_current_command = {0};
static uint32_t g_last_state_tick = 0U;
static uint32_t g_last_control_tick = 0U;
static uint32_t g_last_wwdg_refresh_tick = 0U;
static imu_data_t g_imu_data = {0};
static uint8_t g_imu_uart_rx_byte = 0U;
static uint8_t g_imu_uart_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_imu_uart_rx_head = 0U;
static volatile uint16_t g_imu_uart_rx_tail = 0U;
static volatile uint8_t g_imu_uart_rx_overflow = 0U;

static int16_t IMU_ReadInt16LE(const uint8_t *data)
{
  uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  return (int16_t)value;
}

static float IMU_ReadFloatLE(const uint8_t *data)
{
  float value;
  uint8_t bytes[4];

  bytes[0] = data[0];
  bytes[1] = data[1];
  bytes[2] = data[2];
  bytes[3] = data[3];
  memcpy(&value, bytes, sizeof(value));
  return value;
}

static void IMU_ParseFrame(const uint8_t *frame, uint8_t length)
{
  uint8_t checksum = 0U;

  for (uint8_t i = 0U; i < (uint8_t)(length - 1U); ++i)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }

  if (checksum != frame[length - 1U])
  {
    return;
  }

  if ((frame[3] == IMU_FUNC_RAW_DATA) && (length == 0x17U))
  {
    const float accel_scale = 16.0f / 32767.0f;
    const float gyro_scale = (2000.0f / 32767.0f) * ((float)M_PI / 180.0f);
    const float mag_scale = 800.0f / 32767.0f;

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      g_imu_data.accel_g[axis] = (float)IMU_ReadInt16LE(&frame[4U + axis * 2U]) * accel_scale;
      g_imu_data.gyro_rad_s[axis] = (float)IMU_ReadInt16LE(&frame[10U + axis * 2U]) * gyro_scale;
      g_imu_data.mag[axis] = (float)IMU_ReadInt16LE(&frame[16U + axis * 2U]) * mag_scale;
    }

    g_imu_data.raw_timestamp_ms = HAL_GetTick();
    g_imu_data.status |= IMU_STATUS_RAW_VALID;
  }
  else if ((frame[3] == IMU_FUNC_QUATERNION) && (length == 0x15U))
  {
    float quat[4];
    float norm_squared = 0.0f;

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      quat[i] = IMU_ReadFloatLE(&frame[4U + i * 4U]);
      norm_squared += quat[i] * quat[i];
    }

    if (isfinite(norm_squared) && (norm_squared > 0.25f) && (norm_squared < 4.0f))
    {
      float inverse_norm = 1.0f / sqrtf(norm_squared);

      for (uint8_t i = 0U; i < 4U; ++i)
      {
        g_imu_data.quat[i] = quat[i] * inverse_norm;
      }

      g_imu_data.quat_timestamp_ms = HAL_GetTick();
      g_imu_data.status |= IMU_STATUS_QUAT_VALID;
    }
  }
}

static void IMU_UART_Process(void)
{
  static uint8_t frame[IMU_FRAME_MAX_LENGTH];
  static uint8_t frame_index = 0U;
  static uint8_t frame_length = 0U;

  if (g_imu_uart_rx_overflow != 0U)
  {
    g_imu_uart_rx_overflow = 0U;
    g_imu_data.status |= IMU_STATUS_RX_OVERFLOW;
    frame_index = 0U;
    frame_length = 0U;
  }

  while (g_imu_uart_rx_tail != g_imu_uart_rx_head)
  {
    uint8_t byte = g_imu_uart_rx_buffer[g_imu_uart_rx_tail];
    g_imu_uart_rx_tail = (uint16_t)((g_imu_uart_rx_tail + 1U) & IMU_UART_RX_BUFFER_MASK);

    if (frame_index == 0U)
    {
      if (byte == IMU_FRAME_HEADER_1)
      {
        frame[frame_index++] = byte;
      }
    }
    else if (frame_index == 1U)
    {
      if (byte == IMU_FRAME_HEADER_2)
      {
        frame[frame_index++] = byte;
      }
      else if (byte != IMU_FRAME_HEADER_1)
      {
        frame_index = 0U;
      }
    }
    else if (frame_index == 2U)
    {
      if ((byte >= 5U) && (byte <= IMU_FRAME_MAX_LENGTH))
      {
        frame_length = byte;
        frame[frame_index++] = byte;
      }
      else
      {
        frame_index = 0U;
        frame_length = 0U;
      }
    }
    else
    {
      frame[frame_index++] = byte;
      if (frame_index == frame_length)
      {
        IMU_ParseFrame(frame, frame_length);
        frame_index = 0U;
        frame_length = 0U;
      }
    }
  }
}

static void IMU_UART_Start(void)
{
  HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  if (HAL_UART_Receive_IT(&huart1, &g_imu_uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void IMU_SetOutputFrequency(uint8_t frequency_hz)
{
  uint8_t command[7] = {
    IMU_FRAME_HEADER_1,
    IMU_FRAME_HEADER_2,
    0x07U,
    IMU_FUNC_SET_FREQUENCY,
    frequency_hz,
    0x5FU,
    0x00U
  };

  for (uint8_t i = 0U; i < 6U; ++i)
  {
    command[6] = (uint8_t)(command[6] + command[i]);
  }

  (void)HAL_UART_Transmit(&huart1, command, sizeof(command), 10U);
}

static void USB_ApplyCommandToMotors(const usb_cdc_command_t *command)
{
  if (command == NULL)
  {
    return;
  }

  motor_many_pos_vel_tqe_kp_kd_2(PORT1, 1, command->target_joint_pos[0], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);
  motor_many_pos_vel_tqe_kp_kd_2(PORT1, 2, command->target_joint_pos[1], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);
  motor_many_pos_vel_tqe_kp_kd_2(PORT1, 3, command->target_joint_pos[2], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);
  motor_many_pos_vel_tqe_kp_kd_2(PORT2, 1, command->target_joint_pos[3], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);
  motor_many_pos_vel_tqe_kp_kd_2(PORT2, 2, command->target_joint_pos[4], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);
  motor_many_pos_vel_tqe_kp_kd_2(PORT2, 3, command->target_joint_pos[5], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, USB_CONTROL_KP_NM_PER_TURN, USB_CONTROL_KD_NMS_PER_TURN);

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
}

static void USB_ConfigureMotorTorqueLimits(void)
{
  /*
   * True motion-control packets do not carry register 0x025 (maximum torque).
   * Seed it through the position/velocity/maximum-torque mode before entering
   * true motion control. NAN means no position target and zero velocity prevents
   * an intentional startup motion; the same frames request the initial states.
   */
  motor_many_pos_vel_MAXtqe(PORT1, 1, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT1, 2, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT1, 3, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 1, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 2, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 3, NAN_FLOAT, 0.0f, USB_CONTROL_MAX_TORQUE);

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
}

static void USB_SendRobotState(void)
{
  usb_cdc_state_t state = {0};
  uint32_t now = HAL_GetTick();
  uint16_t imu_status = g_imu_data.status & IMU_STATUS_RX_OVERFLOW;

  motor_process_state_all();

  state.joint_pos[0] = motor_get_state(PORT1, 1)->position;
  state.joint_pos[1] = motor_get_state(PORT1, 2)->position;
  state.joint_pos[2] = motor_get_state(PORT1, 3)->position;
  state.joint_pos[3] = motor_get_state(PORT2, 1)->position;
  state.joint_pos[4] = motor_get_state(PORT2, 2)->position;
  state.joint_pos[5] = motor_get_state(PORT2, 3)->position;

  state.joint_vel[0] = motor_get_state(PORT1, 1)->velocity;
  state.joint_vel[1] = motor_get_state(PORT1, 2)->velocity;
  state.joint_vel[2] = motor_get_state(PORT1, 3)->velocity;
  state.joint_vel[3] = motor_get_state(PORT2, 1)->velocity;
  state.joint_vel[4] = motor_get_state(PORT2, 2)->velocity;
  state.joint_vel[5] = motor_get_state(PORT2, 3)->velocity;

  if (((g_imu_data.status & IMU_STATUS_QUAT_VALID) != 0U) &&
      ((now - g_imu_data.quat_timestamp_ms) <= IMU_DATA_TIMEOUT_MS))
  {
    memcpy(state.base_quat, g_imu_data.quat, sizeof(state.base_quat));
    imu_status |= IMU_STATUS_QUAT_VALID;
  }
  else
  {
    state.base_quat[0] = 1.0f;
  }

  state.base_lin_vel[0] = 0.0f;
  state.base_lin_vel[1] = 0.0f;
  state.base_lin_vel[2] = 0.0f;

  if (((g_imu_data.status & IMU_STATUS_RAW_VALID) != 0U) &&
      ((now - g_imu_data.raw_timestamp_ms) <= IMU_DATA_TIMEOUT_MS))
  {
    memcpy(state.base_ang_vel, g_imu_data.gyro_rad_s, sizeof(state.base_ang_vel));
    imu_status |= IMU_STATUS_RAW_VALID;
  }

  state.cmd[0] = 0.0f;
  state.cmd[1] = 0.0f;
  state.cmd[2] = 0.0f;

  state.timestamp_ms = now;
  state.status = imu_status;

  (void)USB_CDC_SendState(&state);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  IMU_UART_Start();

  fdcan_filter_init(&hfdcan1);
  fdcan_filter_init(&hfdcan2);

  // 打开电机电源
  HAL_GPIO_WritePin(GPIOC, MOTOR2_PWR_EN_Pin | MOTOR1_PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  IMU_SetOutputFrequency(IMU_OUTPUT_FREQUENCY_HZ);

  USB_ConfigureMotorTorqueLimits();
  HAL_Delay(2U);
  USB_SendRobotState();
  g_current_command.target_joint_pos[0] = motor_get_state(PORT1, 1)->position;
  g_current_command.target_joint_pos[1] = motor_get_state(PORT1, 2)->position;
  g_current_command.target_joint_pos[2] = motor_get_state(PORT1, 3)->position;
  g_current_command.target_joint_pos[3] = motor_get_state(PORT2, 1)->position;
  g_current_command.target_joint_pos[4] = motor_get_state(PORT2, 2)->position;
  g_current_command.target_joint_pos[5] = motor_get_state(PORT2, 3)->position;
  USB_ApplyCommandToMotors(&g_current_command);
  g_last_state_tick = HAL_GetTick();
  g_last_control_tick = g_last_state_tick;

  /* Start WWDG only after all blocking startup operations have completed. */
  MX_WWDG1_Init();
  g_last_wwdg_refresh_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    usb_cdc_command_t received_command;

    IMU_UART_Process();

    if (USB_CDC_GetLatestCommand(&received_command) != 0U)
    {
      g_current_command = received_command;
    }

    if ((HAL_GetTick() - g_last_control_tick) >= USB_CONTROL_PERIOD_MS)
    {
      USB_ApplyCommandToMotors(&g_current_command);
      g_last_control_tick = HAL_GetTick();
    }

    if ((HAL_GetTick() - g_last_state_tick) >= USB_STATE_PERIOD_MS)
    {
      USB_SendRobotState();
      g_last_state_tick = HAL_GetTick();
    }

    /*
     * Refresh only after the main-loop work has completed. With the configured
     * 120 MHz WWDG clock, prescaler 128, counter 127 and window 112, 100 ms is
     * inside the valid refresh window of approximately 65.5 ms to 275.3 ms.
     */
    if ((HAL_GetTick() - g_last_wwdg_refresh_tick) >= WWDG_REFRESH_PERIOD_MS)
    {
      (void)HAL_WWDG_Refresh(&hwwdg1);
      g_last_wwdg_refresh_tick = HAL_GetTick();
    }

    HAL_Delay(1);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 31;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 10;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 4;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 28;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = ENABLE;
  hfdcan2.Init.NominalPrescaler = 2;
  hfdcan2.Init.NominalSyncJumpWidth = 8;
  hfdcan2.Init.NominalTimeSeg1 = 31;
  hfdcan2.Init.NominalTimeSeg2 = 8;
  hfdcan2.Init.DataPrescaler = 2;
  hfdcan2.Init.DataSyncJumpWidth = 2;
  hfdcan2.Init.DataTimeSeg1 = 5;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.MessageRAMOffset = 756;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 10;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 4;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 28;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief WWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG1_Init(void)
{

  /* USER CODE BEGIN WWDG1_Init 0 */

  /* USER CODE END WWDG1_Init 0 */

  /* USER CODE BEGIN WWDG1_Init 1 */

  /* USER CODE END WWDG1_Init 1 */
  hwwdg1.Instance = WWDG1;
  hwwdg1.Init.Prescaler = WWDG_PRESCALER_128;
  hwwdg1.Init.Window = 112;
  hwwdg1.Init.Counter = 127;
  hwwdg1.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG1_Init 2 */

  /* USER CODE END WWDG1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR2_PWR_EN_Pin|MOTOR1_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR2_PWR_EN_Pin MOTOR1_PWR_EN_Pin */
  GPIO_InitStruct.Pin = MOTOR2_PWR_EN_Pin|MOTOR1_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_SW_Pin */
  GPIO_InitStruct.Pin = ESTOP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ESTOP_SW_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ESTOP_SW_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next_head = (uint16_t)((g_imu_uart_rx_head + 1U) & IMU_UART_RX_BUFFER_MASK);

    if (next_head != g_imu_uart_rx_tail)
    {
      g_imu_uart_rx_buffer[g_imu_uart_rx_head] = g_imu_uart_rx_byte;
      g_imu_uart_rx_head = next_head;
    }
    else
    {
      g_imu_uart_rx_overflow = 1U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &g_imu_uart_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart1, &g_imu_uart_rx_byte, 1U);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)// 外部中断回调函数
{
    if (GPIO_Pin == ESTOP_SW_Pin)
    {
        // // 断电
        // HAL_GPIO_WritePin(MOTOR1_PWR_EN_GPIO_Port, MOTOR1_PWR_EN_Pin | MOTOR2_PWR_EN_Pin, GPIO_PIN_RESET);
        // // 蜂鸣器响一声
        // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        // HAL_Delay(200);
        // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
