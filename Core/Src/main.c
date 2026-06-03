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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static void USB_ApplyCommandToMotors(const usb_cdc_command_t *command);
static void USB_SendRobotState(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USB_STATE_PERIOD_MS 20U
#define USB_CONTROL_PERIOD_MS 20U
#define USB_CONTROL_VELOCITY 20.0f
#define USB_CONTROL_TORQUE   1.0f

static usb_cdc_command_t g_current_command = {0};
static uint32_t g_last_state_tick = 0U;
static uint32_t g_last_control_tick = 0U;

static void USB_ApplyCommandToMotors(const usb_cdc_command_t *command)
{
  if (command == NULL)
  {
    return;
  }

  motor_many_pos_vel_MAXtqe(PORT1, 1, command->target_joint_pos[0], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT1, 2, command->target_joint_pos[1], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT1, 3, command->target_joint_pos[2], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 1, command->target_joint_pos[3], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 2, command->target_joint_pos[4], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);
  motor_many_pos_vel_MAXtqe(PORT2, 3, command->target_joint_pos[5], USB_CONTROL_VELOCITY, USB_CONTROL_TORQUE);

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
}

static void USB_SendRobotState(void)
{
  usb_cdc_state_t state = {0};

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

  state.base_quat[0] = 1.0f;
  state.base_quat[1] = 0.0f;
  state.base_quat[2] = 0.0f;
  state.base_quat[3] = 0.0f;

  state.base_lin_vel[0] = 0.0f;
  state.base_lin_vel[1] = 0.0f;
  state.base_lin_vel[2] = 0.0f;

  state.base_ang_vel[0] = 0.0f;
  state.base_ang_vel[1] = 0.0f;
  state.base_ang_vel[2] = 0.0f;

  state.cmd[0] = 0.0f;
  state.cmd[1] = 0.0f;
  state.cmd[2] = 0.0f;

  state.timestamp_ms = HAL_GetTick();
  state.status = 0U;

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
  fdcan_filter_init(&hfdcan1);
  fdcan_filter_init(&hfdcan2);

  // 启动 FDCAN
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);

  // 打开电机电源
  HAL_GPIO_WritePin(GPIOC, MOTOR2_PWR_EN_Pin | MOTOR1_PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // //所有电机置零
  //   motor_many_pos_vel_MAXtqe(PORT1, 1, 0.0, 20.0, 1);
  //   motor_many_pos_vel_MAXtqe(PORT1, 2, 0.0, 20.0, 1);
  //   motor_many_pos_vel_MAXtqe(PORT1, 3, 0.0, 20.0, 1);
  //   motor_many_pos_vel_MAXtqe(PORT2, 1, 0.0, 20.0, 1);
  //   motor_many_pos_vel_MAXtqe(PORT2, 2, 0.0, 20.0, 1);
  //   motor_many_pos_vel_MAXtqe(PORT2, 3, 0.0, 20.0, 1);
  //   motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  //   motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    usb_cdc_command_t received_command;

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
  hfdcan2.Init.MessageRAMOffset = 0;
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
