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
static void USB_ApplyGainTestCommandToMotors(const usb_cdc_gain_test_command_t *command,
                                              uint8_t command_fresh);
static void USB_ConfigureMotorTorqueLimits(void);
static void USB_ConfigureTorqueLimits(float max_torque_nm);
static void USB_SendRobotState(void);
static void IMU_UART_Start(void);
static void IMU_UART_Process(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USB_STATE_PERIOD_MS 20U
#define USB_CONTROL_PERIOD_MS 20U
#define WWDG_REFRESH_PERIOD_MS 100U
#define USB_CONTROL_TARGET_VELOCITY     0.0f
#define USB_CONTROL_FEEDFORWARD_TORQUE  0.0f
#define USB_CONTROL_MAX_TORQUE          2.0f
#define USB_STARTUP_MAX_VELOCITY_DEG_S  20.0f
#define USB_STARTUP_ACCELERATION_DEG_S2 40.0f
#define USB_GAIN_TEST_COMMAND_TIMEOUT_MS 100U
#define USB_GAIN_TEST_MAX_TORQUE_NM      2.0f
#define USB_GAIN_TEST_MAX_KP_NM_PER_RAD  100.0f
#define USB_GAIN_TEST_MAX_KD_NMS_PER_RAD 10.0f
#define USB_TWO_PI                       6.2831853071795864769f
#define MOTOR_FEEDBACK_MONITOR_TIMEOUT_MS 60U
#define ROBOT_JOINT_COUNT 6U

#define IMU_FRAME_HEADER         0xFCU
#define IMU_FRAME_END            0xFDU
#define IMU_PACKET_IMU           0x40U
#define IMU_PACKET_AHRS          0x41U
#define IMU_PAYLOAD_IMU_LENGTH   56U
#define IMU_PAYLOAD_AHRS_LENGTH  48U
#define IMU_FRAME_OVERHEAD       8U
#define IMU_FRAME_MAX_LENGTH     (255U + IMU_FRAME_OVERHEAD)
#define IMU_UART_RX_BUFFER_SIZE  512U
#define IMU_UART_RX_BUFFER_MASK  (IMU_UART_RX_BUFFER_SIZE - 1U)
#define IMU_DATA_TIMEOUT_MS      200U

#define IMU_STATUS_RAW_VALID     0x0001U
#define IMU_STATUS_QUAT_VALID    0x0002U
#define MOTOR_STATUS_FEEDBACK_FRESH 0x0100U
#define MOTOR_STATUS_SUSPECT_FEEDBACK 0x0200U
#define MOTOR_STATUS_RX_FIFO_LOST 0x0400U
#define MOTOR_STATUS_TX_ENQUEUE_ERROR 0x0800U
#define MOTOR_STATUS_GAIN_TEST_ACTIVE 0x1000U
#define IMU_STATUS_RX_OVERFLOW   0x8000U

typedef struct
{
  float accel_m_s2[3];
  float gyro_rad_s[3];
  float mag[3];
  float quat[4];
  uint32_t raw_timestamp_ms;
  uint32_t quat_timestamp_ms;
  uint16_t status;
} imu_data_t;

static usb_cdc_command_t g_current_command = {0};
static usb_cdc_gain_test_command_t g_gain_test_command = {0};
static uint8_t g_gain_test_active = 0U;
static uint32_t g_last_gain_test_command_tick = 0U;
static float g_configured_max_torque_nm = USB_CONTROL_MAX_TORQUE;
/* Keep this pose synchronized with DEFAULT_JOINT_POS_RAD in pc_32_linux.py. */
static const float g_startup_joint_pos_deg[ROBOT_JOINT_COUNT] = {
  10.0f, -20.0f, 10.0f, 10.0f, -20.0f, 10.0f
};
static const float g_joint_min_deg[ROBOT_JOINT_COUNT] = {
  -60.0f, -120.0f, -45.0f, -60.0f, -120.0f, -45.0f
};
static const float g_joint_max_deg[ROBOT_JOINT_COUNT] = {
  60.0f, 0.0f, 45.0f, 60.0f, 0.0f, 45.0f
};
static uint32_t g_last_state_tick = 0U;
static uint32_t g_last_control_tick = 0U;
static uint32_t g_last_wwdg_refresh_tick = 0U;
static imu_data_t g_imu_data = {0};
static uint8_t g_imu_uart_rx_byte = 0U;
static uint8_t g_imu_uart_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_imu_uart_rx_head = 0U;
static volatile uint16_t g_imu_uart_rx_tail = 0U;
static volatile uint8_t g_imu_uart_rx_overflow = 0U;

static uint8_t IMU_CRC8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x01U) != 0U) ? (uint8_t)((crc >> 1U) ^ 0x8CU)
                                  : (uint8_t)(crc >> 1U);
    }
  }

  return crc;
}

static uint16_t IMU_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0U;

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                    : (uint16_t)(crc << 1U);
    }
  }

  return crc;
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

/*
 * FDILink's ROS driver maps the sensor frame to the robot base frame with a
 * fixed 180-degree rotation about X:
 *   vector:     [x, y, z] -> [x, -y, -z]
 *   quaternion: [w, x, y, z] -> [w, x, -y, -z]
 *
 * Apply the same transform to every IMU quantity exported to the host so the
 * angular velocity and orientation observations use one consistent frame.
 */
static void IMU_VectorSensorToBase(const float sensor[3], float base[3])
{
  base[0] = sensor[0];
  base[1] = -sensor[1];
  base[2] = -sensor[2];
}

static void IMU_QuaternionSensorToBase(const float sensor[4], float base[4])
{
  base[0] = sensor[0];
  base[1] = sensor[1];
  base[2] = -sensor[2];
  base[3] = -sensor[3];
}

static void IMU_ParseFrame(const uint8_t *frame, uint16_t length)
{
  uint8_t payload_length = frame[2];
  const uint8_t *payload = &frame[7];
  uint16_t received_crc16;

  if ((length != ((uint16_t)payload_length + IMU_FRAME_OVERHEAD)) ||
      (frame[length - 1U] != IMU_FRAME_END) ||
      (IMU_CRC8(frame, 4U) != frame[4]))
  {
    return;
  }

  received_crc16 = ((uint16_t)frame[5] << 8U) | frame[6];
  if (IMU_CRC16(payload, payload_length) != received_crc16)
  {
    return;
  }

  if ((frame[1] == IMU_PACKET_IMU) && (payload_length == IMU_PAYLOAD_IMU_LENGTH))
  {
    float gyro[3];
    float accel[3];
    float mag[3];

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      gyro[axis] = IMU_ReadFloatLE(&payload[axis * 4U]);
      accel[axis] = IMU_ReadFloatLE(&payload[12U + axis * 4U]);
      mag[axis] = IMU_ReadFloatLE(&payload[24U + axis * 4U]);
    }

    if (isfinite(gyro[0]) && isfinite(gyro[1]) && isfinite(gyro[2]))
    {
      IMU_VectorSensorToBase(gyro, g_imu_data.gyro_rad_s);
      IMU_VectorSensorToBase(accel, g_imu_data.accel_m_s2);
      IMU_VectorSensorToBase(mag, g_imu_data.mag);
      g_imu_data.raw_timestamp_ms = HAL_GetTick();
      g_imu_data.status |= IMU_STATUS_RAW_VALID;
    }
  }
  else if ((frame[1] == IMU_PACKET_AHRS) && (payload_length == IMU_PAYLOAD_AHRS_LENGTH))
  {
    float gyro[3];
    float quat[4];
    float norm_squared = 0.0f;

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      gyro[axis] = IMU_ReadFloatLE(&payload[axis * 4U]);
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      /* FDILink Q1..Q4 are Qw, Qx, Qy, Qz, as in the vendor example. */
      quat[i] = IMU_ReadFloatLE(&payload[24U + i * 4U]);
      norm_squared += quat[i] * quat[i];
    }

    if (isfinite(gyro[0]) && isfinite(gyro[1]) && isfinite(gyro[2]))
    {
      IMU_VectorSensorToBase(gyro, g_imu_data.gyro_rad_s);
      g_imu_data.raw_timestamp_ms = HAL_GetTick();
      g_imu_data.status |= IMU_STATUS_RAW_VALID;
    }

    if (isfinite(norm_squared) && (norm_squared > 0.25f) && (norm_squared < 4.0f))
    {
      float inverse_norm = 1.0f / sqrtf(norm_squared);
      float normalized_quat[4];

      for (uint8_t i = 0U; i < 4U; ++i)
      {
        normalized_quat[i] = quat[i] * inverse_norm;
      }
      IMU_QuaternionSensorToBase(normalized_quat, g_imu_data.quat);

      g_imu_data.quat_timestamp_ms = HAL_GetTick();
      g_imu_data.status |= IMU_STATUS_QUAT_VALID;
    }
  }
}

static void IMU_UART_Process(void)
{
  static uint8_t frame[IMU_FRAME_MAX_LENGTH];
  static uint16_t frame_index = 0U;
  static uint16_t frame_length = 0U;

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
      if (byte == IMU_FRAME_HEADER)
      {
        frame[frame_index++] = byte;
      }
    }
    else if (frame_index == 2U)
    {
      frame[frame_index++] = byte;
      frame_length = (uint16_t)byte + IMU_FRAME_OVERHEAD;
    }
    else
    {
      frame[frame_index++] = byte;

      if ((frame_index == 5U) && (IMU_CRC8(frame, 4U) != frame[4]))
      {
        frame_index = 0U;
        frame_length = 0U;
        continue;
      }

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

static void USB_ApplyCommandToMotors(const usb_cdc_command_t *command)
{
  if (command == NULL)
  {
    return;
  }

  if ((command->flags & USB_CDC_COMMAND_FLAG_STARTUP_TRAJECTORY) != 0U)
  {
    /*
     * Use the drive's trapezoidal trajectory generator while moving to the
     * policy default pose. This avoids commanding a moving position target
     * together with zero desired velocity through the policy-time PD loop.
     */
    motor_many_pos_vel_acc(PORT1, 1, command->target_joint_pos[0], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
    motor_many_pos_vel_acc(PORT1, 2, command->target_joint_pos[1], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
    motor_many_pos_vel_acc(PORT1, 3, command->target_joint_pos[2], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
    motor_many_pos_vel_acc(PORT2, 1, command->target_joint_pos[3], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
    motor_many_pos_vel_acc(PORT2, 2, command->target_joint_pos[4], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
    motor_many_pos_vel_acc(PORT2, 3, command->target_joint_pos[5], USB_STARTUP_MAX_VELOCITY_DEG_S, USB_STARTUP_ACCELERATION_DEG_S2);
  }
  else
  {
    float kp_nm_per_turn;
    float kd_nms_per_turn;

    if (!isfinite(command->kp_nm_per_rad) ||
        !isfinite(command->kd_nms_per_rad) ||
        (command->kp_nm_per_rad < 0.0f) ||
        (command->kp_nm_per_rad > USB_GAIN_TEST_MAX_KP_NM_PER_RAD) ||
        (command->kd_nms_per_rad < 0.0f) ||
        (command->kd_nms_per_rad > USB_GAIN_TEST_MAX_KD_NMS_PER_RAD))
    {
      return;
    }

    /* The drive evaluates errors in turns; convert the PC's SI radian gains. */
    kp_nm_per_turn = command->kp_nm_per_rad * USB_TWO_PI;
    kd_nms_per_turn = command->kd_nms_per_rad * USB_TWO_PI;
    motor_many_pos_vel_tqe_kp_kd_2(PORT1, 1, command->target_joint_pos[0], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
    motor_many_pos_vel_tqe_kp_kd_2(PORT1, 2, command->target_joint_pos[1], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
    motor_many_pos_vel_tqe_kp_kd_2(PORT1, 3, command->target_joint_pos[2], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
    motor_many_pos_vel_tqe_kp_kd_2(PORT2, 1, command->target_joint_pos[3], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
    motor_many_pos_vel_tqe_kp_kd_2(PORT2, 2, command->target_joint_pos[4], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
    motor_many_pos_vel_tqe_kp_kd_2(PORT2, 3, command->target_joint_pos[5], USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE, kp_nm_per_turn, kd_nms_per_turn);
  }

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
}

static p_motor_state_s USB_GetJointMotorState(uint8_t joint_index)
{
  if (joint_index >= ROBOT_JOINT_COUNT)
  {
    return NULL;
  }

  if (joint_index < 3U)
  {
    return motor_get_state(PORT1, (uint8_t)(joint_index + 1U));
  }

  return motor_get_state(PORT2, (uint8_t)(joint_index - 2U));
}

static void USB_SetGainTestMotor(uint8_t joint_index, float target_position_deg,
                                 float kp_nm_per_turn, float kd_nms_per_turn)
{
  if (joint_index < 3U)
  {
    motor_many_pos_vel_tqe_kp_kd_2(
        PORT1, (uint8_t)(joint_index + 1U), target_position_deg,
        USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE,
        kp_nm_per_turn, kd_nms_per_turn);
  }
  else
  {
    motor_many_pos_vel_tqe_kp_kd_2(
        PORT2, (uint8_t)(joint_index - 2U), target_position_deg,
        USB_CONTROL_TARGET_VELOCITY, USB_CONTROL_FEEDFORWARD_TORQUE,
        kp_nm_per_turn, kd_nms_per_turn);
  }
}

static void USB_ApplyGainTestCommandToMotors(
    const usb_cdc_gain_test_command_t *command, uint8_t command_fresh)
{
  uint8_t selected_joint = ROBOT_JOINT_COUNT;
  float selected_target_deg = 0.0f;
  float kp_nm_per_turn = 0.0f;
  float kd_nms_per_turn = 0.0f;
  uint8_t feedback_fresh = motor_all_active_states_fresh(
      HAL_GetTick(), MOTOR_FEEDBACK_MONITOR_TIMEOUT_MS);

  if ((command != NULL) && (command_fresh != 0U) &&
      ((command->flags & USB_CDC_GAIN_TEST_FLAG_ENABLE) != 0U) &&
      (command->joint_index < ROBOT_JOINT_COUNT) &&
      isfinite(command->target_position_deg) &&
      isfinite(command->kp_nm_per_rad) &&
      isfinite(command->kd_nms_per_rad) &&
      (command->kp_nm_per_rad >= 0.0f) &&
      (command->kp_nm_per_rad <= USB_GAIN_TEST_MAX_KP_NM_PER_RAD) &&
      (command->kd_nms_per_rad >= 0.0f) &&
      (command->kd_nms_per_rad <= USB_GAIN_TEST_MAX_KD_NMS_PER_RAD) &&
      (feedback_fresh != 0U))
  {
    selected_joint = command->joint_index;
    selected_target_deg = fmaxf(
        g_joint_min_deg[selected_joint],
        fminf(g_joint_max_deg[selected_joint], command->target_position_deg));

    /*
     * The motor protocol computes position and velocity errors in turns and
     * turns/s. Multiplying the SI gains by 2*pi preserves the requested
     * N*m/rad and N*m*s/rad gains before the vendor pid_adjust() conversion.
     */
    kp_nm_per_turn = command->kp_nm_per_rad * USB_TWO_PI;
    kd_nms_per_turn = command->kd_nms_per_rad * USB_TWO_PI;
  }

  for (uint8_t joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
  {
    p_motor_state_s motor_state = USB_GetJointMotorState(joint);
    float target_position_deg =
        (motor_state != NULL) ? motor_state->position : 0.0f;
    float joint_kp = 0.0f;
    float joint_kd = 0.0f;

    if (joint == selected_joint)
    {
      target_position_deg = selected_target_deg;
      joint_kp = kp_nm_per_turn;
      joint_kd = kd_nms_per_turn;
    }

    USB_SetGainTestMotor(joint, target_position_deg, joint_kp, joint_kd);
  }

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
}

static void USB_ConfigureMotorTorqueLimits(void)
{
  USB_ConfigureTorqueLimits(USB_CONTROL_MAX_TORQUE);
}

static void USB_ConfigureTorqueLimits(float max_torque_nm)
{
  max_torque_nm = fmaxf(0.0f, fminf(USB_GAIN_TEST_MAX_TORQUE_NM,
                                   max_torque_nm));

  /*
   * True motion-control packets do not carry register 0x025 (maximum torque).
   * Seed it through the position/velocity/maximum-torque mode before entering
   * true motion control. NAN means no position target and zero velocity prevents
   * an intentional startup motion; the same frames request the initial states.
   */
  motor_many_pos_vel_MAXtqe(PORT1, 1, NAN_FLOAT, 0.0f, max_torque_nm);
  motor_many_pos_vel_MAXtqe(PORT1, 2, NAN_FLOAT, 0.0f, max_torque_nm);
  motor_many_pos_vel_MAXtqe(PORT1, 3, NAN_FLOAT, 0.0f, max_torque_nm);
  motor_many_pos_vel_MAXtqe(PORT2, 1, NAN_FLOAT, 0.0f, max_torque_nm);
  motor_many_pos_vel_MAXtqe(PORT2, 2, NAN_FLOAT, 0.0f, max_torque_nm);
  motor_many_pos_vel_MAXtqe(PORT2, 3, NAN_FLOAT, 0.0f, max_torque_nm);

  motor_many_send(PORT1, MANY_GET_POS_VEL_TQE);
  motor_many_send(PORT2, MANY_GET_POS_VEL_TQE);
  g_configured_max_torque_nm = max_torque_nm;
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

  if ((g_gain_test_active != 0U) &&
      (g_gain_test_command.joint_index < ROBOT_JOINT_COUNT))
  {
    p_motor_state_s selected_state = USB_GetJointMotorState(
        g_gain_test_command.joint_index);
    if (selected_state != NULL)
    {
      state.cmd[0] = selected_state->torque;
    }
    state.cmd[1] = g_gain_test_command.kp_nm_per_rad;
    state.cmd[2] = g_gain_test_command.kd_nms_per_rad;
    imu_status |= MOTOR_STATUS_GAIN_TEST_ACTIVE;
  }

  state.timestamp_ms = now;
  if (motor_all_active_states_fresh(now, MOTOR_FEEDBACK_MONITOR_TIMEOUT_MS) != 0U)
  {
    imu_status |= MOTOR_STATUS_FEEDBACK_FRESH;
  }
  if (motor_get_suspect_count() != 0U)
  {
    imu_status |= MOTOR_STATUS_SUSPECT_FEEDBACK;
  }
  if (motor_get_rx_lost_count() != 0U)
  {
    imu_status |= MOTOR_STATUS_RX_FIFO_LOST;
  }
  if (fdcan_get_tx_error_count() != 0U)
  {
    imu_status |= MOTOR_STATUS_TX_ENQUEUE_ERROR;
  }
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

  USB_ConfigureMotorTorqueLimits();
  HAL_Delay(2U);
  USB_SendRobotState();

  /* Move to the policy's default pose immediately after motor power-up. */
  memcpy(g_current_command.target_joint_pos, g_startup_joint_pos_deg,
         sizeof(g_startup_joint_pos_deg));
  g_current_command.flags = USB_CDC_COMMAND_FLAG_STARTUP_TRAJECTORY;
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
    usb_cdc_gain_test_command_t received_gain_test_command;

    IMU_UART_Process();
    motor_process_state_all();

    if (USB_CDC_GetLatestCommand(&received_command) != 0U)
    {
      if (g_gain_test_active != 0U)
      {
        USB_ConfigureMotorTorqueLimits();
        g_gain_test_active = 0U;
      }
      g_current_command = received_command;
    }

    if (USB_CDC_GetLatestGainTestCommand(&received_gain_test_command) != 0U)
    {
      uint8_t command_valid =
          (received_gain_test_command.joint_index < ROBOT_JOINT_COUNT) &&
          isfinite(received_gain_test_command.target_position_deg) &&
          isfinite(received_gain_test_command.kp_nm_per_rad) &&
          isfinite(received_gain_test_command.kd_nms_per_rad) &&
          isfinite(received_gain_test_command.max_torque_nm) &&
          (received_gain_test_command.kp_nm_per_rad >= 0.0f) &&
          (received_gain_test_command.kp_nm_per_rad <=
           USB_GAIN_TEST_MAX_KP_NM_PER_RAD) &&
          (received_gain_test_command.kd_nms_per_rad >= 0.0f) &&
          (received_gain_test_command.kd_nms_per_rad <=
           USB_GAIN_TEST_MAX_KD_NMS_PER_RAD) &&
          (received_gain_test_command.max_torque_nm >= 0.0f) &&
          (received_gain_test_command.max_torque_nm <=
           USB_GAIN_TEST_MAX_TORQUE_NM);

      if (command_valid == 0U)
      {
        received_gain_test_command.flags = 0U;
        received_gain_test_command.max_torque_nm = 0.0f;
      }

      if ((g_gain_test_active == 0U) ||
          (fabsf(received_gain_test_command.max_torque_nm -
                 g_configured_max_torque_nm) > 0.0001f))
      {
        USB_ConfigureTorqueLimits(received_gain_test_command.max_torque_nm);
      }

      g_gain_test_command = received_gain_test_command;
      g_gain_test_active = 1U;
      g_last_gain_test_command_tick = HAL_GetTick();
    }

    if ((HAL_GetTick() - g_last_control_tick) >= USB_CONTROL_PERIOD_MS)
    {
      if (g_gain_test_active != 0U)
      {
        uint8_t command_fresh =
            ((HAL_GetTick() - g_last_gain_test_command_tick) <=
             USB_GAIN_TEST_COMMAND_TIMEOUT_MS) ? 1U : 0U;
        USB_ApplyGainTestCommandToMotors(&g_gain_test_command, command_fresh);
      }
      else
      {
        USB_ApplyCommandToMotors(&g_current_command);
      }
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
  huart1.Init.BaudRate = 921600;
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
