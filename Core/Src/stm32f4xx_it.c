/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bsp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  // user_dbus_DR16 是大疆接收机的结构体（可以粗糙的理解为变量的文件夹）。
  // 其中的变量有 ch0、ch1、ch2、ch3、roll、sw1、sw2
  // 其中 ch0、ch1、ch2、ch3、roll 的取值范围是 -660到660
  // 其中 sw1、sw2 可能的值为 1、2、3 。默认中间记得值是 1

  // 舵电机控制
  // 函数的第二个参数是目标角度，取值范围 0-8095，对应 0-360 度
  DJI_Motor_Target(&PICH_GM6020, 3700.0700f - 3.0667f * 0.8f * (float) user_dbus_DR16.ch1);
  if (user_dbus_DR16.sw2==1) {
    DJI_Motor_Target(&LW_M3508, -5000);
    DJI_Motor_Target(&RW_M3508, 5000);
    if (user_dbus_DR16.sw1==1) {
      DJI_Motor_Target(&TP_M2006, -400);
    }else if (user_dbus_DR16.sw1==2) {
      DJI_Motor_Target(&TP_M2006, -3000);
    }else {
      DJI_Motor_Target(&TP_M2006, 0);
    }
  }else if (user_dbus_DR16.sw2==2) {
    DJI_Motor_Target(&TP_M2006, 400);
    DJI_Motor_Target(&LW_M3508, -5000);
    DJI_Motor_Target(&RW_M3508, 5000);
  }else {
    DJI_Motor_Target(&LW_M3508, 0);
    DJI_Motor_Target(&RW_M3508, 0);
  }
  DJI_Motor_Execute(&user_can_1);

  uint8_t user_can_2_send_frame[8] = {0};

  user_can_2_send_frame [0] = (uint8_t) (user_dbus_DR16.ch2 >> 0);
  user_can_2_send_frame [1] = (uint8_t) (user_dbus_DR16.ch2 >> 8);
  user_can_2_send_frame [2] = (uint8_t) (user_dbus_DR16.ch3 >> 0);
  user_can_2_send_frame [3] = (uint8_t) (user_dbus_DR16.ch3 >> 8);
  user_can_2_send_frame [4] = (uint8_t) (user_dbus_DR16.ch0 >> 0);
  user_can_2_send_frame [5] = (uint8_t) (user_dbus_DR16.ch0 >> 8);
  user_can_2_send_frame [6] = (uint8_t) (user_dbus_DR16.ch1 >> 0);
  user_can_2_send_frame [7] = (uint8_t) (user_dbus_DR16.ch1 >> 8);

  CAN_Send(&user_can_2, Re_control_data_ID_1 , user_can_2_send_frame, 8);

 

  // 发送轮电机控制报文

  /*uint8_t C6x0_control_id_1_frame   [8] = {0};

  C6x0_control_id_1_frame   [2 * 1 - 1] = (uint8_t) ((int16_t)( 10 * ch1) >> 0);
  C6x0_control_id_1_frame   [2 * 1 - 2] = (uint8_t) ((int16_t)( 10 * ch1) >> 8);
  C6x0_control_id_1_frame   [2 * 2 - 1] = (uint8_t) ((int16_t)( 10 * ch1) >> 0);
  C6x0_control_id_1_frame   [2 * 2 - 2] = (uint8_t) ((int16_t)( 10 * ch1) >> 8);
  C6x0_control_id_1_frame   [2 * 3 - 1] = (uint8_t) ((int16_t)( 10 * ch1) >> 0);
  C6x0_control_id_1_frame   [2 * 3 - 2] = (uint8_t) ((int16_t)( 10 * ch1) >> 8);
  C6x0_control_id_1_frame   [2 * 4 - 1] = (uint8_t) ((int16_t)(-10 * ch1) >> 0);
  C6x0_control_id_1_frame   [2 * 4 - 2] = (uint8_t) ((int16_t)(-10 * ch1) >> 8);

  CAN_Send(&user_can_1, C6x0_CURRENT_CONTROL_ID_1, C6x0_control_id_1_frame, 8);*/

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
