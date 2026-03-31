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
#include <sys/types.h>

#include "bsp.h"
#include "../../User_Lib/user_music.h"

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
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
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
  while (1) {
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
  user_data_processing(&user_holder_data ,&user_vt03, &user_HWT906 ,  &user_PC );//数据处理
  //PICH轴控制
  DJI_Motor_Set_State(&PICH_GM6020,  (float)user_holder_data.pitch_angle);
  //计时器
  static int user_time_counyer = 100000 ;
  static uint8_t time_flash = 0 ;
  user_holder_data.user_time_flash = user_time_counyer;

  if (user_time_counyer == 1000) {
    user_time_counyer = 0 ;
  }else {
    user_time_counyer ++ ;
  }



  //热量管理
  static float shoot_heat = 0 ;
  if (user_time_counyer % 100 == 0) {
    if (shoot_heat <= 198.2) {
      shoot_heat += 1.2f ;
    }else {
      shoot_heat = 200 ;
    }
  }

  //模式控制
  static uint8_t shoot_mode = 0 ;
  static uint8_t ai_shoot_mode = 0 ;

  if (shoot_mode == 0 || shoot_mode == 1) {
    if (user_time_counyer % 10 == 0) {
      DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) + 81.91));
    }
    if (DJI_Motor_Get_Speed(&TP_M2006) > 50)
    {
      shoot_mode = 1 ;
    }
  }

  if (shoot_mode == 1 && DJI_Motor_Get_Speed(&TP_M2006) < 5) {
    shoot_mode = 2 ;
  }

  if (shoot_mode == 2) {
    DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1152.0f));
    shoot_mode = 3 ;
  }

  //发射机构控制
  if (shoot_mode == 3) {

    //全自动连发
    // if (user_PC.shoot_delay != 0xFFFF && ai_shoot_mode == 0) {
    //   time_flash = user_time_counyer;
    //   ai_shoot_mode = 1 ;
    // }
    // if (ai_shoot_mode == 1 && user_time_counyer - time_flash == user_PC.shoot_delay && shoot_heat >= 10) {
    //   DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
    //   shoot_heat -= 10 ;
    //   ai_shoot_mode = 0 ;
    // }

    //连发
    if (user_holder_data.key_mode == 1 || user_holder_data.key_mode == 2){
      DJI_Motor_Set_State(&RW_M3508, 6500);
      DJI_Motor_Set_State(&LW_M3508, -6500);
      if (user_holder_data.key_shoot == 1 && ai_shoot_mode == 0) {
        //发射频率计时
        if (user_time_counyer % 33 == 0 && shoot_heat >= 10 ) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
          shoot_heat -= 10 ;
        }
      }
      if (user_holder_data.key_back == 1) {
        if (user_time_counyer % 1000 == 0) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) + 1296.0f));
        }
      }
    }

    if (user_holder_data.key_mode == 0){
      DJI_Motor_Set_State(&RW_M3508, 0);
      DJI_Motor_Set_State(&LW_M3508, 0);
    }
  }

  DJI_Motor_Execute(&user_can_1);

  //底盘通信

  if (user_time_counyer % 2) {
    uint8_t user_can_2_send_frame_1[8] = {0};

    user_can_2_send_frame_1 [0] = (uint8_t) (user_holder_data.w_theta_chassis >> 0);
    user_can_2_send_frame_1 [1] = (uint8_t) (user_holder_data.w_theta_chassis >> 8);
    user_can_2_send_frame_1 [2] = (uint8_t) (user_holder_data.d_theta_turret >> 0);
    user_can_2_send_frame_1 [3] = (uint8_t) (user_holder_data.d_theta_turret >> 8);
    user_can_2_send_frame_1 [4] = (uint8_t) (user_holder_data.value_y >> 0);
    user_can_2_send_frame_1 [5] = (uint8_t) (user_holder_data.value_y >> 8);
    user_can_2_send_frame_1 [6] = (uint8_t) (user_holder_data.value_x >> 0);
    user_can_2_send_frame_1 [7] = (uint8_t) (user_holder_data.value_x >> 8);

    CAN_Send(&user_can_2, Chassis_data_ID_1 , user_can_2_send_frame_1, 8);

    uint8_t user_can_2_send_frame_2[8] = {0};

    user_can_2_send_frame_2 [0] = (uint8_t) (user_holder_data.w_theta_chassis >> 0);
    user_can_2_send_frame_2 [1] = (uint8_t) (user_holder_data.w_theta_chassis >> 8);
    user_can_2_send_frame_2 [2] = (uint8_t) (user_holder_data.d_theta_turret >> 0);
    user_can_2_send_frame_2 [3] = (uint8_t) (user_holder_data.d_theta_turret >> 8);
    user_can_2_send_frame_2 [4] = (uint8_t) ((int16_t)((float)user_holder_data.value_y/660*4000) >> 0);
    user_can_2_send_frame_2 [5] = (uint8_t) ((int16_t)((float)user_holder_data.value_y/660*4000) >> 8);
    user_can_2_send_frame_2 [6] = (uint8_t) ((int16_t)((float)user_holder_data.value_x/660*4000) >> 0);
    user_can_2_send_frame_2 [7] = (uint8_t) ((int16_t)((float)user_holder_data.value_x/660*4000) >> 8);

    CAN_Send(&user_can_2, Chassis_data_ID_2 , user_can_2_send_frame_2, 8);

    uint8_t user_can_2_send_frame_3[8] = {0};

    user_can_2_send_frame_3 [0] = (uint8_t) ((uint16_t)((float)user_HWT906.user_angle.angle_z * 100.0f) >> 0);
    user_can_2_send_frame_3 [1] = (uint8_t) ((uint16_t)((float)user_HWT906.user_angle.angle_z * 100.0f) >> 8);
    user_can_2_send_frame_3 [2] = 0;
    user_can_2_send_frame_3 [3] = 0;
    user_can_2_send_frame_3 [4] = 0;
    user_can_2_send_frame_3 [5] = 0;
    user_can_2_send_frame_3 [6] = 0;
    user_can_2_send_frame_3 [7] = 0;

    CAN_Send(&user_can_2, Chassis_data_ID_3 , user_can_2_send_frame_3, 8);
  }

  char angle_z[8] = {0};
  char angle_data_head[2] = { 0xEB , 0x90 };
  char angle_data_tail[2] = { 0x90 , 0xEB };
  angle_z[0] = angle_data_head[0];
  angle_z[1] = angle_data_head[1];
  const float inv_angle_z = -user_holder_data.angle_z;
  angle_z[2] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 0);
  angle_z[3] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 8);
  angle_z[4] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 16);
  angle_z[5] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 24);
  angle_z[6] = angle_data_tail[0];
  angle_z[7] = angle_data_tail[1];
  UART_Send(user_PC.user_uart, angle_z , 8);








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
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

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
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
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
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  JScope_Transmit(htim2.Init.AutoReloadPreload + 1);

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
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

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
