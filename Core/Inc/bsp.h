#ifndef USER_BSP_H
#define USER_BSP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "../../SEGGER_RTT/SEGGER_RTT.h"
#include "../../User_Drives/User_Motor/user_dji_motor.h"
#include "../../User_Algorithm/User_Controller/user_pid.h"
#include "../../User_Drives/User_remote/user_dji_vt03.h"
#include "../../User_Drives/user_hwt906.h"
#include "../../User_Application/PC_communication.h"
#include "../../User_Application/Holder_Data_Processing.h"
#include "../../User_Algorithm/User_Controller/user_pid_tolerance.h"
#include "../../User_Algorithm/User_Controller/user_ladrc.h"

/* 全局注册表 ----------------------------------------------------------------*/
#define MAX_LOOP_EVENT 32
void LOOP_EVENT_Handle(void);
typedef void (*LOOP_Event)(void);
extern LOOP_Event loop_event[MAX_LOOP_EVENT];
extern uint8_t loop_event_num;

/* JScope ------------------------------------------------------------------*/
#include "../../SEGGER_RTT/user_JScope_Transmit.h"
extern CCMRAM JScope_Transmit_t jscope_transmit;
extern CCMRAM uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP];

/* 接口定义 ------------------------------------------------------------------*/

/*底盘CAN地址注册--------------------------------------------------------------*/
#define Chassis_data_ID_1          (0x200)
#define Chassis_data_ID_2          (0x201)
#define Chassis_data_ID_3          (0x202)

// 调试串口
#include "../../User_Drives/user_uart.h"
extern UART_DRIVES user_debug_uart;

// 状态灯
#include "../../User_Drives/user_led.h"
extern LED_DRIVES user_red_led;
extern LED_DRIVES user_green_led;

// can 总线
#include "../../User_Drives/user_can.h"
extern CAN_DRIVES user_can_1;
extern CAN_DRIVES user_can_2;

float can_RX_callback( CAN_DRIVES* user_can);

// 蜂鸣器
#include "../../User_Drives/user_pwm.h"

//控制器注冊
extern PID_Controller TP_M2006_Controller;
extern PID_Controller LW_M3508_Controller;
extern PID_Controller RW_M3508_Controller;
extern LADRC_Controller GM_6020_Controller;

//电机注册

extern DJI_MOTOR_DRIVES TP_M2006;
extern DJI_MOTOR_DRIVES LW_M3508;
extern DJI_MOTOR_DRIVES RW_M3508;
extern DJI_MOTOR_DRIVES PICH_GM6020;

//遥控器注册
extern UART_DRIVES vt03_uart;
extern VT03_DRIVES user_vt03;

//陀螺仪注册
extern UART_DRIVES hwt906_uart_chassis;
extern HWT906_DRIVES user_HWT906_chassis;

//电脑注册
extern UART_DRIVES PC_uart;
extern PC_DRIVES user_PC;

//全局数据包注册
extern Holder_Data user_holder_data;

//全局虚拟遥控器注册
extern USER_REMOTE virtual_user_remote;

#endif // USER_BSP_H
