#ifndef __USER_BSP_H__
#define __USER_BSP_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"

/* 全局注册表 ----------------------------------------------------------------*/
#define MAX_LOOP_EVENT 32
void LOOP_EVENT_Handle(void);
typedef void (*LOOP_Event)(void);
extern LOOP_Event loop_event[MAX_LOOP_EVENT];
extern uint8_t loop_event_num;

/* 接口定义 ------------------------------------------------------------------*/
// 调试串口
#include "../../User_Drives/Inc/user_uart.h"
extern UART_DRIVES user_debug_uart;
void user_debug_uart_callback(void * user_uart);
// 大疆 DR16 接收机
#include "../../User_Drives/Inc/user_dji_bus.h"
extern DBUS_DRIVES user_dbus_DR16;

// 状态灯
#include "../../User_Drives/Inc/user_led.h"
extern LED_DRIVES red_led;
extern LED_DRIVES green_led;

// can 总线
#include "../../User_Drives/Inc/user_can.h"
extern CAN_DRIVES user_can_1;
void user_can_1_callback(void * user_can);
extern CAN_DRIVES user_can_2;
void user_can_2_callback(void * user_can);

// 大疆电机
#include "../../User_Drives/Inc/user_dji_motor.h"
extern DJI_MOTOR_DRIVES PICH_GM6020;
extern DJI_MOTOR_DRIVES THC_GM6020;
extern DJI_MOTOR_DRIVES LW_M3508;
extern DJI_MOTOR_DRIVES RW_M3508;
extern DJI_MOTOR_DRIVES TP_M2006;

//大疆图传
#include "../../User_Drives/Inc/user_dji_vt03.h"
extern VT03_DRIVES user_vt03;

/* 全局结构体 ----------------------------------------------------------------*/



#endif //__USER_BSP_H__
