/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp.h"
#include <string.h>
#include <stdio.h>

/* 主循环注册表 --------------------------------------------------------------*/
void (*loop_event[MAX_LOOP_EVENT])(void) = {0};
uint8_t loop_event_num = 0;

void LOOP_EVENT_Handle(void) {
    for (uint8_t event_index = 0 ; event_index < loop_event_num ; event_index++) {
        loop_event[event_index]();
    }
}

/* JScope ------------------------------------------------------------------*/
JScope_Transmit_t jscope_transmit = {0};
uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP] = {0};

/* 接口定义 --------------------------------------------------------------------*/

// 调试串口
UART_DRIVES user_debug_uart = {0};
// 状态灯
LED_DRIVES user_red_led = {0};
LED_DRIVES user_green_led = {0};

// can 总线
CAN_DRIVES user_can_1 = {0};
CAN_DRIVES user_can_2 = {0};

//can接收函数注册
float can_RX_callback( CAN_DRIVES* user_can) {
    const CAN_DRIVES *can = (CAN_DRIVES*)user_can;
    uint8_t receive_data[8];
    if (user_can->rx_msg.StdId == Chassis_data_ID_3) {
        for (uint8_t i = 0; i < 8; i++) {
            receive_data[i] = can->rx_msg.Data[i];
        }
        user_HWT906_chassis.user_angle.angle_z = *(float*)&receive_data;
    }
    return user_HWT906_chassis.user_angle.angle_z;
}

// 蜂鸣器
PWM_DRIVES user_buzzer = {0};

// 控制器注冊

PID_Controller TP_M2006_Controller = {0};
PID_Controller RW_M3508_Controller = {0};
PID_Controller LW_M3508_Controller = {0};
LADRC_Controller GM_6020_Controller = {0};

// 电机注册
DJI_MOTOR_DRIVES TP_M2006 = {0};
DJI_MOTOR_DRIVES LW_M3508 = {0};
DJI_MOTOR_DRIVES RW_M3508 = {0};
DJI_MOTOR_DRIVES PICH_GM6020 = {0};

//遥控器注册
UART_DRIVES vt03_uart = {0};
VT03_DRIVES user_vt03 = {0};
DBUS_DRIVES dji_user_dbus = {0};

//陀螺仪注册

UART_DRIVES hwt906_uart_chassis = {0};

HWT906_DRIVES user_HWT906_chassis = {0};

//电脑注册
UART_DRIVES PC_uart = {0};
PC_DRIVES user_PC = {0};


//全局虚拟遥控器注册
USER_REMOTE virtual_user_remote = {0};