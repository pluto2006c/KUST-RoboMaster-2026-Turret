/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp.h"
#include "bsp_config.h"
#include <stdio.h>

/* 全局注册表 --------------------------------------------------------------*/
void (*loop_event[MAX_LOOP_EVENT])(void) = {0};
uint8_t loop_event_num = 0;

void LOOP_EVENT_Handle(void) {
    for (uint8_t event_index = 0 ; event_index < loop_event_num ; event_index++) {
        loop_event[event_index]();
    }
}

/* 接口定义 --------------------------------------------------------------------*/

// 调试串口
UART_DRIVES user_debug_uart = {0};
void user_debug_uart_callback(void * user_uart) {
    UART_DRIVES *uart = (UART_DRIVES*)user_uart;
    uint8_t message[UART_BUFFER_SIZE] = {0};
    const uint16_t size = RBuffer_GetWithLen(&uart->rx_ringBuffer,message,RBuffer_GetLength(&uart->rx_ringBuffer));
    if (size) {
        char temp[UART_BUFFER_SIZE+32] = {0};
        sprintf(temp, "message: %s\ndata.size: %d\n", (char *)message,size);
        UART_QSend(uart, temp);
    }
}

// 大疆 DR16 接收机
DBUS_DRIVES user_dbus_DR16 = {0};

// 状态灯
LED_DRIVES red_led = {0};
LED_DRIVES green_led = {0};

// can 总线
CAN_DRIVES user_can_1 = {0};
void user_can_1_callback(void * user_can) {
    CAN_DRIVES *can = (CAN_DRIVES*)user_can;
    DJI_Motor_Handle(can);

    // char message[30] = {0};
    // sprintf(message,"%d\n",RT_M3508.rotor_angle);
    // UART_QSend(&user_debug_uart, message);
}

CAN_DRIVES user_can_2 = {0};
void user_can_2_callback(void * user_can) {
//    CAN_DRIVES *can = (CAN_DRIVES*)user_can;

}

// 大疆电机
DJI_MOTOR_DRIVES PICH_GM6020 = {0};
DJI_MOTOR_DRIVES THC_GM6020  = {0};
DJI_MOTOR_DRIVES LW_M3508    = {0};
DJI_MOTOR_DRIVES RW_M3508    = {0};
DJI_MOTOR_DRIVES TP_M2006    = {0};

//大疆图传
VT03_DRIVES user_vt03 = {0};












