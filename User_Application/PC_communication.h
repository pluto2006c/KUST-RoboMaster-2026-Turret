#ifndef DJI_A_BOARD_PC_COMMUNICATION_H
#define DJI_A_BOARD_PC_COMMUNICATION_H

/*头文件包含-----------------------------------------*/
#include "main.h"
#include"../../User_Drives/user_uart.h"
#include "../../User_Application/PC_communication.h"

/*常量定义------------------------------------------*/

#define PC_BUFFLEN (12)

/*类型定义------------------------------------------*/

typedef struct {
    UART_DRIVES *user_uart;
    float holder_pitch;
    float holder_yaw;
    uint16_t shoot_delay;
}PC_DRIVES;

/*函数声明------------------------------------------*/

void PC_Init(PC_DRIVES* User_PC , UART_DRIVES* PC);
#endif //DJI_A_BOARD_PC_COMMUNICATION_H