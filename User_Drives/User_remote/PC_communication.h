#ifndef DJI_A_BOARD_PC_COMMUNICATION_H
#define DJI_A_BOARD_PC_COMMUNICATION_H

/*头文件包含-----------------------------------------*/
#include "main.h"
#include"../../User_Drives/user_uart.h"
#include "../../User_Drives/User_remote/PC_communication.h"

/*常量定义------------------------------------------*/

#define PC_BUFFLEN (14)

/*类型定义------------------------------------------*/

typedef struct {
    UART_DRIVES *user_uart;   /* UART 驱动结构体指针 */
    float holder_pitch;       /* 云台俯仰角 */
    float holder_yaw;         /* 云台偏航角 */
    uint16_t shoot_delay;     /* 射击延迟时间 */
}PC_DRIVES;

/*函数声明------------------------------------------*/

void PC_Init(PC_DRIVES* User_PC , UART_DRIVES* PC);
#endif //DJI_A_BOARD_PC_COMMUNICATION_H