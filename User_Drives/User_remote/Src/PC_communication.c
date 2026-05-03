/* 包含头文件 ----------------------------------------------------------------*/
#include "../PC_communication.h"

/* 私有变量 ------------------------------------------------------------------*/
static PC_DRIVES* user_PC_drive = NULL;
static uint8_t buf[PC_BUFFLEN] = {0};

/* 函数体 --------------------------------------------------------------------*/


/**
* @brief PC 串口回调函数
* @param user_uart 串口驱动结构体指针
*/
static void PC_UartCallback(void* user_uart) {
    UART_DRIVES* uart = (UART_DRIVES*)user_uart;
    const char PC_buffer_head[2] ={ 0xEB , 0x90} ;
    const char PC_buffer_tail[2] ={ 0X90 , 0XEB} ;

    /* 获取串口数据 */
    if (!UART_GetDataWithHT(user_uart, buf, PC_buffer_head , PC_buffer_tail)) {
        return;
    }

    uint32_t t_holder_pitch = (uint32_t)buf[2] | (uint32_t)(buf[3] << 8) | (uint32_t)(buf[4] << 16) | (uint32_t)(buf[5] << 24);
    user_PC_drive -> holder_pitch = *(float*)&t_holder_pitch;
    uint32_t t_holder_yaw = (uint32_t)buf[6] | (uint32_t)(buf[7] << 8) | (uint32_t)(buf[8] << 16) | (uint32_t)(buf[9] << 24);
    user_PC_drive -> holder_yaw  = *(float*)&t_holder_yaw;
    user_PC_drive -> shoot_delay=   (uint32_t)buf[10] | (uint16_t)(buf[11] << 8);
}

/**
* @brief 初始化 HWT906
* @param User_PC HWT906 驱动结构体指针
*/
void PC_Init(PC_DRIVES* User_PC , UART_DRIVES* PC_UART) {
    user_PC_drive = User_PC;
    User_PC->user_uart = PC_UART;
    User_PC->shoot_delay = 0xFFFF;
    /* 注册串口回调函数 */
    UART_RegisterCallback(User_PC->user_uart, PC_UartCallback);
}