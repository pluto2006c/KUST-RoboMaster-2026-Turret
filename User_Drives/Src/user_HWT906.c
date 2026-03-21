//头文件包含
#include "../../User_Drives/user_HWT906.h"

/* 私有变量 ------------------------------------------------------------------*/
static HWT906_DRIVES* HWT906_drive = NULL;
uint8_t buf[HWT906_BUFFLEN] = {0};

/* 函数声明 ------------------------------------------------------------------*/

char Get_SUMCRC(HWT906_TYPE type, uint8_t* data, uint8_t len) {
    char sum = 0;
    sum += type;
    for (uint8_t i = 0; i < len - 1; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static void HWT906_UartCallback(void* user_uart) {
    UART_DRIVES* uart = (UART_DRIVES*)user_uart;
    char buffer_head[1] = {0x55} ;

    if (!UART_GetDataWithHLen(user_uart, buf, buffer_head , HWT906_BUFFLEN)) {
        return;
    }

    if (buf[10] != Get_SUMCRC(buf[1], buf, 10)) {
        return;
    }

    switch (buf[1]) {
        case hwt906_acceleration:
            HWT906_drive->user_acceleration.acceleration_x = (short)((short)buf[2] | (buf[3] << 8));
            HWT906_drive->user_acceleration.acceleration_y = (short)((short)buf[4] | (buf[5] << 8));
            HWT906_drive->user_acceleration.acceleration_z = (short)((short)buf[6] | (buf[7] << 8));
            HWT906_drive->user_acceleration.temperature = (short)((short)buf[8] | (buf[9] << 8));
            break;

        case hwt906_angular_velocity:
            HWT906_drive->user_angular_velocity.angular_velocity_x = (short)((short)buf[2] | (buf[3] << 8));
            HWT906_drive->user_angular_velocity.angular_velocity_y = (short)((short)buf[4] | (buf[5] << 8));
            HWT906_drive->user_angular_velocity.angular_velocity_z = (short)((short)buf[6] | (buf[7] << 8));
            HWT906_drive->user_angular_velocity.voltage = (short)((short)buf[8] | (buf[9] << 8));
            break;

        case hwt906_angle:
            HWT906_drive->user_angle.angle_x = (short)((short)buf[2] | (buf[3] << 8));
            HWT906_drive->user_angle.angle_y = (short)((short)buf[4] | (buf[5] << 8));
            HWT906_drive->user_angle.angle_z = (short)((short)buf[6] | (buf[7] << 8));
            HWT906_drive->user_angle.version = (short)((short)buf[8] | (buf[9] << 8));
            break;

        default:

            break;
    }


}

void HWT906_Init(HWT906_DRIVES* User_HWT906) {
    HWT906_drive = User_HWT906;
    UART_RegisterCallback(&User_HWT906->user_uart,HWT906_UartCallback );
}