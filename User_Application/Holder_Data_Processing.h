#ifndef DJI_A_BOARD_HOLDER_DATA_PROCESSING_H
#define DJI_A_BOARD_HOLDER_DATA_PROCESSING_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "../../User_Drives/user_uart.h"
#include "../../User_Drives/user_HWT906.h"
#include "../../User_Drives/user_dji_vt03.h"
#include "../../User_Application/PC_communication.h"
/*类定义----------------------------------------------------------------------*/
typedef struct {
    int16_t holder_pitch;
    int16_t holder_yaw;
    int16_t w_theta_chassis;
    int16_t d_theta_turret;
    int16_t v_y;
    int16_t v_x;
    short angle_z;
    uint8_t key_left;
    uint8_t key_right;
    uint8_t key_mode;
    uint8_t key_shoot;
    uint8_t key_back;
    int16_t wheel;
    float pitch_angle;
}Holder_Data;

/*函数声明-----------------------------------------------------------------------*/
void angle_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC);

#endif //DJI_A_BOARD_HOLDER_DATA_PROCESSING_H