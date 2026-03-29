#ifndef DJI_A_BOARD_HOLDER_DATA_PROCESSING_H
#define DJI_A_BOARD_HOLDER_DATA_PROCESSING_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "../../User_Drives/user_uart.h"
#include "../../User_Drives/user_HWT906.h"
#include "../../User_Drives/user_dji_vt03.h"
#include "../../User_Application/PC_communication.h"
/*类定义----------------------------------------------------------------------*/

typedef struct {
    int16_t a;
    int8_t m;
    int16_t v;
    int16_t p;
}ANAC;//以上均为无量纲参数

typedef struct {
    int16_t holder_pitch;
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
    int16_t value_x;
    int16_t value_y;
    uint16_t value_max;
    uint16_t user_time_flash;
    ANAC anac;
}Holder_Data;



/*函数声明-----------------------------------------------------------------------*/
void user_data_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC);

#endif //DJI_A_BOARD_HOLDER_DATA_PROCESSING_H