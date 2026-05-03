#ifndef DJI_A_BOARD_USER_REMOTE_H
#define DJI_A_BOARD_USER_REMOTE_H
/*实体遥控器引入-----------------------------------------------------*/

#include "../../User_remote/PC_communication.h"
#include "../../User_Drives/User_remote/user_dji_bus.h"
#include "../../User_Drives/User_remote/user_dji_vt03.h"

/*用户配置导入-----------------------------------------------------*/

#include "../../User_Drives/Controller_config/ZhouZishun.h"

/*遥控器最大数量声明-------------------------------------------------*/

#define USER_REMOTE_NUM (10)


/* 类型定义 ------------------------------------------------------------------*/

typedef void(*controller_config)(void* controler_remote);

typedef struct {
    controller_config remote_config;
    uint16_t chassis_x;
    uint16_t chassis_y;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t wheel;
    uint16_t shoot_by_user;
    uint16_t shoot_by_ai;
    uint8_t control_mode;//0-手动射击 1-保险关闭（可调整摩擦轮启停） 2-自动模式
    uint8_t key_middle;//仅针对能读取鼠标的相关外设需要实现（一般用于启停摩擦轮，可通过其他方式实现）
    uint8_t custom_key[14] ;//此为自定义按键，不同外设对应按键不同
}USER_REMOTE;


/* 函数声明 ------------------------------------------------------------------*/

void user_remote_init(USER_REMOTE* my_remote ,const controller_config tatol_controller_config);
void Mech_Operating_Config(void);//此函数必须放于滴答中断

/*变量初始化--------------------------------------------------------------------*/
extern USER_REMOTE user_device_remote[USER_REMOTE_NUM];
extern uint8_t user_device_remote_num;

#endif //DJI_A_BOARD_USER_REMOTE_H
