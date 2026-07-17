/*包含头文件-------------------------------------------------------------------*/
#include "../../User_Drives/User_remote/user_remote.h"
/* 私有变量 ------------------------------------------------------------------*/

/*全局变量 -------------------------------------------------------------------*/

USER_REMOTE user_device_remote[USER_REMOTE_NUM]={0};
uint8_t user_device_remote_num = 0;

/* 私有函数声明 --------------------------------------------------------------*/

static USER_REMOTE* user_remote = NULL;

/* 函数体 --------------------------------------------------------------------*/
static float max_data(float max , float user_data) {
    if (user_data >= max) {
        user_data = max;
    } else if (user_data <= -max) {
        user_data = -max;
    }
    return user_data;
}

static void data_fusion(void) {
    user_remote->chassis_x = 0;
    user_remote->chassis_y = 0;
    user_remote->yaw = 0;
    user_remote->pitch = 0;
    user_remote->wheel = 0;
    user_remote->shoot_by_user = 0;
    user_remote->shoot_by_ai = 0;
    user_remote->control_mode = 0;
    user_remote->key_middle = 0;
    for (uint8_t key_num = 0; key_num < 14; key_num++) {
        user_remote->custom_key[key_num] = 0;
    }

    for (uint8_t remote_num = 0; remote_num < user_device_remote_num; remote_num++) {
        user_remote->chassis_x += user_device_remote[remote_num].chassis_x;
        user_remote->chassis_y += user_device_remote[remote_num].chassis_y;
        user_remote->yaw += user_device_remote[remote_num].yaw;
        user_remote->pitch += user_device_remote[remote_num].pitch;
        user_remote->wheel += user_device_remote[remote_num].wheel;
        user_remote->shoot_by_user += user_device_remote[remote_num].shoot_by_user;
        user_remote->shoot_by_ai += user_device_remote[remote_num].shoot_by_ai;
        user_remote->control_mode = user_device_remote[remote_num].control_mode;
        user_remote->key_middle += user_device_remote[remote_num].key_middle;

        for (uint8_t key_num = 0; key_num < 14; key_num++) {
            user_remote->custom_key[key_num] += user_device_remote[remote_num].custom_key[key_num];
        }

        user_remote->chassis_x = max_data(660.0f, user_remote->chassis_x);
        user_remote->chassis_y = max_data(660.0f, user_remote->chassis_y);
        user_remote->yaw       = max_data(660.0f, user_remote->yaw);
        user_remote->pitch     = max_data(660.0f, user_remote->pitch);
        user_remote->wheel     = max_data(660.0f, user_remote->wheel);
        user_remote->shoot_by_user = max_data(1.0f, user_remote->shoot_by_user);
        user_remote->shoot_by_ai   = max_data(1.0f, user_remote->shoot_by_ai);
        user_remote->key_middle    = max_data(1.0f, user_remote->key_middle);

        for (uint8_t key_num = 0; key_num < 14; key_num++) {
            user_remote->custom_key[key_num] = max_data(1.0f, user_remote->custom_key[key_num]);
        }
    }
}

void user_remote_init(USER_REMOTE* my_remote ,const controller_config tatol_controller_config) {
    my_remote->remote_config = tatol_controller_config;
    //配置软件定时器
    user_time_counyer = 0 ;
    //配置热量管理
    shoot_heat = 0 ;
    user_remote = my_remote;
}

void Mech_Operating_Config(void) {

    //软件计时器
    if (user_time_counyer <= 1000000) {
        user_time_counyer ++ ;
    }else {
        user_time_counyer = 0 ;
    }
    data_fusion();
    //配置执行函数
    user_remote->remote_config();
}