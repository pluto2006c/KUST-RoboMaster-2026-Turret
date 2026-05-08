/*包含头文件-------------------------------------------------------------------*/
#include "../../User_Drives/User_remote/user_remote.h"
/* 私有变量 ------------------------------------------------------------------*/

/*全局变量 -------------------------------------------------------------------*/

USER_REMOTE user_device_remote[USER_REMOTE_NUM]={0};
uint8_t user_device_remote_num = 0;

/* 私有函数声明 --------------------------------------------------------------*/

static USER_REMOTE* virtual_user_remote = NULL;

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
    for (uint8_t remote_num = 0; remote_num < user_device_remote_num; remote_num++) {
        virtual_user_remote->chassis_x += user_device_remote[remote_num].chassis_x;
        virtual_user_remote->chassis_y += user_device_remote[remote_num].chassis_y;
        virtual_user_remote->yaw += user_device_remote[remote_num].yaw;
        virtual_user_remote->pitch += user_device_remote[remote_num].pitch;
        virtual_user_remote->wheel += user_device_remote[remote_num].wheel;
        virtual_user_remote->shoot_by_user += user_device_remote[remote_num].shoot_by_user;
        virtual_user_remote->shoot_by_ai += user_device_remote[remote_num].shoot_by_ai;
        virtual_user_remote->control_mode = user_device_remote[remote_num].control_mode;
        virtual_user_remote->key_middle += user_device_remote[remote_num].key_middle;

        for (uint8_t key_num = 0; key_num < 14; key_num++) {
            virtual_user_remote->custom_key[key_num] += user_device_remote->custom_key[key_num];
        }

        virtual_user_remote->chassis_x = max_data(660.0f, virtual_user_remote->chassis_x);
        virtual_user_remote->chassis_y = max_data(660.0f, virtual_user_remote->chassis_y);
        virtual_user_remote->yaw       = max_data(660.0f, virtual_user_remote->yaw);
        virtual_user_remote->pitch     = max_data(660.0f, virtual_user_remote->pitch);
        virtual_user_remote->wheel     = max_data(660.0f, virtual_user_remote->wheel);
        virtual_user_remote->shoot_by_user = max_data(1.0f, virtual_user_remote->shoot_by_user);
        virtual_user_remote->shoot_by_ai   = max_data(1.0f, virtual_user_remote->shoot_by_ai);
        virtual_user_remote->key_middle    = max_data(1.0f, virtual_user_remote->key_middle);

        for (uint8_t key_num = 0; key_num < 14; key_num++) {
            virtual_user_remote->custom_key[key_num] = max_data(1.0f, virtual_user_remote->custom_key[key_num]);
        }
    }
}

void user_remote_init(USER_REMOTE* my_remote ,const controller_config tatol_controller_config) {
    my_remote->remote_config = tatol_controller_config;
    //配置软件定时器
    user_time_counyer = 0 ;
    //配置热量管理
    shoot_heat = 0 ;
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
    virtual_user_remote->remote_config(virtual_user_remote);
}