/*包含头文件-------------------------------------------------------------------*/
#include "../../User_Drives/User_remote/user_remote.h"
/* 私有变量 ------------------------------------------------------------------*/

/*全局变量 -------------------------------------------------------------------*/

USER_REMOTE user_device_remote[USER_REMOTE_NUM]={0};
uint8_t user_device_remote_num = 0;

/* 私有函数声明 --------------------------------------------------------------*/

static USER_REMOTE* user_remote = NULL;

/* 函数体 --------------------------------------------------------------------*/

/**
 * @brief 数据限幅函数
 * @param max       最大值
 * @param user_data 待限幅数据
 * @return 限幅后的数据
 */
static float max_data(float max , float user_data) {
    if (user_data >= max) {
        user_data = max;
    } else if (user_data <= -max) {
        user_data = -max;
    }
    return user_data;
}

/**
 * @brief 遍历所有遥控器，对指定字段累加并限幅
 * @param first     首地址
 * @param count     遥控器数量
 * @param stride    结构体大小（步长）
 * @param elem_size 元素大小（1 或 2）
 * @param max       限幅最大值
 */
static int16_t remote_data_sum(const void *first, uint8_t count,uint8_t stride, uint8_t elem_size, float max) {
    int16_t total = 0;
    const uint8_t *p = (const uint8_t *)first;
    for (uint8_t i = 0; i < count; i++) {
        int16_t val = (elem_size == 1) ? (int16_t)(*p) : *(const int16_t *)p;
        total = (int16_t)max_data(max, (float)(total + val));
        p += stride;
    }
    return total;
}

#define REMOTE_SUM(member, max_val) \
remote_data_sum(&user_device_remote[0].member, user_device_remote_num, \
sizeof(USER_REMOTE), sizeof(user_device_remote[0].member), max_val)

static void data_fusion(void) {
    user_remote->chassis_x     = REMOTE_SUM(chassis_x,     660.0f);
    user_remote->chassis_y     = REMOTE_SUM(chassis_y,     660.0f);
    user_remote->yaw           = REMOTE_SUM(yaw,           660.0f);
    user_remote->pitch         = REMOTE_SUM(pitch,         660.0f);
    user_remote->wheel         = REMOTE_SUM(wheel,         660.0f);
    user_remote->shoot_by_user = REMOTE_SUM(shoot_by_user, 1.0f);
    user_remote->shoot_by_ai   = REMOTE_SUM(shoot_by_ai,   1.0f);
    user_remote->key_middle    = REMOTE_SUM(key_middle,    1.0f);
    user_remote->control_mode  = user_device_remote[0].control_mode;

    for (uint8_t key_num = 0; key_num < 14; key_num++) {
        user_remote->custom_key[key_num] = REMOTE_SUM(custom_key[key_num], 1.0f);
    }
}

/**
 * @brief 初始化用户遥控器
 * @param my_remote               用户遥控器结构体指针
 * @param tatol_controller_config  控制器配置函数
 */
void user_remote_init(USER_REMOTE* my_remote ,const controller_config tatol_controller_config) {
    my_remote->remote_config = tatol_controller_config;
    //配置软件定时器
    user_time_counyer = 0 ;
    //配置热量管理
    shoot_heat = 0 ;
    user_remote = my_remote;
}

/**
 * @brief 机械操作配置函数
 * @note  该函数应在滴答中断中调用，负责软件计时和数据融合
 */
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