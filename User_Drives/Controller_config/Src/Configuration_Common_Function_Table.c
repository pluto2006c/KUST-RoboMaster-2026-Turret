/* 包含头文件 ----------------------------------------------------------------*/
#include "../../User_Drives/Controller_config/Configuration_Common_Function_Table.h"
#include "bsp.h"

/* 全局变量 ------------------------------------------------------------------*/
int user_time_counyer = 0 ;
float shoot_heat = 0 ;

/* 函数体 --------------------------------------------------------------------*/

/**
 * @brief 软件定时器计数判断
 * @param time 定时间隔
 * @return 1-到达定时间隔, 0-未到达
 */
int time_counyer(int time) {
    if (user_time_counyer % time != 0) {
        return false;
    }
    return true;

}

/**
 * @brief 用户延时判断
 * @param time 延时时间
 * @return 1-延时到达, 0-延时未到达
 */
int user_delay(const int time) {
    static uint8_t time_mode = 0;
    if (user_time_counyer >= time)
        time_mode = 1;
    if (time_mode == 0)
        return false;
    return true ;

}

/**
 * @brief 射击热量重置
 * @param max_shoot_heat   最大热量值
 * @param Reset_Interval   重置间隔
 * @param Reset_num        每次恢复的热量值
 */
void shoot_heat_reset(float max_shoot_heat,int Reset_Interval,float Reset_num){
    //热量恢复
    if (user_time_counyer % Reset_Interval == 0) {
        if (shoot_heat <= max_shoot_heat) {
            shoot_heat += Reset_num ;
        }else {
            shoot_heat = max_shoot_heat ;
        }
    }
}

/**
 * @brief 射击热量控制
 * @return 1-热量充足可射击, 0-热量不足
 */
int shoot_heat_control() {
    //热量判断
    if (shoot_heat <=0)
        return false;
    shoot_heat -= 10;
    return true;
}
