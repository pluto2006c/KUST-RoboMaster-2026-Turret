#include "../../User_Drives/Controller_config/Configuration_Common_Function_Table.h"
#include "bsp.h"

int user_time_counyer = 0 ;
float shoot_heat = 0 ;

int time_counyer(int time) {
    if (user_time_counyer % time != 0) {
        return false;
    }
    return true;

}

int user_delay(const int time) {
    static uint8_t time_mode = 0;
    if (user_time_counyer >= time)
        time_mode = 1;
    if (time_mode == 0)
        return false;
    return true ;

}

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

int shoot_heat_control() {
    //热量判断
    if (shoot_heat <=0)
        return false;
    shoot_heat -= 10;
    return true;
}
