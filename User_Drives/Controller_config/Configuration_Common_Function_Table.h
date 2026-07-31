#ifndef DJI_A_BOARD_CONFIGURATION_COMMON_FUNCTION_TABLE_H
#define DJI_A_BOARD_CONFIGURATION_COMMON_FUNCTION_TABLE_H

/*全局变量-----------------------------------------------*/
//配置软件定时器
extern int user_time_counyer ;
//配置热量管理
extern float shoot_heat ;

/*函数声明-----------------------------------------------*/

int time_counyer(int time);
void shoot_heat_reset(float max_shoot_heat,int Reset_Interval,float Reset_num);
int shoot_heat_control();
int user_delay(int time);
float max_value(float max , float user_data);

#endif //DJI_A_BOARD_CONFIGURATION_COMMON_FUNCTION_TABLE_H
