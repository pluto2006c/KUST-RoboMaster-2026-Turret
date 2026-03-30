/*头文件包含------------------------------------------------------------------------*/
#include "../../User_Application/Holder_Data_Processing.h"

/*私有变量---------------------------------------------------------------------------*/


static Holder_Data *user_holder = NULL;  /* 用户数据结构体指针 */

/*函数实现----------------------------------------------------------------------------*/

/**
 * @brief 限制数据范围
 * @param max       最大值
 * @param user_data 用户数据
 * @return 限制后的数据
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
 * @brief 用户数据处理
 * @param user_holder 用户数据结构体指针
 * @param user_VT03   VT03 遥控器驱动结构体指针
 * @param user_HWT906 HWT906 传感器驱动结构体指针
 * @param user_PC     PC 通信驱动结构体指针
 */
void user_data_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC) {
    static uint8_t keyboard_shoot_mode = 0 ; /* 键盘射击模式 */
    static float old_angle_z = 0;
    static uint8_t get_angel_mode = 0 ; /* 获取角度模式 */



    /* 按 R 键切换射击模式 */
    if (user_holder->user_time_flash % 1000 == 0 && VT03_IsKeyboardDown(KEY_R) ==1 ) {
        keyboard_shoot_mode ++ ;
        if (keyboard_shoot_mode > 1) {
            keyboard_shoot_mode = 0 ;
        }
    }

    /* 加速度解析 */
    user_holder->user_value = sqrt(user_holder->value_x*user_holder->value_x + user_holder->value_y*user_holder->value_y);
    static float value_data = 1;
    value_data = 1 + user_holder->user_value;
    user_holder->anac.p = 1000;
    user_holder->anac.m = 50;
    user_holder->anac.a = user_holder->anac.p/(user_holder->anac.m + value_data);
    if (user_holder->anac.a <1)
        user_holder->anac.a = 1;

    /* 角度处理 */
    if (get_angel_mode == 0 && user_HWT906->user_angle.angle_z != 0) {
        old_angle_z= user_HWT906->user_angle.angle_z;
        get_angel_mode = 1 ;
    }
    float angle_z =user_HWT906->user_angle.angle_z;
    float angle_z_diff = angle_z - old_angle_z;
    if (angle_z_diff > 180) {
        angle_z_diff -= 360;
    } else if (angle_z_diff < -180) {
        angle_z_diff += 360;
    }
    old_angle_z = angle_z;
    user_holder->angle_z += angle_z_diff;
    if (user_holder->angle_z > 180) {
        user_holder->angle_z -= 360;
    } else if (user_holder->angle_z < -180) {
        user_holder->angle_z += 360;
    }


    /* 遥控器数据处理 */
    user_holder->key_mode = user_VT03->mode_sw + keyboard_shoot_mode;
    if (user_VT03->mouse_left == 1 || user_VT03->mouse_right == 1) {
        if (user_VT03->mouse_left ==0 && user_VT03->mouse_right == 1) {
            user_holder->key_mode = 2; /* 右键模式 */
        }else if (user_VT03->mouse_left == 1 && user_VT03->mouse_right == 0){
            user_holder->key_mode = 1; /* 左键模式 */
        }
    }

    user_holder->key_left = max_data(1 , user_VT03->fn1 + VT03_IsKeyboardDown(KEY_Q));
    user_holder->key_right = max_data(1 ,user_VT03->fn2 + VT03_IsKeyboardDown(KEY_E));
    user_holder->key_shoot = max_data(1 ,user_VT03->trigger + user_VT03->mouse_left + user_VT03->mouse_right);
    user_holder->key_back  = max_data(1 ,user_holder->key_right = user_VT03->fn2);

    if (user_holder->key_mode == 2) { /* PC 控制模式 */
        user_holder->holder_pitch = max_data(660 ,user_VT03->ch1 + user_PC->holder_pitch + user_VT03->mouse_y );
        user_holder->d_theta_turret = max_data(660 , user_VT03->ch0 + user_PC->holder_yaw   + user_VT03->mouse_x );
    }else if(user_holder->key_mode == 1 || user_holder->key_mode == 0){ /* 遥控器控制模式 */
        user_holder->holder_pitch = max_data(660 ,user_VT03->ch1 + user_VT03->mouse_y ) ;
        user_holder->d_theta_turret = max_data(660 , user_VT03->ch0 + user_VT03->mouse_x );
    }

    user_holder->v_x =user_VT03->ch2 - VT03_IsKeyboardDown(KEY_S)*660 + VT03_IsKeyboardDown(KEY_W)*660;
    user_holder->v_y =user_VT03->ch3 - VT03_IsKeyboardDown(KEY_A)*660 + VT03_IsKeyboardDown(KEY_D)*660;

    if (user_holder->key_left != 0 && user_holder->user_time_flash % 1000 == 0) {
        user_holder->w_theta_chassis = user_VT03->wheel + user_VT03->mouse_z;
    }

    user_holder->pitch_angle = max_data(75 ,user_holder->pitch_angle + 0.3f*0.0008f*user_holder->holder_pitch);

    /* 速度限制 */
    if (VT03_IsKeyboardDown(KEY_SHIFT)) {
        user_holder->value_max = 220;
    }else if (VT03_IsKeyboardDown(KEY_CTRL) || user_VT03->pause == 1) {
        user_holder->value_max = 660;
    }else {
        user_holder->value_max = 440;
    }

    if (user_holder->user_time_flash % 10 == 0 ) {
        /* X轴加速逻辑 */
        if (user_holder->v_x > 0 ) {
            user_holder->value_x += user_holder->anac.a;
        }else if (user_holder->v_x < 0 ){
            user_holder->value_x -= user_holder->anac.a;
        }

        /* X轴减速逻辑 */
        if (user_holder->value_x >0 && user_holder->v_x == 0) {
            user_holder->value_x -=  user_holder->anac.a;
        }else if (user_holder->value_x <0 && user_holder->v_x == 0) {
            user_holder->value_x +=  user_holder->anac.a;
        }else if(user_holder->v_x == 0 && user_holder->user_value <= 20) {
            user_holder->value_x = 0;
        }

        /* Y轴加速逻辑 */
        if (user_holder->v_y >0 ) {
            user_holder->value_y += user_holder->anac.a;
        }else if (user_holder->v_y < 0 ) {
            user_holder->value_y -= user_holder->anac.a;
        }

        /* Y轴减速逻辑 */
        if (user_holder->value_y >0 && user_holder->v_y == 0) {
            user_holder->value_y -=  user_holder->anac.a;
        }else if (user_holder->value_y <0 && user_holder->v_y == 0) {
            user_holder->value_y +=  user_holder->anac.a;
        }else if(user_holder->v_y == 0 && user_holder->user_value <= 20) {
            user_holder->value_y = 0;
        }
    }

    user_holder->value_y = max_data(user_holder->value_max , user_holder->value_y);
    user_holder->value_x = max_data(user_holder->value_max , user_holder->value_x);
}