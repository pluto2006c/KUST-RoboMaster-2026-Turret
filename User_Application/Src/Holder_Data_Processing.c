/*头文件包含------------------------------------------------------------------------*/
#include "../../User_Application/Holder_Data_Processing.h"
/*函数声明--------------------------------------------------------------------------*/

void angle_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC);

/*私有变量---------------------------------------------------------------------------*/
static int16_t old_angle_z = 0;

/*函数实现----------------------------------------------------------------------------*/
void angle_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC){
    //角度处理
    const float angle_z = (float) user_HWT906->user_angle.angle_z / 100.0f;
    float angle_z_diff = angle_z - old_angle_z;

    if (angle_z_diff > 180.0f) {
        angle_z_diff -= 180.0f;
    } else if (angle_z_diff < -180.0f) {
        angle_z_diff += 180.0f;
    }
    user_holder->angle_z += angle_z_diff;
    //遥控器数据处理
    user_holder->key_shoot = user_VT03->trigger + user_VT03->mouse_left;
    user_holder->key_back  = user_holder->key_left = user_VT03->fn2 + VT03_IsKeyboardDown(KEY_A) + user_VT03->mouse_right;
    if (user_holder->key_mode ==0) {
        user_holder->holder_pitch = user_VT03->ch0 + user_PC->holder_pitch + user_VT03->mouse_x *360/32768;
        user_holder->holder_yaw   = user_VT03->ch1 + user_PC->holder_yaw   + user_VT03->mouse_y *360/32768;
    }else {
        user_holder->holder_pitch = user_VT03->ch0 + user_VT03->mouse_x *360/32768;
        user_holder->holder_yaw   = user_VT03->ch1 + user_VT03->mouse_y *360/32768;
    }
    user_holder->v_x = user_VT03->ch2 - VT03_IsKeyboardDown(KEY_A)*660 + VT03_IsKeyboardDown(KEY_D)*660;
    user_holder->v_y = user_VT03->ch3 - VT03_IsKeyboardDown(KEY_S)*660 + VT03_IsKeyboardDown(KEY_W)*660;
    if (user_holder->wheel ==1) {
        user_holder->w_theta_chassis = user_VT03->wheel + user_VT03->mouse_z *360/32768;
    }

    if (user_holder->holder_pitch >= 660) {
        user_holder->holder_pitch = 660;
    } else if (user_holder->holder_pitch <= -660) {
        user_holder->holder_pitch = -660;
    }




}