/*头文件包含------------------------------------------------------------------------*/
#include "../../User_Application/Holder_Data_Processing.h"
/*函数声明--------------------------------------------------------------------------*/

void angle_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC);

/*私有变量---------------------------------------------------------------------------*/
static float old_angle_z = 0;
static Holder_Data *user_holder = NULL;

/*函数实现----------------------------------------------------------------------------*/
void angle_processing(Holder_Data* user_holder , VT03_DRIVES* user_VT03, HWT906_DRIVES* user_HWT906 , PC_DRIVES* user_PC){
    //角度处理
    const float angle_z = (float) user_HWT906->user_angle.angle_z / 100.0f;
    float angle_z_diff = angle_z - old_angle_z;
    if (angle_z_diff > 180) {
        angle_z_diff -= 360;
    } else if (angle_z_diff < -180) {
        angle_z_diff += 360;
    }
    old_angle_z = angle_z;
    user_holder->angle_z += angle_z_diff;
    //遥控器数据处理
    user_holder->key_mode = user_VT03->mode_sw;
    if (user_VT03->mouse_left == 1 || user_VT03->mouse_middle == 1) {
        if (user_VT03->mouse_left ==1 && user_VT03->mouse_right == 0) {
            user_holder->key_mode = 2;
        }else if (user_VT03->mouse_left == 0 && user_VT03->mouse_right == 1){
            user_holder->key_mode = 1;
        }
    }
    user_holder->key_left = user_VT03->fn1 + VT03_IsKeyboardDown(KEY_Q);
    user_holder->key_right = user_VT03->fn2 + VT03_IsKeyboardDown(KEY_E);
    user_holder->key_shoot = user_VT03->trigger + user_VT03->mouse_left + user_VT03->mouse_right;
    user_holder->key_back  = user_holder->key_right = user_VT03->fn2   + user_VT03 ->mouse_middle;
    if (user_holder->key_mode == 2) {
        user_holder->holder_pitch = user_VT03->ch1 + user_PC->holder_pitch + user_VT03->mouse_x *360/32768;
        user_holder->holder_yaw   = user_VT03->ch0 + user_PC->holder_yaw   + user_VT03->mouse_y *360/32768;
        user_holder->d_theta_turret = user_VT03->ch0 + user_PC->holder_yaw   + user_VT03->mouse_y *360/32768;
    }else {
        user_holder->holder_pitch = user_VT03->ch1 + user_VT03->mouse_x *360/32768;
        user_holder->holder_yaw   = user_VT03->ch0 + user_VT03->mouse_y *360/32768;
        user_holder->d_theta_turret = user_VT03->ch0 + user_VT03->mouse_y *360/32768;
    }
    user_holder->v_x = user_VT03->ch2 - VT03_IsKeyboardDown(KEY_S)*660 + VT03_IsKeyboardDown(KEY_W)*660;
    user_holder->v_y = user_VT03->ch3 - VT03_IsKeyboardDown(KEY_A)*660 + VT03_IsKeyboardDown(KEY_D)*660;
    if (user_holder->key_left != 0) {
        user_holder->w_theta_chassis = user_VT03->wheel + user_VT03->mouse_z *360/32768;
    }


    user_holder->pitch_angle -= 0.3f*0.0008f*user_holder->holder_pitch;

    if (user_holder->pitch_angle >= 75) {
        user_holder->pitch_angle = 75;
    } else if (user_holder->pitch_angle <= -75) {
        user_holder->pitch_angle = -75;
    }
}