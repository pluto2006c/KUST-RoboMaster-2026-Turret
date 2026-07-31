/* 包含头文件 ----------------------------------------------------------------*/
#include "../../User_Drives/Controller_config/ZhouZishun.h"
#include "bsp.h"

/* 函数体 --------------------------------------------------------------------*/

/**
 * @brief 周子顺控制器配置
 * @note  该函数负责处理云台控制、发射机构控制和热量管理
 */
void ZhouZishun_Config(void) {

  can_RX_callback(&user_can_2);
  //PICH轴控制
  static float pitch_angle = 0;
  pitch_angle += 0.0005*(float)virtual_user_remote.pitch;
  pitch_angle = max_value(75.0f , pitch_angle);
  DJI_Motor_Set_State(&PICH_GM6020,  pitch_angle);
  shoot_heat_reset(80,100,10);


  //模式控制
  static uint8_t shoot_mode = 0 ;

  if (shoot_mode == 0 || shoot_mode == 1) {
    if (time_counyer(10)) {
      DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) + 81.91));
    }
    if (DJI_Motor_Get_Speed(&TP_M2006) > 50){
      shoot_mode = 1 ;
    }
  }

  if (shoot_mode == 1 && DJI_Motor_Get_Speed(&TP_M2006) < 5) {
    shoot_mode = 2 ;
  }

  if (shoot_mode == 2) {
    DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1152.0f));
    shoot_mode = 3 ;
  }

  //发射机构控制
  if (shoot_mode == 3) {
    //连发
    if (virtual_user_remote.control_mode == 1 || virtual_user_remote.control_mode == 2){
      DJI_Motor_Set_State(&RW_M3508, 16000);
      DJI_Motor_Set_State(&LW_M3508, -16000);

      //全自动连发
      if (virtual_user_remote.control_mode == 2  && shoot_heat_control()) {
        if (user_PC.shoot_delay != 0xFFFF && virtual_user_remote.shoot_by_ai == 0) {
          virtual_user_remote.shoot_by_ai = 1 ;
        }
        if (virtual_user_remote.shoot_by_ai == 1 && user_delay(user_PC.shoot_delay) ) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
          virtual_user_remote.shoot_by_ai = 0 ;
        }
      }

      if (virtual_user_remote.shoot_by_user == 1) {
        //发射频率计时
        if (user_time_counyer % 100 == 0 && shoot_heat_control() ) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
          shoot_heat -= 10 ;
        }
      }
      if (virtual_user_remote.custom_key[2] == 1) {
        if (user_time_counyer % 1000 == 0) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) + 1296.0f));
        }
      }
    }

    if (virtual_user_remote.control_mode == 0){
      if (virtual_user_remote.shoot_by_user == 1 && virtual_user_remote.custom_key[2] == 1) {
        DJI_Motor_Set_State(&RW_M3508, 2000);
        DJI_Motor_Set_State(&LW_M3508, -2000);
        if (time_counyer(50)) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
        }
      }else {
        DJI_Motor_Set_State(&RW_M3508, 0);
        DJI_Motor_Set_State(&LW_M3508, 0);
      }
    }
  }

  DJI_Motor_Execute(&user_can_1);

  static int16_t chassis_w = 0;

  if (virtual_user_remote.custom_key[0] == 1) {
    chassis_w = virtual_user_remote.wheel;
  }

  if (user_time_counyer % 2) {
    uint8_t user_can_2_send_frame_1[8] = {0};

    user_can_2_send_frame_1 [0] = (uint8_t) (chassis_w >> 0);
    user_can_2_send_frame_1 [1] = (uint8_t) (chassis_w >> 8);
    user_can_2_send_frame_1 [2] = (uint8_t) (virtual_user_remote.yaw  >> 0);
    user_can_2_send_frame_1 [3] = (uint8_t) (virtual_user_remote.yaw  >> 8);
    user_can_2_send_frame_1 [4] = (uint8_t) (virtual_user_remote.chassis_x >> 0);
    user_can_2_send_frame_1 [5] = (uint8_t) (virtual_user_remote.chassis_x >> 8);
    user_can_2_send_frame_1 [6] = (uint8_t) (virtual_user_remote.chassis_y >> 0);
    user_can_2_send_frame_1 [7] = (uint8_t) (virtual_user_remote.chassis_y >> 8);

    CAN_Send(&user_can_2, Chassis_data_ID_1 , user_can_2_send_frame_1, 8);

    uint8_t user_can_2_send_frame_2[8] = {0};

    user_can_2_send_frame_2 [0] = (uint8_t) (chassis_w >> 0);
    user_can_2_send_frame_2 [1] = (uint8_t) (chassis_w >> 8);
    user_can_2_send_frame_2 [2] = (uint8_t) (virtual_user_remote.yaw  >> 0);
    user_can_2_send_frame_2 [3] = (uint8_t) (virtual_user_remote.yaw  >> 8);
    user_can_2_send_frame_2 [4] = (uint8_t) ((int16_t)((float)virtual_user_remote.chassis_x/660*1200) >> 0);
    user_can_2_send_frame_2 [5] = (uint8_t) ((int16_t)((float)virtual_user_remote.chassis_x/660*1200) >> 8);
    user_can_2_send_frame_2 [6] = (uint8_t) ((int16_t)((float)virtual_user_remote.chassis_y/660*1200) >> 0);
    user_can_2_send_frame_2 [7] = (uint8_t) ((int16_t)((float)virtual_user_remote.chassis_y/660*1200) >> 8);

    CAN_Send(&user_can_2, Chassis_data_ID_2 , user_can_2_send_frame_2, 8);

    virtual_user_remote.yaw = 0 ;

    uint8_t user_can_2_send_frame_3[8] = {0};

    user_can_2_send_frame_3 [0] = (uint8_t) ((uint16_t)((float)user_HWT906_chassis.user_angle.angle_z * 100.0f) >> 0);
    user_can_2_send_frame_3 [1] = (uint8_t) ((uint16_t)((float)user_HWT906_chassis.user_angle.angle_z * 100.0f) >> 8);
    user_can_2_send_frame_3 [2] = 0;
    user_can_2_send_frame_3 [3] = 0;
    user_can_2_send_frame_3 [4] = 0;
    user_can_2_send_frame_3 [5] = 0;
    user_can_2_send_frame_3 [6] = 0;
    user_can_2_send_frame_3 [7] = 0;

    CAN_Send(&user_can_2, Chassis_data_ID_3 , user_can_2_send_frame_3, 8);
  }

  if (user_delay(49999)) {
    char angle_z[8] = {0};
    char angle_data_head[2] = { 0xEB , 0x90 };
    char angle_data_tail[2] = { 0x90 , 0xEB };
    angle_z[0] = angle_data_head[0];
    angle_z[1] = angle_data_head[1];
    const float inv_angle_z = user_HWT906_chassis.user_angle.angle_z;
    angle_z[2] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 0);
    angle_z[3] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 8);
    angle_z[4] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 16);
    angle_z[5] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 24);
    angle_z[6] = angle_data_tail[0];
    angle_z[7] = angle_data_tail[1];
    UART_Send(user_PC.user_uart, angle_z , 8);
  }
}