#include "../../User_Drives/Controller_config/ZhouZishun.h"
#include "bsp.h"

void ZhouZishun_Config(USER_REMOTE* my_remote) {
  can_RX_callback(&user_can_2);
  //PICH轴控制
  DJI_Motor_Set_State(&PICH_GM6020,  (float)virtual_user_remote.pitch);


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
      DJI_Motor_Set_State(&RW_M3508, 6300);
      DJI_Motor_Set_State(&LW_M3508, -6300);

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

      if (virtual_user_remote.shoot_by_user) {
        //发射频率计时
        if (user_time_counyer % 100 == 0 && shoot_heat >=  10 ) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
          shoot_heat -= 10 ;
        }
      }
      if (user_holder_data.key_back == 1) {
        if (user_time_counyer % 1000 == 0) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) + 1296.0f));
        }
      }
    }

    if (user_holder_data.key_mode == 0){
      if (virtual_user_remote.shoot_by_user == 1 && user_holder_data.key_back == 1) {
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
  if (user_delay(49999)) {
    char angle_z[8] = {0};
    char angle_data_head[2] = { 0xEB , 0x90 };
    char angle_data_tail[2] = { 0x90 , 0xEB };
    angle_z[0] = angle_data_head[0];
    angle_z[1] = angle_data_head[1];
    const float inv_angle_z = user_holder_data.angle_z;
    angle_z[2] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 0);
    angle_z[3] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 8);
    angle_z[4] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 16);
    angle_z[5] = (uint8_t) ((*(uint32_t*)&inv_angle_z) >> 24);
    angle_z[6] = angle_data_tail[0];
    angle_z[7] = angle_data_tail[1];
    UART_Send(user_PC.user_uart, angle_z , 8);
  }
}