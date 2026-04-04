#include "../../User_Drives/Controller_config/ZhouZishun.h"
#include "bsp.h"

void ZhouZishun_Config(USER_REMOTE* my_remote) {
  can_RX_callback(&user_can_2);
  user_data_processing(&user_holder_data ,&user_vt03,NULL,&user_PC );//数据处理
  //PICH轴控制
  DJI_Motor_Set_State(&PICH_GM6020,  (float)user_holder_data.pitch_angle);
  //计时器
  static int user_time_counyer = 0 ;
  static int user_shoot_flash = 0;
  static uint8_t time_flash = 0 ;
  user_holder_data.user_time_flash = user_time_counyer;

  if (user_time_counyer <= 1000000) {
    user_time_counyer ++ ;
  }else {
    user_time_counyer = 0 ;
  }

  if (user_time_counyer > 49999) {
    user_shoot_flash = 1 ;
  }


  //热量管理
  static float shoot_heat = 0 ;
  if (user_time_counyer % 100 == 0) {
    if (shoot_heat <= 78.2) {
      shoot_heat += 1.2f ;
    }else {
      shoot_heat = 80 ;
    }
  }

  //模式控制
  static uint8_t shoot_mode = 0 ;
  static uint8_t ai_shoot_mode = 0 ;

  if (shoot_mode == 0 || shoot_mode == 1) {
    if (user_time_counyer % 10 == 0) {
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
    if (user_holder_data.key_mode == 1 || user_holder_data.key_mode == 2){
      DJI_Motor_Set_State(&RW_M3508, 6300);
      DJI_Motor_Set_State(&LW_M3508, -6300);

      //全自动连发
      if (user_holder_data.key_mode == 2  && shoot_heat >= 10) {
        if (user_PC.shoot_delay != 0xFFFF && ai_shoot_mode == 0) {
          time_flash = user_time_counyer;
          ai_shoot_mode = 1 ;
        }
        if (ai_shoot_mode == 1 && user_time_counyer - time_flash >= user_PC.shoot_delay) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
          shoot_heat -= 10 ;
          ai_shoot_mode = 0 ;
        }
      }

      if (user_holder_data.key_shoot == 1 ) {
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
      if (user_holder_data.key_shoot == 1 && user_holder_data.key_back == 1) {
        DJI_Motor_Set_State(&RW_M3508, 2000);
        DJI_Motor_Set_State(&LW_M3508, -2000);
        if (user_time_counyer % 50 == 0 ) {
          DJI_Motor_Set_State(&TP_M2006,(float)(DJI_Motor_Get_Angle(&TP_M2006) - 1296.0f));
        }
      }else {
        DJI_Motor_Set_State(&RW_M3508, 0);
        DJI_Motor_Set_State(&LW_M3508, 0);
      }
    }
  }

  DJI_Motor_Execute(&user_can_1);

  if (user_shoot_flash == 1) {
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