#ifndef __USER_DJI_VTO3_H__
#define __USER_DJI_VTO3_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "user_uart.h"
#include "bsp_config.h"
#include <stdbool.h>

/* 常量定义 ------------------------------------------------------------------*/
#define DJI_VT03_BUFFLEN      (21)
#define DJI_VT03_SOF_1        (0xA9)
#define DJI_VT03_SOF_2        (0x53)
#define DJI_VT03_CH_OFFSET    (1024)

/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief 键盘按键枚举
 */
typedef enum {
    KEY_W     = 0,   // W键
    KEY_S     = 1,   // S键
    KEY_A     = 2,   // A键
    KEY_D     = 3,   // D键
    KEY_SHIFT = 4,   // Shift键
    KEY_CTRL  = 5,   // Ctrl键
    KEY_Q     = 6,   // Q键
    KEY_E     = 7,   // E键
    KEY_R     = 8,   // R键
    KEY_F     = 9,   // F键
    KEY_G     = 10,  // G键
    KEY_Z     = 11,  // Z键
    KEY_X     = 12,  // X键
    KEY_C     = 13,  // C键
    KEY_V     = 14,  // V键
    KEY_B     = 15,  // B键
} Keyboard;

/**
 * @brief DJI VT03遥控器数据结构
 */
typedef struct {
    UART_DRIVES* remote_uart;  // UART驱动指针
    
    // 帧头
    uint8_t sof_1;             // 起始字节1
    uint8_t sof_2;             // 起始字节2
    
    // 遥控器通道数据
    uint16_t ch0;              // 通道0 (右摇杆水平)
    uint16_t ch1;              // 通道1 (右摇杆垂直)
    uint16_t ch2;              // 通道2 (左摇杆水平)
    uint16_t ch3;              // 通道3 (左摇杆垂直)
    
    // 开关和按键
    uint16_t mode_sw;          // 模式开关
    uint16_t pause;            // 暂停开关
    uint8_t fn1;               // 功能键1
    uint8_t fn2;               // 功能键2
    uint16_t wheel;            // 滚轮值
    uint8_t trigger;           // 扳机
    
    // 鼠标数据
    int16_t mouse_x;           // 鼠标X轴移动
    int16_t mouse_y;           // 鼠标Y轴移动
    int16_t mouse_z;           // 鼠标滚轮
    uint8_t mouse_left;        // 鼠标左键
    uint8_t mouse_right;       // 鼠标右键
    uint8_t mouse_middle;      // 鼠标中键
    
    // 键盘数据
    uint16_t key_value;        // 键盘按键值(位掩码)
} VT03_DRIVES;

/* 函数声明 ------------------------------------------------------------------*/

// 初始化函数
void DJI_VT03_Init(VT03_DRIVES* vt03);

// 键盘查询函数
uint8_t VT03_IsKeyboardDown(const Keyboard keyboard);

// 数据验证函数
uint8_t VT03_VerifyCRC16(uint8_t* p_msg, uint16_t len);

#endif //__USER_DJI_VTO3_H__