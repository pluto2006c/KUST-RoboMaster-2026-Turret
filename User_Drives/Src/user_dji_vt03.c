/* 包含头文件 ----------------------------------------------------------------*/
#include "../Inc/user_dji_vt03.h"
#include <stdio.h>
#include <string.h>

/* 私有变量 ------------------------------------------------------------------*/
static VT03_DRIVES* vt03_drive = NULL;

/* CRC16查找表 */
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/* 私有函数声明 --------------------------------------------------------------*/
static uint16_t VT03_Parse11Bit(const uint8_t* data, int offset_bits);
static uint16_t VT03_GetCRC16(uint8_t* p_msg, uint16_t len, uint16_t crc16);
static void VT03_UartCallback(void* user_uart);

/* 函数体 --------------------------------------------------------------------*/

/**
 * @brief 从数据流中解析11位数据
 * @param data 数据缓冲区
 * @param offset_bits 位偏移量
 * @return 解析出的11位数据
 */
static uint16_t VT03_Parse11Bit(const uint8_t* data, int offset_bits) {
    const int byte_offset = offset_bits / 8;
    const int bit_offset = offset_bits % 8;
    uint32_t value = 0;

    // 读取11位(可能跨2或3字节)
    if (bit_offset <= 5) {
        // 完全在相邻两个字节内
        value = (data[byte_offset] << (3 + bit_offset)) | 
                (data[byte_offset + 1] >> (5 - bit_offset));
    } else {
        // 跨三个字节
        value = (data[byte_offset] << (3 + bit_offset)) |
                (data[byte_offset + 1] << (bit_offset - 5)) |
                (data[byte_offset + 2] >> (13 - bit_offset));
    }
    
    return value & 0x7FF;  // 取低11位
}

/**
 * @brief 检查指定键盘按键是否按下
 * @param keyboard 键盘按键枚举
 * @return 1-按下, 0-未按下
 */
uint8_t VT03_IsKeyboardDown(const Keyboard keyboard) {
    if (vt03_drive == NULL) {
        return 0;
    }
    return (uint8_t)((vt03_drive->key_value & (1 << keyboard)) != 0);
}

/**
 * @brief 计算CRC16校验和
 * @param p_msg 待校验数据
 * @param len 数据长度
 * @param crc16 初始CRC16值
 * @return CRC16校验和
 */
static uint16_t VT03_GetCRC16(uint8_t* p_msg, uint16_t len, uint16_t crc16) {
    uint8_t data;

    if (p_msg == NULL) {
        return 0xFFFF;
    }

    while (len--) {
        data = *p_msg++;
        crc16 = ((uint16_t)(crc16) >> 8) ^ 
                crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00FF];
    }

    return crc16;
}
/**
 * @brief 验证CRC16校验和
 * @param p_msg 待验证数据(包含校验和)
 * @param len 数据流长度(数据+校验和)
 * @return 1-校验通过, 0-校验失败
 */
uint8_t VT03_VerifyCRC16(uint8_t* p_msg, uint16_t len) {
    uint16_t w_expected = 0;

    if ((p_msg == NULL) || (len <= 2)) {
        return 0;
    }

    const uint16_t crc16_init = 0xFFFF;
    w_expected = VT03_GetCRC16(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xFF) == p_msg[len - 2] && 
            ((w_expected >> 8) & 0xFF) == p_msg[len - 1]);
}

/**
 * @brief UART接收回调函数
 * @param user_uart UART驱动指针
 */
static void VT03_UartCallback(void* user_uart) {
    UART_DRIVES* uart = (UART_DRIVES*)user_uart;

    uint8_t buf[DJI_VT03_BUFFLEN] = {0};
    const char head[3] = {DJI_VT03_SOF_1, DJI_VT03_SOF_2, 0x00};
    
    // 从环形缓冲区获取数据
    if (!RBuffer_GetWithHLen(&uart->rx_ringBuffer, buf, head, 
                             RBuffer_GetLength(&uart->rx_ringBuffer))) {
        return;
    }

    // 验证CRC16校验和
    if (!VT03_VerifyCRC16(buf, DJI_VT03_BUFFLEN)) {
        return;
    }

    // 解析遥控器通道数据
    vt03_drive->ch0 = VT03_Parse11Bit(buf, 16) - DJI_VT03_CH_OFFSET;
    vt03_drive->ch1 = VT03_Parse11Bit(buf, 27) - DJI_VT03_CH_OFFSET;
    vt03_drive->ch2 = VT03_Parse11Bit(buf, 38) - DJI_VT03_CH_OFFSET;
    vt03_drive->ch3 = VT03_Parse11Bit(buf, 49) - DJI_VT03_CH_OFFSET;

    // 解析开关和按键数据
    const int byte_offset = 60 / 8;
    const int bit_offset = 60 % 8;

    vt03_drive->mode_sw = (buf[byte_offset] >> (6 - bit_offset)) & 0x03;
    vt03_drive->pause = (buf[byte_offset] >> (5 - bit_offset)) & 0x01;
    vt03_drive->fn1 = (buf[byte_offset] >> (4 - bit_offset)) & 0x01;
    vt03_drive->fn2 = (buf[byte_offset + 1] >> (11 - bit_offset)) & 0x01;
    vt03_drive->wheel = VT03_Parse11Bit(buf, 65) - DJI_VT03_CH_OFFSET;
    
    // 解析键盘数据
    vt03_drive->key_value = (uint16_t)(buf[17] >> 0 | buf[18] << 8);
}

/**
 * @brief 初始化DJI VT03遥控器
 * @param vt03 VT03驱动结构体指针
 */
void DJI_VT03_Init(VT03_DRIVES* vt03) {
    if (vt03 == NULL) {
        return;
    }
    
    vt03_drive = vt03;
    UART_Init(vt03->remote_uart, &huart3, VT03_UartCallback);
}