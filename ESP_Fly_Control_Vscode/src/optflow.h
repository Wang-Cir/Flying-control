#ifndef OPTFLOW_H
#define OPTFLOW_H

#include <stdint.h>
#include <stdbool.h>

// UART1 配置（光流传感器使用32、33引脚）
#define UART1_RX_PIN 33
#define UART1_TX_PIN 32
#define UART1_BAUD_RATE 115200

// 光流传感器数据结构
typedef struct {
    uint32_t distance;      // 距离(mm)，0表示数据不可用
    uint8_t strength;       // 信号强度
    uint8_t precision;      // 精度
    uint8_t tof_status;     // 状态
    int16_t flow_vel_x;     // 光流速度x轴 (cm/s@1m)
    int16_t flow_vel_y;     // 光流速度y轴 (cm/s@1m)
    uint8_t flow_quality;   // 光流质量
    uint8_t flow_status;    // 光流状态
    uint32_t timestamp;     // 时间戳
} optflow_data_t;

// 全局变量声明
extern optflow_data_t optflow_data;
extern bool optflow_data_ready;
extern uint32_t optflow_packet_count;
extern uint32_t optflow_error_count;
extern int32_t X_Speed,Y_Speed,Z_distant;
// 函数声明
void optflow_init(void);
void optflow_decode(uint8_t data);
void onUART1Data(void);
void RealSpeed(void);


#endif