#include "optflow.h"
#include "HardwareSerial.h"
#include "esp32-hal-uart.h"

// UART1 硬件串口对象
HardwareSerial SerialPort1(1);

// 全局变量定义
optflow_data_t optflow_data = {0};
bool optflow_data_ready = false;
uint32_t optflow_packet_count = 0;
uint32_t optflow_error_count = 0;
int32_t X_Speed,Y_Speed,Z_distant;

// micolink协议相关定义和函数
#pragma pack (1)
typedef struct {
    uint32_t  time_ms;
    uint32_t  distance;
    uint8_t   strength;
    uint8_t   precision;
    uint8_t   tof_status;
    uint8_t   reserved1;
    int16_t   flow_vel_x;
    int16_t   flow_vel_y;
    uint8_t   flow_quality;
    uint8_t   flow_status;
    uint16_t  reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack ()

typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[64];
    uint8_t checksum;
    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51

// 协议解析函数
bool micolink_check_sum(MICOLINK_MSG_t* msg) {
    uint8_t length = msg->len + 6;
    uint8_t checksum = 0;
    
    checksum += msg->head;
    checksum += msg->dev_id;
    checksum += msg->sys_id;
    checksum += msg->msg_id;
    checksum += msg->seq;
    checksum += msg->len;
    
    for(uint8_t i = 0; i < msg->len; i++) {
        checksum += msg->payload[i];
    }
    
    return (checksum == msg->checksum);
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
    switch(msg->status) {
        case 0: // 帧头
            if(data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->status++;
            }
            break;
            
        case 1: // 设备ID
            msg->dev_id = data;
            msg->status++;
            break;
        
        case 2: // 系统ID
            msg->sys_id = data;
            msg->status++;
            break;
        
        case 3: // 消息ID
            msg->msg_id = data;
            msg->status++;
            break;
        
        case 4: // 包序列
            msg->seq = data;
            msg->status++;
            break;
        
        case 5: // 负载长度
            msg->len = data;
            if(msg->len == 0)
                msg->status += 2;
            else if(msg->len > 64)
                msg->status = 0;
            else
                msg->status++;
            break;
            
        case 6: // 数据负载接收
            msg->payload[msg->payload_cnt++] = data;
            if(msg->payload_cnt == msg->len) {
                msg->payload_cnt = 0;
                msg->status++;
            }
            break;
            
        case 7: // 帧校验
            msg->checksum = data;
            msg->status = 0;
            if(micolink_check_sum(msg)) {
                return true;
            }
            break;
        
        default:
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }
    return false;
}

// 光流传感器初始化
void optflow_init(void) {
    // 初始化UART1
    SerialPort1.begin(UART1_BAUD_RATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    SerialPort1.setRxBufferSize(512);
    SerialPort1.onReceive(onUART1Data);
    
    // 初始化光流数据结构
    memset(&optflow_data, 0, sizeof(optflow_data));
    optflow_data_ready = false;
}

// 数据解码函数
void optflow_decode(uint8_t data) {
    static MICOLINK_MSG_t msg;
    
    if(micolink_parse_char(&msg, data)) {
        switch(msg.msg_id) {
            case MICOLINK_MSG_ID_RANGE_SENSOR: {
                MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
                memcpy(&payload, msg.payload, sizeof(payload));
                
                // 更新光流数据
                optflow_data.distance = payload.distance;
                optflow_data.strength = payload.strength;
                optflow_data.precision = payload.precision;
                optflow_data.tof_status = payload.tof_status;
                optflow_data.flow_vel_x = payload.flow_vel_x;
                optflow_data.flow_vel_y = payload.flow_vel_y;
                optflow_data.flow_quality = payload.flow_quality;
                optflow_data.flow_status = payload.flow_status;
                optflow_data.timestamp = millis();
                
                optflow_data_ready = true;
                optflow_packet_count++;
                
                // 可选：打印调试信息
                /*
                Serial.printf("距离: %dmm, 速度X: %d, 速度Y: %d, 质量: %d\n",
                             optflow_data.distance, 
                             optflow_data.flow_vel_x,
                             optflow_data.flow_vel_y,
                             optflow_data.flow_quality);
                */
                break;
            }
            default:
                break;
        }
    }
}

// UART1中断回调函数
void onUART1Data(void) {
    while (SerialPort1.available()) {
        uint8_t data = SerialPort1.read();
        optflow_decode(data);
    }
}


void RealSpeed(void)
{
  if (optflow_data.distance == 0) {
        // 距离为0表示数据不可用
        X_Speed = 0;
        Y_Speed = 0;
        Z_distant = 0;
    } else {
        Z_distant = optflow_data.distance;    
        // 距离单位mm转换为m，然后乘以光流速度
        float height_m = optflow_data.distance / 1000.0f;
        X_Speed = optflow_data.flow_vel_x * height_m;  // 单位: cm/s
        Y_Speed = optflow_data.flow_vel_y * height_m;  // 单位: cm/s
    }
}
