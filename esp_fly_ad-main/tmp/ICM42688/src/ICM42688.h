#ifndef ICM42688P_h
#define ICM42688P_h

#include "Arduino.h"
#include <SPI.h>

// 寄存器地址
#define ICM42688_DEVICE_CONFIG 0x11         // 设备配置寄存器地址
#define ICM42688_PWR_MGMT0 0x4E             // 电源管理寄存器地址
#define ICM42688_WHO_AM_I 0x75              // 设备ID寄存器地址
#define ICM42688_ACCEL_DATA_X1 0x1F         // 加速度计X轴数据寄存器地址（高8位）
#define ICM42688_ACCEL_DATA_X0 0x20         // 加速度计X轴数据寄存器地址（低8位）
#define ICM42688_GYRO_DATA_X1 0x25          // 陀螺仪X轴数据寄存器地址（高8位）
#define ICM42688_GYRO_DATA_X0 0x26          // 陀螺仪X轴数据寄存器地址（低8位）
#define ICM42688_REG_BANK_SEL 0x76          // 寄存器库选择地址
#define ICM42688_TEMP_DATA1 0x1D            // 温度数据寄存器地址（高8位）

// 配置用的位掩码
#define ICM42688_PWR_TEMP_ON (0 << 5)       // 电源管理：温度传感器开启
#define ICM42688_PWR_TEMP_OFF (1 << 5)      // 电源管理：温度传感器关闭
#define ICM42688_PWR_GYRO_MODE_LN (3 << 2)  // 电源管理：陀螺仪低噪声模式
#define ICM42688_PWR_ACCEL_MODE_LN (3 << 0) // 电源管理：加速度计低噪声模式

#define ICM42688_GFS_2000DPS (0x00 << 5)    // 陀螺仪满量程范围：2000度/秒
#define ICM42688_AFS_16G (0x00 << 5)        // 加速度计满量程范围：16g
#define ICM42688_GODR_1kHz 0x06             // 陀螺仪输出数据速率：1kHz
#define ICM42688_AODR_1kHz 0x06             // 加速度计输出数据速率：1kHz

#define SPI_TIMEOUT 500000  // SPI通信超时时间（500ms）

class ICM42688P {
  public:
    ICM42688P(int csPin);
    void initialize();
    void configureAccelerometer();
    void configureGyroscope();
    void readAccelerometer(float &accelX, float &accelY, float &accelZ);
    void readGyroscope(float &gyroX, float &gyroY, float &gyroZ);
    float readTemperature();

  private:
    int _csPin;
    SPIClass *_spi;
    float _accelScale = 16.0;
    float _gyroScale = 2000.0;

    void _writeRegister(uint8_t reg, uint8_t value);
    uint8_t _readRegister(uint8_t reg);
    void _readMultipleRegisters(uint8_t reg, uint8_t *buffer, int length);
};

#endif