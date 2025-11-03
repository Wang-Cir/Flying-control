#include <Arduino.h>
#include <QuickPID.h>

#include "imu.h"
#include "now.h"
#include "motors.h"
#include "config.h"
#include "utility.h"

IMUClass imu; // 使用自定义引脚 IMUClass imu(10, 11, 12, 13);

// Define Variables we'll be connecting to
float Setpoint=0, Input=0, Output=0;

float Kp = 1, Ki = 0, Kd = 0;

// Specify PID links
QuickPID myPID(&Input, &Output, &Setpoint);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    } // 等待串口连接
    // 初始化IMU
    if (!imu.begin())
    {
        Serial.println("IMU initialization unsuccessful");
        while (1)
        {
        }
    }
    imu.calculateGyrBias(100); // 计算陀螺仪Z轴零偏
    Serial.println("初始化 ICM4268 成功...");

    Setpoint = 100;

    // apply PID gains
    myPID.SetOutputLimits(-1000, 1000);
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetMode(1);
    
    }
    void loop()
    {
        imu.update();
        Input = imu.getRoll();
        myPID.Compute();
        Serial.printf("Input: %f\t Output: %f\t Setpoint: %f\n", Input, Output, Setpoint);
    }