#include <Arduino.h>
#include "imu.h"

IMUClass imu;  // 使用自定义引脚 IMUClass imu(10, 11, 12, 13);

float gx, gy, gz, ax, ay, az;
float pitch, roll, yaw;
float deltat;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("Start...");
    
    if (!imu.begin()) {
        Serial.println("IMU initialization unsuccessful");
        while (1) {}
    }
    
    Serial.println("ICM42688 begin...");
    analogWrite(2, 512);

    imu.calculateGyrZBias(100);  // 计算陀螺仪Z轴零偏
}


void loop() {

    imu.update();
    
    // 显示数据
    // Serial.printf("AccX: %f\t AccY: %f\t AccZ: %f\t GyrX: %f\t GyrY: %f\t GyrZ: %f\t Temp: %f\n", 
    //              ax, ay, az, gx, gy, gz, temp);

    imu.getPitchRollYaw(pitch, roll, yaw);  // 获取姿态角
    deltat = imu.getDeltat() * 1000;        // ms
    
    // 对于倒置的IMU，将pitch和roll加减180度
    pitch = (pitch > 0) ? (pitch - 180) : (pitch + 180);
    roll  =  (roll > 0) ?  (roll - 180) :  (roll + 180);

    Serial.printf("Pitch: %.2f\t Roll: %.2f\t Yaw: %.2f\t deltat: %.2f ms \n", pitch, roll, yaw, deltat);

    delay(10);
}