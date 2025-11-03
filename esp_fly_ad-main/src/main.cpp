// 标准库
#include <Arduino.h>
#include <QuickPID.h>

// 自定义库
#include "imu.h"
#include "now.h"
#include "motors.h"
#include "uart.h"

#include "config.h"
#include "utility.h"


// 创建数据结构体
TargetAngle tgAngle;
MeasureAngle msAngle;
TargetRate tgRate;
MeasureRate msRate;
OutRate outRate;  // 输出角速度控制信号

// 全局变量
float deltat; // IMU循环时间间隔
bool isReady = false;

int lx, ly, rx, ry; // 摇杆数据
float rc_thr = 0;     // 推力

// 创建IMU对象
IMUClass imu; // 使用自定义引脚 IMUClass imu(10, 11, 12, 13);

// 创建NOW对象
static uint8_t Mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 默认广播地址
ESPNowReceiver receiver(Mac); 

// 创建电机对象
QuadMotorController motors; // 使用自定义引脚 QuadMotorController motors(9, 8, 7, 6);

// 创建PID对象, 参数在 config.h 中定义, QuickPID(float *Input, float *Output, float *Setpoint);
QuickPID pidRollAngle(&msAngle.roll, &tgRate.roll, &tgAngle.roll, 
                      ROL_ANGLE_P, ROL_ANGLE_I, ROL_ANGLE_D, QuickPID::Action::direct);
QuickPID pidPitchAngle(&msAngle.pitch, &tgRate.pitch, &tgAngle.pitch, 
                       PIT_ANGLE_P, PIT_ANGLE_I, PIT_ANGLE_D, QuickPID::Action::direct);
// QuickPID pidYawAngle(&msAngle.yaw, &tgAngle.yaw, &tgRate.yaw,
//                         YAW_ANGLE_P, YAW_ANGLE_I, YAW_ANGLE_D, QuickPID::Action::direct);

QuickPID pidRollRate(&msRate.roll, &outRate.roll, &tgRate.roll,
                     ROL_RATE_P, ROL_RATE_I, ROL_RATE_D, QuickPID::Action::reverse);
QuickPID pidPitchRate(&msRate.pitch, &outRate.pitch, &tgRate.pitch,
                      PIT_RATE_P, PIT_RATE_I, PIT_RATE_D, QuickPID::Action::direct);
QuickPID pidYawRate(&msRate.yaw, &outRate.yaw, &tgRate.yaw,
                    YAW_RATE_P, YAW_RATE_I, YAW_RATE_D, QuickPID::Action::direct);

LowPassFilter msAngle_roll(0.1);  // 低通滤波器, 参数越小, 滤波效果越好, 响应时间越慢
LowPassFilter msAngle_pitch(0.1);
LowPassFilter msAngle_yaw(0.1);

LowPassFilter msRate_roll(0.7);
LowPassFilter msRate_pitch(0.7);
LowPassFilter msRate_yaw(0.7);

UART pid_config;   // 串口增强类, 用于接收 vofa+ 调试 PID 参数


// 任务函数定义
void remoteDataTask(void *parameter) {

    bool waitingForRelease = false; // 等待松开按键

    while (1) {
        // 更新 ESPNOW 的遥控数据
        int* parsedData = receiver.getParsedDataFix();
        int parsedDataSize = receiver.getParsedDataSize();
        const uint8_t* lastMacAddr = receiver.getLastMacAddr();

        // 紧急停止
        if (parsedData[6] != 0x0) {
            isReady = false;
            Serial.println("紧急停止...");
        }

        // 解析遥控数据
        lx = parsedData[1];
        ly = parsedData[2];
        rx = parsedData[3];
        ry = parsedData[4];

        // 外八解锁
        if (!waitingForRelease && lx < -50 && ly < -50 && rx > 50 && ry < -50 && !isReady) {
            waitingForRelease = true;
            Serial.println("请松开按键以解锁...");
        } else if (waitingForRelease && !(lx < -50 && ly < -50 && rx > 50 && ry < -50)) {
            waitingForRelease = false;
            isReady = true;
            Serial.println("已解锁");

            // 重置 PID, 防止积分误差累积
            pidRollAngle.Reset();
            pidPitchAngle.Reset();
            // pidYawAngle.Reset();

            pidRollRate.Reset();
            pidPitchRate.Reset();
            pidYawRate.Reset();
        }
        
        // 打印解析后的数据
        // Serial.print("now Data: ");
        // for (int i = 0; i < parsedDataSize; i++) {
        //     Serial.print(parsedData[i]);
        //     Serial.print(" ");
        // }
        // Serial.println("\n");

        // Serial.printf("lx: %d, ly: %d, rx: %d, ry: %d \n", lx, ly, rx, ry);

        // 延时一段时间再进行下一次数据处理
        delay(20); // 延时
    }
}

void imuControlTask(void *parameter) {

    float motor1 = 0;
    float motor2 = 0;
    float motor3 = 0;
    float motor4 = 0;
    
    static TickType_t xLastWakeTime = 0; // 用于 vTaskDelayUntil 的变量

    while (1) {
        // 修正 vTaskDelayUntil 的使用
        if (xLastWakeTime == 0) {
            xLastWakeTime = xTaskGetTickCount();
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 10ms

        // 更新 IMU 数据
        imu.update();
        // 获取姿态角
        imu.getPitchRollYaw(msAngle.pitch, msAngle.roll, msAngle.yaw);
        // 获取角速度
        imu.getGyrData(msRate.pitch, msRate.roll, msRate.yaw);
        deltat = imu.getDeltat() * 1000; // ms

        // 对于倒置的 IMU，将 pitch 和 roll 加减 180 度, roll 取反
        msAngle.pitch =   (msAngle.pitch > 0) ? (msAngle.pitch - 180) : (msAngle.pitch + 180);
        msAngle.roll  = -((msAngle.roll > 0) ? (msAngle.roll - 180) : (msAngle.roll + 180));

        // 低通滤波
        msAngle.roll  = msAngle_roll.filter(msAngle.roll);
        msAngle.pitch = msAngle_pitch.filter(msAngle.pitch);

        // msRate.roll  = msRate_roll.filter(msRate.roll);
        // msRate.pitch = msRate_pitch.filter(msRate.pitch);
        // msRate.yaw   = msRate_yaw.filter(msRate.yaw);
        
        // // 打印角度数据
        // Serial.printf(">msAngle_Roll: %.2f, msAngle_Pitch: %.2f, msAngle_Yaw:%.2f, deltat_ms: %.6f\n",
        //               msAngle.roll, msAngle.pitch, msAngle.yaw, deltat);
        // 打印角度数据
        // Serial.printf("msAngle: %.2f, %.2f, %.2f, %.6f\n",
        //               msAngle.roll, msAngle.pitch, msAngle.yaw, deltat);

        // 摔倒, 快速翻转检测
        if (abs(msAngle.roll)  > 70  ||
            abs(msAngle.pitch) > 60  ||
            abs(msRate.roll)   > 300 ||
            abs(msRate.pitch)  > 300 )
        {
            motors.reset(); // 重置电机
            isReady = false;
            Serial.println("检测到摔倒, 紧急停止...");
        }

        // 更新 PID 参数
        if (pid_config.UpdatePidParams(ROL_ANGLE_P, ROL_ANGLE_I, ROL_ANGLE_D,
                                       ROL_RATE_P, ROL_RATE_I, ROL_RATE_D))
        {
            pidRollAngle.SetTunings(ROL_ANGLE_P, ROL_ANGLE_I, ROL_ANGLE_D);
            pidRollRate.SetTunings(ROL_RATE_P, ROL_RATE_I, ROL_RATE_D);

            pidRollAngle.Reset();
            pidRollRate.Reset();
            
            Serial.println("PID参数更新成功");
        }
        Serial.printf("当前 angle_PID 参数, ANGLE_P = %f, ANGLE_I = %f, ANGLE_D = %f\n",
                      ROL_ANGLE_P, ROL_ANGLE_I, ROL_ANGLE_D);
        Serial.printf("当前 rate_PID 参数: RATE_P: %f, RATE_I: %f, RATE_D: %f\n",
                      ROL_RATE_P, ROL_RATE_I, ROL_RATE_D);

        // PID 运算
        pidRollAngle.Compute();
        pidPitchAngle.Compute();
        // pidYawAngle.Compute();
        pidRollRate.Compute();
        pidPitchRate.Compute();
        pidYawRate.Compute();

        // 打印 角度环 和 角速度环 输出
        // Serial.printf("角度环: Roll: %.2f, Pitch: %.2f, Yaw: %.2f, deltat: %.6f ms \n", 
        //               tgRate.roll, tgRate.pitch, tgRate.yaw, deltat);
        // Serial.printf("outRate: %.2f, %.2f, %.2f, %.6f \n",
        //                outRate.roll, outRate.pitch, outRate.yaw, deltat);

        // VOFA+ 调试输出
        Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.6f\n",
                msAngle.roll, msAngle.pitch, msAngle.yaw, 
                tgRate.roll, tgRate.pitch, tgRate.yaw,
                msRate.roll, msRate.pitch, msRate.yaw,
                outRate.roll, outRate.pitch, outRate.yaw, deltat);

        // 推力控制
        rc_thr = rc_thr + (ly * 0.03);        // 从摇杆控制推力
        rc_thr = constrain(rc_thr, 0, 650);   // 推力限制

        // 电机控制
        motor1 = rc_thr + outRate.roll - outRate.pitch - outRate.yaw;
        motor2 = rc_thr - outRate.roll - outRate.pitch + outRate.yaw;
        motor3 = rc_thr - outRate.roll + outRate.pitch - outRate.yaw;
        motor4 = rc_thr + outRate.roll + outRate.pitch + outRate.yaw;

        // 限制电机推力范围
        motor1 = constrain(motor1, 0, 1000);  // 左前
        motor2 = constrain(motor2, 0, 1000);  // 右前
        motor3 = constrain(motor3, 0, 1000);  // 右后
        motor4 = constrain(motor4, 0, 1000);  // 左后

        Serial.printf("motors= %.2f %.2f %.2f %.2f  rc_thr= %.2f\n", 
                        motor1, motor2, motor3, motor4, rc_thr);

        // 检查是否准备好
        if (isReady) {
            // 准备好的话, 发送电机控制指令
            motors.setMotorsThr(motor1, motor2, motor3, motor4); // 设置电机推力
            digitalWrite(15, !digitalRead(15));

        } else {
            // 未准备好, 重置电机和IMU
            digitalWrite(15, LOW);
            motors.reset();    // 重置电机
            
            rc_thr = 0;        // 重置推力

            Serial.println("未解锁, 重置电机..........................");
        }
    }
}


void setup()
{
    // 设置灯光输出引脚
    pinMode(15, OUTPUT);  
    digitalWrite(15, HIGH);

    // 初始化串口
    Serial.begin(115200);
    pid_config.begin(Serial);  // 初始化串口增强类

    // while (!Serial){} // 等待串口连接
    Serial.println("初始化CDC串口成功...");

    // 初始化IMU
    if (!imu.begin())
    {
        Serial.println("等待 IMU 初始化...");
        while (!imu.begin()) {delay(100);} // 等待 IMU 初始化
    }
    imu.calculateGyrBias(100, 3); // 计算陀螺仪Z轴零偏, 参数为采样次数
    Serial.println("初始化 ICM4268 成功...");

    // 初始化NOW
    receiver.begin();

    // 初始化PID
    pidRollAngle.SetTunings(ROL_ANGLE_P, ROL_ANGLE_I, ROL_ANGLE_D);
    pidRollAngle.SetOutputLimits(-1000, 1000);  // 设置输出限幅
    pidRollAngle.SetMode(1);                    // 设置PID模式为自动控制
    pidPitchAngle.SetTunings(PIT_ANGLE_P, PIT_ANGLE_I, PIT_ANGLE_D);
    pidPitchAngle.SetOutputLimits(-1000, 1000);
    pidPitchAngle.SetMode(1);
    // pidYawAngle.SetTunings(YAW_ANGLE_P, YAW_ANGLE_I, YAW_ANGLE_D);
    // pidYawAngle.SetOutputLimits(-1000, 1000);
    // pidYawAngle.SetMode(1);

    pidRollRate.SetTunings(ROL_RATE_P, ROL_RATE_I, ROL_RATE_D);
    pidRollRate.SetOutputLimits(-1000, 1000);
    pidRollRate.SetMode(1);
    pidPitchRate.SetTunings(PIT_RATE_P, PIT_RATE_I, PIT_RATE_D);
    pidPitchRate.SetOutputLimits(-1000, 1000);
    pidPitchRate.SetMode(1);
    pidYawRate.SetTunings(YAW_RATE_P, YAW_RATE_I, YAW_RATE_D);
    pidYawRate.SetOutputLimits(-1000, 1000);
    pidYawRate.SetMode(1);

    // 初始化电机
    motors.reset(); // 重置电机

    // 创建遥控数据处理任务
    xTaskCreate(
        remoteDataTask,   // 任务函数
        "RemoteData",     // 任务名称
        2048,             // 任务堆栈大小
        NULL,             // 传递给任务的参数
        1,                // 任务优先级
        NULL              // 任务句柄
    );

    // 创建 IMU 控制任务
    xTaskCreate(
        imuControlTask,   // 任务函数
        "IMUControl",     // 任务名称
        4096,             // 任务堆栈大小
        NULL,             // 传递给任务的参数
        5,               // 任务优先级 最高级
        NULL              // 任务句柄
    );
}

void loop()
{
    // 任务延时
}


