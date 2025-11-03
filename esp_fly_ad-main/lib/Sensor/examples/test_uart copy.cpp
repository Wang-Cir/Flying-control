#include <Arduino.h>
#include "config.h"


// 设置需要调试的参数 
float *kp = &ROL_RATE_P;
float *ki = &ROL_RATE_I;
float *kd = &ROL_RATE_D;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    } // 等待串口连接
    Serial.println("初始化CDC串口成功...");
}


int i = 0;
void loop()
{
    // 从串口读取输入数据
    if (Serial.available())
    {
        String receivedData = ""; // 初始化接收缓冲区
        while (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n')
            {
                break; // 遇到换行符，结束循环
            }
            receivedData += c; // 将字符添加到接收缓冲区
        }

    // 从输入的数据中解析参数
        if (receivedData.length() > 0)
        { // 检查是否读取到有效数据
            Serial.print("Received: ");
            Serial.println(receivedData); // 处理接收到的数据

            // 解析数据
            int startIndex = 0;
            int endIndex = 0;

            while (startIndex < receivedData.length())
            {
                // 找到冒号的位置
                endIndex = receivedData.indexOf(':', startIndex);
                if (endIndex == -1)
                {
                    break; // 没有找到冒号，结束解析
                }

                // 提取键
                String key = receivedData.substring(startIndex, endIndex);
                key.trim(); // 调用 trim() 方法

                // 找到换行符的位置
                startIndex = receivedData.indexOf('\n', endIndex);
                if (startIndex == -1)
                {
                    startIndex = receivedData.length(); // 没有找到换行符，处理最后一行
                }

                // 提取值
                String valueStr = receivedData.substring(endIndex + 1, startIndex);
                valueStr.trim(); // 调用 trim() 方法
                float value = valueStr.toFloat();

                // 根据 key 更新 pid 参数
                if (key == "kp") {
                    *kp = value; 
                } else if (key == "ki") {
                    *ki = value;
                } else if (key == "kd") {
                    *kd = value;
                }

                // 打印解析后的数据
                Serial.printf("Key: %s, Value: %f\n", key.c_str(), value);

                // 更新起始位置
                startIndex++;
            }

        }
    }
}