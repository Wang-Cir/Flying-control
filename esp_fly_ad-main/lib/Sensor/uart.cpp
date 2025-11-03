#include "uart.h"

void UART::begin(USBCDC &serial, long baudRate) {
    _serial = &serial;
    _serial->begin(baudRate);
    _serial->println("初始化串口...");
}

String UART::readLine() {
    if (_serial->available()) {
        String receivedData = ""; // 初始化接收缓冲区
        while (_serial->available()) {
            char c = _serial->read();
            if (c == '\n') {
                break; // 遇到换行符，结束循环
            }
            receivedData += c; // 将字符添加到接收缓冲区
        }

        if (receivedData.length() > 0) {
            _serial->print("Received: ");
            _serial->println(receivedData);
            return receivedData;
        }
    }
    return "";  // 返回空字符串
}

bool UART::UpdatePidParams(float &kp, float &ki, float &kd, float &kp2, float &ki2, float &kd2) {
    int startIndex = 0;
    int endIndex = 0;

    receivedData = readLine(); // 读取串口数据

    while (startIndex < receivedData.length()) {
        // 找到冒号的位置
        endIndex = receivedData.indexOf(':', startIndex);
        if (endIndex == -1) {
            break; // 没有找到冒号，结束解析
        }

        // 提取键
        key = receivedData.substring(startIndex, endIndex);
        key.trim(); // 调用 trim() 方法

        // 找到换行符的位置
        startIndex = receivedData.indexOf('\n', endIndex);
        if (startIndex == -1) {
            startIndex = receivedData.length(); // 没有找到换行符，处理最后一行
        }

        // 提取值
        valueStr = receivedData.substring(endIndex + 1, startIndex);
        valueStr.trim(); // 调用 trim() 方法
        value = valueStr.toFloat();

        // 打印解析后的数据
        _serial->printf("Key: %s, Value: %f\n", key.c_str(), value);

        // 更新起始位置
        startIndex++;

        // 根据 key 更新 pid 参数
        if (key == "angle_kp") {
            kp = value;
            return true;
        } else if (key == "angle_ki") {
            ki = value;
            return true;
        } else if (key == "angle_kd") {
            kd = value;
            return true;
        } else if (key == "rate_kp") {
            kp2 = value;
            return true;
        } else if (key == "rate_ki") {
            ki2 = value;
            return true;
        } else if (key == "rate_kd") {
            kd2 = value;
            return true;
        }
    }
    return false;
}