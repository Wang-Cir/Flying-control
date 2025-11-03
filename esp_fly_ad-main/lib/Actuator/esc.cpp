#include "esc.h"

/**
 * 构造函数：初始化ESC控制器
 * 
 * @param pin PWM信号引脚
 * @param channel PWM通道号
 * @param freq PWM频率
 * @param min_us PWM最小脉宽
 * @param max_us PWM最大脉宽
 * @param max_thr 最大油门值
 */
ESCController::ESCController(int pin, int channel, int freq, int min_us, int max_us, int max_thr) {
    this->pin = pin;
    this->channel = channel;
    this->freq = freq;
    this->min_us = min_us;
    this->max_us = max_us;
    this->max_thr = max_thr;
    this->limit_min_thr = 0;
    this->limit_max_thr = max_thr;
    this->target_thr = 0;

    // 设置 LEDC 通道
    ledcSetup(channel, freq, 10); // 10-bit resolution (0-1023)
    ledcAttachPin(pin, channel);
}

void ESCController::setLimit(int limit_min_thr, int limit_max_thr) {
    this->limit_min_thr = limit_min_thr;
    this->limit_max_thr = limit_max_thr;
}

void ESCController::setThr(int target_thr) {
    target_thr = constrain(target_thr, this->limit_min_thr, this->limit_max_thr);
    this->target_thr = target_thr;

    int us = this->min_us + (this->max_us - this->min_us) * (target_thr / (float)this->max_thr);
    setPWM(us);
}

int ESCController::getThr() {
    return this->target_thr;
}

void ESCController::setThrRelative(int relative_thr) {
    this->target_thr += relative_thr;
    this->setThr(this->target_thr);
}

void ESCController::reset(int target_thr) {
    this->setThr(target_thr);
    // Serial.println("reset(): 复位电机");
}

void ESCController::setPWM(int us) {
    // 将脉宽（微秒）转换为占空比
    int duty = (us * 1024) / (1000000 / freq); // 10-bit resolution
    ledcWrite(channel, duty);
}