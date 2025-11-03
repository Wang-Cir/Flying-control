#include "motors.h"

/**
 * 四旋翼电机控制器类的构造函数
 *
 * @param m1_pin 电机1的控制引脚，默认值为18
 * @param m2_pin 电机2的控制引脚，默认值为16
 * @param m3_pin 电机3的控制引脚，默认值为21
 * @param m4_pin 电机4的控制引脚，默认值为17
 * @param limit_min_thr 电机最小转速限制，默认值为0
 * @param limit_max_thr 电机最大转速限制，默认值为1000
 *
 * 本构造函数初始化四个电机对象，并为它们分配对应的控制引脚和PWM通道, 默认使用 1~4 号PWM通道
 */
QuadMotorController::QuadMotorController(int m1_pin, int m2_pin, int m3_pin, int m4_pin, int limit_min_thr, int limit_max_thr)
    : motor_1(m1_pin, 1), motor_2(m2_pin, 2), motor_3(m3_pin, 3), motor_4(m4_pin, 4) // 默认使用 1~4 号PWM通道
{
    motors[0] = &motor_1;
    motors[1] = &motor_2;
    motors[2] = &motor_3;
    motors[3] = &motor_4;
    setMotorsLimit(limit_min_thr, limit_max_thr);
    reset();
}

void QuadMotorController::setMotorsLimit(int limit_min_thr, int limit_max_thr)
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setLimit(limit_min_thr, limit_max_thr);
    }
}
void QuadMotorController::setMotorsThr(int thr1, int thr2, int thr3, int thr4)
{
    motors[0]->setThr(thr1);
    motors[1]->setThr(thr2);
    motors[2]->setThr(thr3);
    motors[3]->setThr(thr4);
}

void QuadMotorController::setMotorsThrRelative(int thr1, int thr2, int thr3, int thr4)
{
    motors[0]->setThrRelative(thr1);
    motors[1]->setThrRelative(thr2);
    motors[2]->setThrRelative(thr3);
    motors[3]->setThrRelative(thr4);
}

void QuadMotorController::getMotorsThr()
{
    motors[0]->getThr();
    motors[1]->getThr();
    motors[2]->getThr();
    motors[3]->getThr();
}

void QuadMotorController::reset()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->reset();
    }
    // Serial.println("reset(): 复位电机");
}