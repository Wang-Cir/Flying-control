#include "utility.h"

// 限制输入的值在给定的范围内
float limit_value(float value, float min_value, float max_value)
{
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}

// 将给定的值映射到给定的目标范围
float map_value(float value, float original_min, float original_max, float target_min, float target_max)
{
    return target_min + (value - original_min) * (target_max - target_min) / (original_max - original_min);
}

// 一阶低通滤波器类
void LowPassFilter::setAlpha(float alpha)
{
    this->alpha = alpha;
}

float LowPassFilter::filter(float input)
{
    filteredValue = alpha * input + (1.0 - alpha) * filteredValue;
    return filteredValue;
}

void LowPassFilter::reset(float initialValue)
{
    filteredValue = initialValue;
}

// 滑动平均滤波器类
MovingAverageFilter::MovingAverageFilter(int windowSize) : windowSize(windowSize), bufferIndex(0), sum(0.0)
{
    buffer = new float[windowSize];
    for (int i = 0; i < windowSize; i++)
    {
        buffer[i] = 0.0;
    }
}

MovingAverageFilter::~MovingAverageFilter()
{
    delete[] buffer;
}

float MovingAverageFilter::filter(float input)
{
    sum -= buffer[bufferIndex];                   // 减去旧值
    buffer[bufferIndex] = input;                  // 存储新值
    sum += input;                                 // 加上新值
    bufferIndex = (bufferIndex + 1) % windowSize; // 更新索引
    return sum / windowSize;                      // 返回平均值
}

// 时间差计算类
TimeDiff::TimeDiff() : last_time(0) {}

unsigned long TimeDiff::time_diff()
{
    unsigned long current_time = micros(); // 获取当前时间（单位：微秒）

    if (last_time == 0) // 如果是第一次调用，更新last_time
    {
        last_time = current_time;
        return 1; // 防止除零错误
    }
    else // 计算时间差
    {
        unsigned long diff = current_time - last_time; // 计算时间差
        last_time = current_time;                      // 更新上次调用时间
        return diff;                                   // 返回时间差微秒
    }
}