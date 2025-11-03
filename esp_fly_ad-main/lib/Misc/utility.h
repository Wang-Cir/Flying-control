#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

// 一些工具代码

// 限制输入的值在给定的范围内
float limit_value(float value, float min_value = -3000, float max_value = 3000);

// 将给定的值映射到给定的目标范围
float map_value(float value, float original_min, float original_max, float target_min, float target_max);

// 一阶低通滤波器类
class LowPassFilter
{
public:
    LowPassFilter(float alpha) : alpha(alpha), filteredValue(0.0) {}
    void setAlpha(float alpha);
    float filter(float input);
    void reset(float initialValue = 0.0);

private:
    float alpha;         // 滤波器系数，范围 0 到 1
    float filteredValue; // 滤波后的值
};

// 滑动平均滤波器类
class MovingAverageFilter
{
public:
    MovingAverageFilter(int windowSize);
    ~MovingAverageFilter();
    float filter(float input);

private:
    int windowSize;
    int bufferIndex;
    float sum;
    float *buffer;
};

// 时间差计算类
class TimeDiff
{
public:
    TimeDiff();
    unsigned long time_diff();

private:
    unsigned long last_time;
};

#endif // UTILITY_H