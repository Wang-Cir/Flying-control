#ifndef OLED_H
#define OLED_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 声明OLED初始化函数
void OLED_init();

// 外部声明OLED对象，供其他文件使用
extern Adafruit_SSD1306 pm;

// 屏幕尺寸宏定义（与cpp文件保持一致）
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#endif // OLED_H