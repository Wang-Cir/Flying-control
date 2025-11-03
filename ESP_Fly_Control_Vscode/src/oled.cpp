#include "head.h"

// 定义屏幕尺寸
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


Adafruit_SSD1306 pm(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void OLED_init()
{
Wire.begin(22, 21);  // SDA=22, SCL=21
    // 初始化OLED屏幕 (这部分是之前缺失的)
if(!pm.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // 如果初始化失败，进入死循环
    for(;;);
  }
    // 初始清屏
  pm.clearDisplay();
  
}


