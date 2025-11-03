#include "head.h"

#define FREQ          50   //频率
#define CHANNEL0       0   //通道
#define CHANNEL1       1   //通道
#define CHANNEL2       2   //通道
#define CHANNEL3       3   //通道
#define RESOLUTION     13  // 提高到12位分辨率（0-8191范围）
#define PIN0           27   //引脚 
#define PIN1           13   //引脚 
#define PIN2           12   //引脚 
#define PIN3           26   //引脚 

void PWM_init(){

//设置通道
ledcSetup(CHANNEL0,FREQ,RESOLUTION);
ledcSetup(CHANNEL1,FREQ,RESOLUTION);
ledcSetup(CHANNEL2,FREQ,RESOLUTION);
ledcSetup(CHANNEL3,FREQ,RESOLUTION);

//绑定通道与引脚
ledcAttachPin(PIN0,CHANNEL0);
ledcAttachPin(PIN1,CHANNEL1);
ledcAttachPin(PIN2,CHANNEL2);
ledcAttachPin(PIN3,CHANNEL3);

}

//输出 
//ledcWrite(CHANNEL,i);  //i=(0~2^8)