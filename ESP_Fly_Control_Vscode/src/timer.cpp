#include <head.h>
  
hw_timer_t*timer = NULL;
volatile int flag,count,main_flag,LEDset,count2,flag2;
volatile float roll_error,pitch_error,Yaw_error;

void timer_interrupt_init(){
//初始化定时器
timer = timerBegin(0,80,true);
//配置定时器
timerAttachInterrupt(timer,&timer_interrupt,true);
//设置定时模式
timerAlarmWrite(timer,1000*1,true);
//启动定时器
timerAlarmEnable(timer);

}

//定时中断 暂时不用
void timer_interrupt(){

}


