#ifndef TIMER_H
#define TIMER_H

// 外部声明定时器变量和计数变量
extern hw_timer_t* timer;
extern volatile int flag,count,main_flag,LEDset,flag2;
extern volatile float roll_error,pitch_error,Yaw_error;
// 函数声明
void  timer_interrupt();
void  timer_interrupt_init();

//void  handle_rx_interrupt();

#endif 