//并环
#include "head.h"
float P_roll,test;
extern int code,num;
// 控制参数（基于F330机型调参）
typedef struct {
  float Kp_angle, Ki_angle, Kd_angle; // 角度环
  float max_angle;                   // 最大允许倾角
  float integral_limit;              // 积分限幅
} PID_Params;

int throttle,DIR_UP,DIR_Down;   
float target_roll = 0, target_pitch = 0, target_Yaw = 0;  
float target_roll_rate;
float target_pitch_rate;
float target_yaw_rate, last_err_yaw; 

// 新增控制变量
float target_X_speed = 0, target_Y_speed = 0; // 目标速度
float target_height = 30.0f; // 目标高度30cm
float Kp_XSpeed=0.15, Kp_YSpeed=0.15;

// 速度环PID参数
float Kp_speed_x = 0.15f, Ki_speed_x = 0.01f, Kd_speed_x = 0;
float Kp_speed_y = 0.15f, Ki_speed_y = 0.01f, Kd_speed_y = 0;
float integral_speed_x = 0, integral_speed_y = 0;
float last_speed_x_error = 0, last_speed_y_error = 0;

// 高度环PID参数
float Kp_height = 0.8f, Ki_height = 0.05f, Kd_height = 0;
float height_integral = 0, last_height_error = 0;

int motor_output[4] = {0};
int motor_output_pre[4] = {0};         
float integral_pitch=0,integral_roll=0;

void PID_Control() {
    
    PID_Params roll_pid =  {0.52f, 0.13, 0.17, 30, 6};
    PID_Params pitch_pid = {0.52f, 0.13 ,0.17, 30, 6};
    PID_Params yaw_pid =   {-0.02, -0.02, 0.2, 180, 5};   
    Yaw_error   = Yaw_error2  (target_Yaw, Yaw);
    RealSpeed();
    
    // // 1. 高度控制（当油门大于520时定高）
    // if (throttle > 520) {
    //     float height_error = target_height - Z_distant;
    //     height_integral += height_error * 0.001;
    //     height_integral = constrain(height_integral, -10.0f, 10.0f);
        
    //     float height_derivative = (height_error - last_height_error) / 0.001;
    //     last_height_error = height_error;
        
    //     float height_adjust = Kp_height * height_error + 
    //                          Ki_height * height_integral + 
    //                          Kd_height * height_derivative;
        
    //     // 高度调整量叠加到基础油门
    //     throttle = 520 + height_adjust * 10.0f;
    //     throttle = constrain(throttle, 500, 580);
    // }

    // 2. 速度环控制（输出到姿态环的目标倾角）
    if (throttle > 450 && Z_distant > 250) {
        // X方向速度控制（前后移动 -> 控制roll）
        float x_speed_error = target_X_speed - X_Speed;
        integral_speed_x += x_speed_error * 0.001;
        integral_speed_x = constrain(integral_speed_x, -5.0f, 5.0f);
        
        float x_speed_derivative = (x_speed_error - last_speed_x_error) / 0.001;
        last_speed_x_error = x_speed_error;
        
        target_roll = Kp_speed_x * x_speed_error + 
                      Ki_speed_x * integral_speed_x + 
                      Kd_speed_x * x_speed_derivative;
       

        // Y方向速度控制（左右移动 -> 控制pitch）
        float y_speed_error = target_Y_speed + Y_Speed;
        integral_speed_y += y_speed_error * 0.001;
        integral_speed_y = constrain(integral_speed_y, -5.0f, 5.0f);
        
        float y_speed_derivative = (y_speed_error - last_speed_y_error) / 0.001;
        last_speed_y_error = y_speed_error;
        
        target_pitch = Kp_speed_y * y_speed_error + 
                     Ki_speed_y * integral_speed_y + 
                     Kd_speed_y * y_speed_derivative;
        
    } else {
        // 非定高模式时重置
        target_roll = 0;
        target_pitch = 0;
        integral_speed_x = 0;
        integral_speed_y = 0;
    }
    if( target_X_speed == 0&&target_Y_speed == 0)
 { 
   target_roll = constrain(target_roll, -10.0f, 10.0f);
   target_pitch = constrain(target_pitch, -10.0f, 10.0f);
 }
 if( target_X_speed != 0||target_Y_speed != 0)
 { 
   target_roll = constrain(target_roll, -15.0f, 15.0f);
   target_pitch = constrain(target_pitch, -15.0f, 15.0f);
 }

    roll_error  = Roll_error2 (target_roll, Roll);
    pitch_error = Pitch_error2(target_pitch, Pitch);

    if (abs(roll_error) < roll_pid.max_angle && motor_output[0] < 650 && throttle > 500) {
        integral_roll = constrain(integral_roll + roll_error * 0.001, 
                                  -roll_pid.integral_limit, 
                                   roll_pid.integral_limit);
        target_roll_rate = roll_pid.Kp_angle * roll_error + 
                          roll_pid.Ki_angle * integral_roll + 
                          roll_pid.Kd_angle * -gyroX;
    } else {
        target_roll_rate = roll_pid.Kp_angle * roll_error;
    }

    if (abs(pitch_error) < pitch_pid.max_angle && motor_output[0] < 650 && throttle > 500) {
        integral_pitch = constrain(integral_pitch + pitch_error * 0.001, 
                                 -pitch_pid.integral_limit, 
                                  pitch_pid.integral_limit);
        target_pitch_rate = pitch_pid.Kp_angle * pitch_error + 
                          pitch_pid.Ki_angle * integral_pitch + 
                          pitch_pid.Kd_angle * -gyroY;
    } else {
        target_pitch_rate = pitch_pid.Kp_angle * pitch_error;
    }

    // Yaw PID控制
    if (abs(Yaw_error) < yaw_pid.max_angle && throttle > 450 && Z_distant > 250) {
        static float integral_yaw = 0;
        integral_yaw = constrain(integral_yaw + Yaw_error * 0.001,
                               -yaw_pid.integral_limit,
                                yaw_pid.integral_limit);
       
        target_yaw_rate = yaw_pid.Kp_angle * Yaw_error +
                         yaw_pid.Ki_angle * integral_yaw +
                         yaw_pid.Kd_angle * gyroZ;
        last_err_yaw = Yaw_error;
    }

    // 限幅
    target_yaw_rate = constrain(target_yaw_rate, -10, 10);

    // 4. 电机混控（去掉直接的速度项，只使用姿态环输出）
    motor_output_pre[0] = throttle + ( target_roll_rate + target_pitch_rate - target_yaw_rate) * 2;  //右下
    motor_output_pre[1] = throttle + (-target_roll_rate + target_pitch_rate + target_yaw_rate) * 2;  //右上
    motor_output_pre[2] = throttle + ( target_roll_rate - target_pitch_rate + target_yaw_rate) * 2;  //左下
    motor_output_pre[3] = throttle + (-target_roll_rate - target_pitch_rate - target_yaw_rate) * 2;  //左上

    for(int i = 0; i < 4; i++) {
        motor_output_pre[i] = constrain(motor_output_pre[i], throttle, throttle * 1.3);
        motor_output_pre[i] = constrain(motor_output_pre[i], 0, 600);
        motor_output[i] = motor_output_pre[i];
    }

    // 遥控器控制（测试模式）
    if(num == 3) {
        motor_output[0] = 800;
        motor_output[1] = 800;
        motor_output[2] = 800;
        motor_output[3] = 800;
    }
    if(num == 4) {
        motor_output[0] = 400;
        motor_output[1] = 400;
        motor_output[2] = 400;
        motor_output[3] = 400;
    }
    if(num == 6) {
         target_Y_speed = 100.0f; // 前进
     } 
     if(num == 7) {
         target_Y_speed = -100.0f; // 后退
     }
     if(num == 8) {
         target_X_speed = 100.0f; // 前进
     } 
     if(num == 9) {
         target_X_speed = -100.0f; // 后退
     }

     if(num == 5) 
    {
throttle*= 0.99;
motor_output[0]*= 0.99;
motor_output[1]*= 0.99;
motor_output[2]*= 0.99;
motor_output[3]*= 0.99;
      }          
    //else if(num == 10) { // 假设9为右移
    //     DIR_Down = 5.0f;
    // } else if(num == 11) { // 假设10为左移
    //     DIR_Down = -5.0f;
    // } else if(num == 5) { // 停止移动
    //     DIR_Down = 0;
    //     DIR_UP = 0;
    // }

    // 5. 安全保护
    if(abs(roll_error) > 30 || abs(pitch_error) > 30 || abs(gyroX) > 120 || abs(gyroY) > 120) {
        throttle = 0;
        for(int i = 0; i < 4; i++) {
            motor_output[i] = 0;
        }
        // 重置所有积分项
        integral_roll = 0;
        integral_pitch = 0;
        height_integral = 0;
        integral_speed_x = 0;
        integral_speed_y = 0;
    }

    for(int i = 0; i < 4; i++) {
        motor_output[i] = constrain(motor_output[i], 0, 800);
    }
}






// #include "head.h"
// float P_roll;
// int PID_flag;
// extern int num;

// // 优化的串级PID参数 - 借鉴官方代码结构
// typedef struct {
//   // 外环参数
//   float Kp_angle, Ki_angle, Kd_angle;        
//   // 内环参数  
//   float Kp_rate, Ki_rate, Kd_rate; 
//   // 限制参数
//   float max_angle, max_rate;                  
//   float integral_limit_angle, integral_limit_rate;       
// } PID_Cascade_Params;

// int throttle;   
// float target_roll = 0, target_pitch = 0;  
// int motor_output[4] = {0};  
// int motor_output_pre[4] = {0};       
// float roll_rate_output, pitch_rate_output;
// // PID控制变量
// float roll_angle_integral = 0, pitch_angle_integral = 0;
// float roll_rate_integral = 0, pitch_rate_integral = 0;
// float last_roll_error = 0, last_pitch_error = 0;
// float last_roll_rate_error = 0, last_pitch_rate_error = 0;

// // 存储上次内环输出
// float last_roll_output = 0, last_pitch_output = 0;

// void PID_Control() {
//     PID_flag++;

//     // 优化的PID参数 - 基于官方代码调整
//     PID_Cascade_Params roll_pid = {
//         0, 0, 0,     
//         1.0f,5.5f,0.04f+P_roll,
//         30, 80,          
//         5, 15            
//     };
    
//     PID_Cascade_Params pitch_pid = {
//         0, 0, 0, 
//         1.0f,5.5f,0.04f+P_roll, 
//         30, 80, 
//         5, 15
//     };

//      roll_error = Roll_error2(target_roll, Roll);
//      pitch_error = Pitch_error2(target_pitch, Pitch);

//     // 积分处理 - 采用官方积分分离策略
//     if (throttle > 440 && fabsf(roll_error) < roll_pid.max_angle) {
//         roll_angle_integral = constrain(roll_angle_integral + roll_error * 0.001f, 
//                                        -roll_pid.integral_limit_angle, roll_pid.integral_limit_angle);
//     } else if (fabsf(roll_error)>30) {
//         roll_angle_integral = 0;  // 积分分离
//     }
    
//     // 外环输出 - 直接使用原始陀螺仪作为微分（官方方法）
//     float target_roll_rate = roll_pid.Kp_angle * roll_error + 
//                             roll_pid.Ki_angle * roll_angle_integral +
//                             gyroX * roll_pid.Kd_angle;  // 使用原始陀螺仪
    
//     target_roll_rate = constrain(target_roll_rate, -roll_pid.max_rate, roll_pid.max_rate);

//     // Pitch轴外环
//     if (throttle > 440 && fabsf(pitch_error) < pitch_pid.max_angle) {
//         pitch_angle_integral = constrain(pitch_angle_integral + pitch_error * 0.001f, 
//                                         -pitch_pid.integral_limit_angle, pitch_pid.integral_limit_angle);
//     } else if (fabsf(pitch_error)>30) {
//         pitch_angle_integral = 0;
//     }
    
//     float target_pitch_rate = pitch_pid.Kp_angle * pitch_error + 
//                              pitch_pid.Ki_angle * pitch_angle_integral +
//                              gyroY * pitch_pid.Kd_angle;
//     target_pitch_rate = constrain(target_pitch_rate, -pitch_pid.max_rate, pitch_pid.max_rate);
     
 
//         // Roll轴内环
//         float roll_rate_error = 0 - gyroX;
        
//         // 内环积分处理
//         if (throttle > 480 && fabsf(roll_rate_error) < 120.0f) {
//             roll_rate_integral = constrain(roll_rate_integral + roll_rate_error * 0.001f,
//                                           -roll_pid.integral_limit_rate, roll_pid.integral_limit_rate);
//         } else {
//             roll_rate_integral = 0;
//         }
        
//         // 内环微分使用误差差分（官方方法）
//         float roll_rate_derivative = roll_rate_error - last_roll_rate_error;
        
//         roll_rate_output = roll_pid.Kp_rate * roll_rate_error + 
//                           roll_pid.Ki_rate * roll_rate_integral +
//                           roll_pid.Kd_rate * roll_rate_derivative;

//         // Pitch轴内环
//         float pitch_rate_error = 0 - gyroY;
        
//         if (throttle > 480 && fabsf(pitch_rate_error) < 120.0f) {
//             pitch_rate_integral = constrain(pitch_rate_integral + pitch_rate_error * 0.001f,
//                                            -pitch_pid.integral_limit_rate, pitch_pid.integral_limit_rate);
//         } else  {
//             pitch_rate_integral = 0;
//         }
        
//         float pitch_rate_derivative = pitch_rate_error - last_pitch_rate_error;
        
//         pitch_rate_output = pitch_pid.Kp_rate * pitch_rate_error + 
//                            pitch_pid.Ki_rate * pitch_rate_integral +
//                            pitch_pid.Kd_rate * pitch_rate_derivative;

//         // 保存本次误差
//         last_roll_rate_error = roll_rate_error;
//         last_pitch_rate_error = pitch_rate_error;

//         // // 保留您的输出平滑处理
//         float max_change = throttle * 0.3f;
//         roll_rate_output = last_roll_output + 
//                            constrain(roll_rate_output - last_roll_output, -max_change, max_change);
//         pitch_rate_output = last_pitch_output + 
//                            constrain(pitch_rate_output - last_pitch_output, -max_change, max_change);
        
//         last_roll_output =  roll_rate_output;
//         last_pitch_output = pitch_rate_output;

//          for(int i = 0; i < 4; i++) {
//         motor_output_pre[i] = constrain(motor_output_pre[i], throttle, throttle*1.4);
//         motor_output[i] = motor_output_pre[i];
//     }
//         // 电机输出
//         motor_output_pre[0] = throttle + (roll_rate_output + pitch_rate_output);
//         motor_output_pre[1] = throttle + (-roll_rate_output + pitch_rate_output);
//         motor_output_pre[2] = throttle + (roll_rate_output - pitch_rate_output);
//         motor_output_pre[3] = throttle + (-roll_rate_output - pitch_rate_output);

     

//     // 遥控器控制（测试模式）
//     if(num == 3) {
//         motor_output[0] = 800;
//         motor_output[1] = 800;
//         motor_output[2] = 800;
//         motor_output[3] = 800;
//     }
//     if(num == 4) {
//         motor_output[0] = 400;
//         motor_output[1] = 400;
//         motor_output[2] = 400;
//         motor_output[3] = 400;
//     }

//     // 5. 安全保护
//     if(abs(roll_error) > 30 || abs(pitch_error) > 30 || abs(gyroX) > 120 || abs(gyroY) > 120) {
//         throttle = 0;
//         for(int i = 0; i < 4; i++) {
//             motor_output[i] = 0;
//         }
//     }

//     for(int i = 0; i < 4; i++) {
//         motor_output[i] = constrain(motor_output[i], 0, 800);
//     }
// }



//并环
// #include "head.h"
// float P_roll,test;
// extern int code,num;
// // 控制参数（基于F330机型调参）
// typedef struct {
//   float Kp_angle, Ki_angle, Kd_angle; // 角度环
//   float max_angle;                   // 最大允许倾角
//   float integral_limit;              // 积分限幅
// } PID_Params;

// int throttle;   
// float target_roll = 0, target_pitch = 0, target_Yaw = 0;  
// float target_roll_rate;
// float target_pitch_rate;
// float target_yaw_rate, last_err_yaw; 
// float target_Y_speed,target_X_speed,Kp_XSpeed=0.15,Kp_YSpeed=0.15;
// int motor_output[4] = {0};
// int motor_output_pre[4] = {0};         
// float integral_pitch=0,integral_roll=0;
// void PID_Control() {
  
   
//   PID_Params roll_pid =  {0.5f+P_roll, 0.13, 0.16, 30, 6};   // PID_Params roll_pid =  {0.49, 0.12, 0.165, 40, 6};
//   PID_Params pitch_pid = {0.5f+P_roll, 0.13 ,0.16, 30, 6};   // PID_Params pitch_pid = {0.49, 0.12 ,0.165, 40, 6};
//   PID_Params yaw_pid =   {0, 0, 0.15, 180, 5};            // PID_Params yaw_pid =   {0, 0, 0.15, 180, 5};   

//   roll_error  = Roll_error2 (target_roll, Roll);
//   pitch_error = Pitch_error2(target_pitch, Pitch);
//   Yaw_error   = Yaw_error2  (target_Yaw, Yaw);
//   RealSpeed();

//    // 抗积分饱和（倾角超限/电机饱和时禁用积分）
//    if (abs(roll_error) < roll_pid.max_angle && motor_output[0] < 650 && throttle > 450) {
//      integral_roll = constrain(integral_roll+ roll_error * 0.001, 
//                               -roll_pid.integral_limit, 
//                                roll_pid.integral_limit);
//      target_roll_rate = roll_pid.Kp_angle*roll_error + 
//                        roll_pid.Ki_angle*integral_roll + 
//                        roll_pid.Kd_angle*-gyroX;
     
    
//    } else {
//     target_roll_rate = roll_pid.Kp_angle*roll_error; // 仅比例项
    
//   }
//   // 抗积分饱和（倾角超限/电机饱和时禁用积分）
//   if (abs(pitch_error) < pitch_pid.max_angle && motor_output[0] < 650 && throttle > 450) {
//     integral_pitch = constrain(integral_pitch+ pitch_error * 0.001, 
//                              -pitch_pid.integral_limit, 
//                               pitch_pid.integral_limit);
//     target_pitch_rate = pitch_pid.Kp_angle*pitch_error + 
//                       pitch_pid.Ki_angle*integral_pitch + 
//                       pitch_pid.Kd_angle*-gyroY;
    
//   } else {
//     target_pitch_rate = pitch_pid.Kp_angle*pitch_error; // 仅比例项
//   }

//    // Yaw PID控制（新增）
//   if (abs(Yaw_error) < yaw_pid.max_angle && throttle > 550&&Z_distant>250) {
//     static float integral_yaw = 0;
//     integral_yaw = constrain(integral_yaw + Yaw_error * 0.001,
//                            -yaw_pid.integral_limit,
//                             yaw_pid.integral_limit);
   
//     target_yaw_rate = yaw_pid.Kp_angle * Yaw_error +
//                      yaw_pid.Ki_angle * integral_yaw +
//                      yaw_pid.Kd_angle * gyroZ;  // 使用陀螺仪Z轴角速度
//     last_err_yaw = Yaw_error;
    
//   target_X_speed=Kp_XSpeed*X_Speed;
//   target_Y_speed=Kp_YSpeed*Y_Speed;
//   } 
  
  

//   target_yaw_rate = constrain(target_yaw_rate, -10, 10);
//   target_X_speed = constrain(target_X_speed, -3, 3);
//   target_Y_speed = constrain(target_Y_speed, -3, 3);

//   // 3. 电机混控（四轴X模式，加入Yaw控制）
//   motor_output_pre[0] = throttle +(+target_Y_speed- target_X_speed+target_roll_rate + target_pitch_rate - target_yaw_rate)*2; // 右下
//   motor_output_pre[1] = throttle +(+target_Y_speed+ target_X_speed-target_roll_rate + target_pitch_rate + target_yaw_rate)*2; // 右上  
//   motor_output_pre[2] = throttle +(-target_Y_speed- target_X_speed+target_roll_rate - target_pitch_rate + target_yaw_rate)*2; // 左下
//   motor_output_pre[3] = throttle +(-target_Y_speed+ target_X_speed-target_roll_rate - target_pitch_rate - target_yaw_rate)*2; // 左上

//      for(int i = 0; i < 4; i++) {
//      motor_output_pre[i] = constrain(motor_output_pre[i], throttle, throttle*1.3);
//      motor_output[i] =motor_output_pre[i];
//    }

//   //遥控器控制
//       if(num==3)
//    {
//   motor_output[0]=800;
//   motor_output[1]=800;
//   motor_output[2]=800;
//   motor_output[3]=800;
//    }
//     if(num==4)
//    {

//    motor_output[0]=400;
//    motor_output[1]=400;
//    motor_output[2]=400;
//    motor_output[3]=400;
//    }

//   // 5. 安全保护
//  if(abs(roll_error) > 30 || abs(pitch_error) > 30 || abs(gyroX)>120 || abs(gyroY)>120) 
// {
//   throttle = 0;
//   for(int i = 0; i < 4; i++)
//   {
//     motor_output[i] = 0;
//   }
// }

// for(int i = 0; i < 4; i++) {
//     motor_output[i] = constrain(motor_output[i], 0, 800);
// }
// }
  