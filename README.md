🚀 项目概述
这是一个完整的四旋翼飞控解决方案，主要特点：

双核多任务架构：使用FreeRTOS实现并行处理

三环PID控制：角度环+角速度环+光流速度环

安全保护机制：倾角过大保护、角速度过大保护、高度限制

多种传感器融合：JY62、光流传感器

无线控制：蓝牙遥控+状态显示

📁 代码结构
核心文件
text
├── main.cpp              # 主程序，任务创建和初始化
├── head.h               # 头文件包含和全局变量声明
├── pid_control.cpp      # PID控制算法实现
└── [其他传感器驱动文件]
任务架构
1. 遥控数据显示任务 (remoteDataTask)
优先级: 1

周期: 20ms

功能:

OLED实时显示电机状态和飞行数据

蓝牙指令解析和处理

油门控制和目标速度设置

2. IMU控制任务 (imuControlTask)
优先级: 5 (更高优先级)

周期: 1ms

功能:

姿态解算和PID控制计算

电机PWM输出

飞行安全保护检测

🔧 控制系统设计
PID控制结构
text
光流速度环(外环) → 角度环(中环) → 角速度环(内环)
1. 速度环（光流传感器）
控制X/Y方向速度

输出目标滚转/俯仰角度

参数: Kp_speed_x = 0.15, Kp_speed_y = 0.15

2. 角度环
控制飞机姿态角度

输出目标角速度

参数: Kp_angle = 0.52, Ki_angle = 0.13, Kd_angle = 0.17

3. 角速度环
控制电机直接输出

实现快速响应

电机混控算法
cpp
motor_output[0] = throttle + (target_roll_rate + target_pitch_rate - target_yaw_rate) * 2;  //右下

motor_output[1] = throttle + (-target_roll_rate + target_pitch_rate + target_yaw_rate) * 2; //右上 

motor_output[2] = throttle + (target_roll_rate - target_pitch_rate + target_yaw_rate) * 2;  //左下

motor_output[3] = throttle + (-target_roll_rate - target_pitch_rate - target_yaw_rate) * 2; //左上

🛡️ 安全保护机制
1. 倾角保护
cpp
if(abs(roll_error)>30 || abs(pitch_error)>30) {
    fly_protect = 0; // 立即切断电机
}
2. 角速度保护
cpp
if(abs(gyroX)>120 || abs(gyroY)>120) {
    fly_protect = 0; // 陀螺仪异常保护
}
3. 高度限制
cpp
if(Z_distant > 1500) {
    fly_protect = 2; // 限高保护，逐渐降低动力
}
📊 蓝牙控制指令
按键	功能
1	油门增加
2	油门减少，速度归零
3	电机全速测试
4	电机低速测试
5	缓降模式
6	Y方向正速度
7	Y方向负速度
8	X方向正速度
9	X方向负速度
⚙️ 硬件配置
主要组件
主控: ESP32 (240MHz)

传感器: MPU6050 (陀螺仪+加速度计)

定位: 光流传感器 + 超声波

显示: OLED屏幕

通信: 蓝牙模块

执行器: 4×无刷电机 + ESC

PWM通道分配
cpp
CHANNEL0 → 右上电机
CHANNEL1 → 右下电机  
CHANNEL2 → 左上电机
CHANNEL3 → 左下电机
🔄 实时性能优化
多任务同步
使用互斥锁(xMutex)保护共享数据

关键数据快速拷贝到局部变量

I2C操作在锁外执行，避免阻塞

时序保证
IMU控制任务: 1ms高优先级

显示和遥控任务: 20ms普通优先级

CPU频率锁定240MHz确保稳定性

🎯 调参指南
姿态环参数
cpp
PID_Params roll_pid = {0.52f, 0.13, 0.17, 30, 6};
PID_Params pitch_pid = {0.52f, 0.13, 0.17, 30, 6};
PID_Params yaw_pid = {-0.02, -0.02, 0.2, 180, 5};
飞行模式
手动模式: 油门 < 520，直接姿态控制

定高模式: 油门 > 520，启用光流速度控制

测试模式: 通过蓝牙指令激活

🚨 注意事项
首次飞行: 在安全环境下进行，确保保护机制正常工作

参数调整: 逐个参数微调，避免大幅改动

电池监控: 确保供电稳定，电压充足

校准流程: 飞行前进行传感器校准

📈 扩展功能
可选的增强功能：

WiFi图传和数据遥测

GPS定位和自主航线

机器视觉目标跟踪

手机APP地面站

这个飞控系统提供了稳定可靠的四旋翼控制基础，适合学术研究和无人机开发项目使用。
