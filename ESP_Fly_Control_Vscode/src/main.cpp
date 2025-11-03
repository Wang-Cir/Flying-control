#include "head.h"

BluetoothSerial BT;

volatile int i,fly_protect=1;
int code,num,led_set;

// 创建互斥锁用于保护共享资源
SemaphoreHandle_t xMutex;
//运行蓝牙wifi等外设
void remoteDataTask(void *parameter) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 20; // 20ms周期
    
    // 定义局部变量用于存储显示数据
    int local_motor_output[4],local_throttle;
    float local_P_roll;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 第一步：快速拷贝数据（锁内，时间很短）
        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            // 快速复制共享数据到局部变量
            memcpy(local_motor_output, motor_output, sizeof(local_motor_output));
            local_P_roll = P_roll;
           local_throttle=throttle;
          xSemaphoreGive(xMutex); // 立即释放锁
        }
          
        // 第二步：OLED显示（锁外，耗时操作）
        pm.clearDisplay();
        pm.setTextColor(SSD1306_WHITE);
        pm.setTextSize(2);
        pm.setCursor(0,0);
        pm.println(local_motor_output[3]);   //左上旋翼
        pm.setCursor(64,0);
        pm.println(local_motor_output[1]);   //右上旋翼
        pm.setCursor(64,48); 
        pm.println(local_motor_output[0]);   //右下旋翼
        pm.setCursor(0,48);  
        pm.println(local_motor_output[2]);   //左下旋翼
        pm.setCursor(0,24);
        pm.println(target_roll);          
        pm.setCursor(64,24);
        pm.println(target_X_speed);         
       
        pm.display(); // 刷新显示
     
        // 蓝牙数据处理
        if(BT.available()) {
            code = BT.read();
            if(code >= '0' && code <= '9') {
                num = code - '0';
            }
            
            // 只有修改共享变量时需要加锁
            if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                switch(num) {
                    case 1:
                        throttle++;
                        throttle = constrain(throttle, 410, 600);
                        break;
                    case 2:
                        throttle--;
                        throttle = constrain(throttle, 410, 600);
                        target_X_speed=0;
                        target_Y_speed=0;
                        break;
                }
                xSemaphoreGive(xMutex);
            }
        }
    }
}
//运行pid平衡算法
void imuControlTask(void *parameter) {

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1; //1ms周期
    xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
       if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {

        if(abs(roll_error)>30 ||abs(pitch_error)>30||abs(gyroX)>120||abs(gyroY)>120 ) {fly_protect=0;}
        if(Z_distant>1500) {fly_protect=2;}
               if(fly_protect==0)
               {  
                 throttle = 0; // 倾倒保护
                ledcWrite(CHANNEL2, 0);   //左上
                ledcWrite(CHANNEL0, 0);   //右上
                ledcWrite(CHANNEL1, 0);   //右下
                ledcWrite(CHANNEL3, 0);   //左下 
                for(int i = 0; i < 4; i++) {
                   motor_output[i] = 0; }
                   digitalWrite(LED, 1);
              }
              if(fly_protect==2)   //限高保护
              {
                for(int i = 0; i < 4; i++) {
                   motor_output[i]-=0.001;}
                ledcWrite(CHANNEL2, motor_output[3]);   //左上
                ledcWrite(CHANNEL0, motor_output[1]);   //右上
                ledcWrite(CHANNEL1, motor_output[0]);   //右下
                ledcWrite(CHANNEL3, motor_output[2]);   //左下
              }
                if(fly_protect==1)
               { 
                PID_Control();
                // 电机输出
                ledcWrite(CHANNEL2, motor_output[3]);   //左上
                ledcWrite(CHANNEL0, motor_output[1]);   //右上
                ledcWrite(CHANNEL1, motor_output[0]);   //右下
                ledcWrite(CHANNEL3, motor_output[2]);   //左下
              }

      
                xSemaphoreGive(xMutex);
          
        }
    }
}

void setup() {
    BT.begin("ESP32_Device");
    pinMode(BUTTON, INPUT_PULLDOWN);
    pinMode(BUTTON1, INPUT_PULLDOWN);
    pinMode(LED, OUTPUT);
    servo_init();
    optflow_init();
    OLED_init(); 
    PWM_init(); 
    Serial.begin(9600);
    int target_freq = 240;
    setCpuFrequencyMhz(target_freq); 
    int current_freq = getCpuFrequencyMhz(); 
    Serial.print("当前CPU主频：");
    Serial.print(current_freq);
    Serial.println(" MHz");

    // 创建互斥锁
    xMutex = xSemaphoreCreateMutex();
    
    if(xMutex == NULL) {
        Serial.println("互斥锁创建失败!");
    }

    // 创建任务
    xTaskCreate(
        remoteDataTask,        // 任务函数
        "RemoteData",          // 任务名称
        4096,                  // 堆栈大小
        NULL,                  // 参数
        1,                     // 优先级
        NULL                   // 任务句柄
    );

    xTaskCreate(
        imuControlTask,        // 任务函数
        "IMUControl",          // 任务名称
        4096,                  // 堆栈大小
        NULL,                  // 参数
        5,                     // 优先级（比RemoteData高）
        NULL                   // 任务句柄
    );

    Serial.println("任务创建完成，系统启动...");
}
void loop() {

    delay(1000); // 防止看门狗复位
}









