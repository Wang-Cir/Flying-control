#include "imu.h"


/**
 * IMUClass类的构造函数
 * 该构造函数用于初始化IMU（惯性测量单元）传感器的SPI通信
 * 
 * @param sck SPI时钟线引脚编号
 * @param miso SPI数据输入线（主设备输入，从设备输出）引脚编号
 * @param mosi SPI数据输出线（主设备输出，从设备输入）引脚编号
 * @param cs SPI芯片选择线引脚编号
 */
IMUClass::IMUClass(int sck, int miso, int mosi, int cs)
    : customSPI(HSPI), // 初始化自定义SPI通信对象
      imuSensor(customSPI, cs), // 初始化IMU传感器对象
      CUSTOM_SPI_SCK(sck), // 设置SPI时钟线引脚
      CUSTOM_SPI_MISO(miso), // 设置SPI数据输入线引脚
      CUSTOM_SPI_MOSI(mosi), // 设置SPI数据输出线引脚
      CUSTOM_SPI_CS(cs), // 设置SPI芯片选择线引脚
      lastUpdate(0) {} // 初始化 lastUpdate

bool IMUClass::begin()
{
    // 初始化自定义SPI
    customSPI.begin(CUSTOM_SPI_SCK, CUSTOM_SPI_MISO, CUSTOM_SPI_MOSI, CUSTOM_SPI_CS);

    // 开始与IMU通信
    int status = imuSensor.begin();
    if (status < 0)
    {
        return false; // 初始化失败
    }
    lastUpdate = micros(); // 初始化 lastUpdate 为当前微秒时间
    return true;           // 初始化成功
}

void IMUClass::update()
{
    // 读取传感器数据
    imuSensor.getAGT();

    // 保存加速度和角速度数据到成员变量
    accX = imuSensor.accX();
    accY = imuSensor.accY();
    accZ = imuSensor.accZ();
    gyrX = imuSensor.gyrX() - gyrXBias; // 减去零漂值
    gyrY = imuSensor.gyrY() - gyrYBias;
    gyrZ = imuSensor.gyrZ() - gyrZBias;
    temp = imuSensor.temp();
}

void IMUClass::getAccData(float &ax, float &ay, float &az)
{
    ax = accX;  ay = accY;  az = accZ;  // 将数据保存到参数中
}

void IMUClass::getGyrData(float &gx, float &gy, float &gz)
{
    gx = gyrX;  gy = gyrY;  gz = gyrZ;  // 将数据保存到参数中
}

// 获取传感器数据
float IMUClass::getAccX() const { return accX; }
float IMUClass::getAccY() const { return accY; }
float IMUClass::getAccZ() const { return accZ; }
float IMUClass::getGyrX() const { return gyrX; }
float IMUClass::getGyrY() const { return gyrY; }
float IMUClass::getGyrZ() const { return gyrZ; }
float IMUClass::getTemp() const { return temp; }

// 计算时间间隔
float IMUClass::deltatUpdate()
{
    unsigned long Now = micros();
    float deltat = (Now - lastUpdate) / 1000000.0f; // 计算自上次更新以来的时间间隔（秒）
    lastUpdate = Now;                               // 更新 lastUpdate
    return deltat;                                  // 返回时间间隔, 单位秒
}

float IMUClass::getDeltat() const { return deltat; }

// 计算 pitch 和 roll
float IMUClass::getPitch()
{
    pitch = atan2(accY, accZ) * 180 / PI;
    return pitch;
}

float IMUClass::getRoll()
{
    roll = atan2(accX, accZ) * 180 / PI;
    return roll;
}

float IMUClass::getYaw()
{
    deltat = deltatUpdate(); // 更新时间间隔
    yaw += (gyrZ) * deltat;    // 陀螺仪数据单位是度每秒，需要乘以时间间隔（毫秒）得到角度变化
    if (yaw < 0)
        yaw += 360; // 保证 yaw 在 0 到 360 度之间
    if (yaw >= 360)
        yaw -= 360;
    return yaw;
}

void IMUClass::getPitchRollYaw(float &pitch, float &roll, float &yaw)
{
    pitch = getPitch();
    roll = getRoll();
    yaw = getYaw();
}

void IMUClass::calculateGyrBias(int numSamples, int sec)
{
    // 计算 gyrX, gyrY, gyrZ 的零漂值的代码
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    for (int i = 0; i < numSamples; ++i)
    {
        update();
        Serial.printf("measuring... gyrX = %f, gyrY = %f, gyrZ = %f \n", gyrX, gyrY, gyrZ);
        sumX += gyrX;
        sumY += gyrY;
        sumZ += gyrZ;
        delay(10); // 稍作延迟以获取稳定的数据
    }

    gyrXBias = sumX / numSamples;
    gyrYBias = sumY / numSamples;
    gyrZBias = sumZ / numSamples;

    Serial.printf("gyrX Bias = %f \n", gyrXBias);
    Serial.printf("gyrY Bias = %f \n", gyrYBias);
    Serial.printf("gyrZ Bias = %f \n", gyrZBias);

    delay(sec*1000); // 等待3秒以查看零漂值
}