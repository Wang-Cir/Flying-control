#include "ICM42688.h"

ICM42688 imu(9);  // CS引脚为9

void setup() {
  Serial.begin(115200);
  imu.initialize();
}

void loop() {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;

  imu.readAccelerometer(accelX, accelY, accelZ);
  imu.readGyroscope(gyroX, gyroY, gyroZ);
  temperature = imu.readTemperature();

  Serial.print("Acceleration (G): X=");
  Serial.print(accelX, 2);
  Serial.print(", Y=");
  Serial.print(accelY, 2);
  Serial.print(", Z=");
  Serial.print(accelZ, 2);
  Serial.println();

  Serial.print("Gyroscope (DPS): X=");
  Serial.print(gyroX, 2);
  Serial.print(", Y=");
  Serial.print(gyroY, 2);
  Serial.print(", Z=");
  Serial.print(gyroZ, 2);
  Serial.println();

  Serial.print("Temperature (C): ");
  Serial.println(temperature, 2);

  delay(1000);
}