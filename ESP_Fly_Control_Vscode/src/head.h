#ifndef __HEAD_H
#define __HEAD_H

#define BUTTON 14
#define BUTTON1 25
#define LED 2
#define LED1 4

extern volatile int i;

#include <Arduino.h>
#include <WiFi.h>
#include "timer.h"
#include "WiFi_connect.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "oled.h"
#include "pwm.h"
#include "servo.h"
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include "pid.h"
#include "optflow.h"

// 光流传感器外部声明
extern uint32_t tof_distance;
extern uint8_t tof_strength;
extern uint8_t tof_precision;
extern uint8_t tof_status;
extern int16_t flow_vel_x;
extern int16_t flow_vel_y;
extern uint8_t flow_quality;
extern uint8_t flow_status;
extern uint32_t flow_packet_count;
extern uint32_t flow_error_count;

// 函数声明
void optical_flow_init();
void onOpticalFlowData();

#endif