#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

// WiFi连接所需的SSID和密码（外部声明，在cpp文件中定义）
extern const char* ssid;
extern const char* password;

// 热点模式的SSID和密码（外部声明，在cpp文件中定义）
extern const char* ssid_OUTPUT;
extern const char* password_OUTPUT;

// WiFi连接函数声明
void WiFi_connect();
void HTTP_connect();

#endif  // HEAD_H