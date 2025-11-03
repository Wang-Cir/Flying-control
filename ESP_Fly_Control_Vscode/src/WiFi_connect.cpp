#include <head.h>

const char * ssid = "Xiaomi 14 Pro";
const char * password = "66668888";

const char * ssid_OUTPUT = "ESP_32";
const char * password_OUTPUT = "66668888";

String url = "http://apis.juhe.cn/simpleWeather/query";
String city = "惠州";
String key = "fc292127f7e08f890a0479487389d653";

void WiFi_connect(){

//连接WiFi
WiFi.begin(ssid,password);

//检查是否连接成功
Serial.print("正在连接wifi");

while(WiFi.status()!=WL_CONNECTED){
  delay(500);
  Serial.print(".");
}

Serial.print("连接成功");
Serial.print("IP地址：");
Serial.print(WiFi.localIP());

if(WiFi.status()==WL_CONNECTED)
digitalWrite(LED,HIGH);

/* 热点设置
WiFi.softAP(ssid_OUTPUT,password_OUTPUT);
Serial.print("热点IP：");
Serial.print(WiFi.softAPIP()); */
}


void HTTP_connect(){

  //创建httpclient对象
  HTTPClient http;
  //指定访问 URL
  http.begin(url+"?city="+city+"&key="+key);
  //接收http响应状态码
  int http_code = http.GET();
  Serial.printf("HTTP 状态码:%d\n",http_code);
  //获取响应正文
  String response = http.getString();
  Serial.print("响应数据：");
  Serial.println(response);
  //关闭连接
  http.end();
  //创建对象
  DynamicJsonDocument doc(1024);
  //解析json数据
  deserializeJson(doc,response);
  //从解析的json中获取值
  int temp = doc["result"]["realtime"]["temperature"].as<int>();
  String info = doc["result"]["realtime"]["info"].as<String>();
  int aqi=doc["result"]["realtime"]["aqi"].as<int>();

  Serial.printf("	温度: %d,天气：%s,空气指数: %d\n",temp,info,aqi);
  
}

  // WiFi_connect();
  // HTTP_connect();