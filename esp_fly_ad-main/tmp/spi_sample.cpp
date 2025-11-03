/* ESP32 有四个 SPI 总线，但目前只有两个可用：HSPI 和 VSPI。简单地使用 SPI API（如 Arduino 示例中所示）会使用 VSPI，而 HSPI 将保持未使用状态。
 *
 * 然而，如果我们为这两个总线分别初始化两个 SPI 类的实例，则可以同时使用它们。但是，仅使用 Arduino 方式时，实际上一次只会有一个总线在输出。
 *
 * 逻辑分析仪捕获的图像与本示例在同一文件夹中，文件名为 "multiple_bus_output.png"
 *
 * 创建者：Alistair Symonds
 * 创建日期：2018年4月30日
 */
#include <Arduino.h>
#include <SPI.h>

// 定义 ALTERNATE_PINS 以使用非标准 GPIO 引脚配置 SPI 总线

#ifdef ALTERNATE_PINS
#define VSPI_MISO 2
#define VSPI_MOSI 4
#define VSPI_SCLK 0
#define VSPI_SS   33

#define HSPI_MISO 26
#define HSPI_MOSI 27
#define HSPI_SCLK 25
#define HSPI_SS   32
#else
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SCLK SCK
#define VSPI_SS   SS

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   15
#endif

#if !defined(CONFIG_IDF_TARGET_ESP32)
#define VSPI FSPI
#endif

static const int spiClk = 1000000;  // 1 MHz

// 未初始化的 SPI 对象指针
SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

void setup() {
  // 初始化两个 SPIClass 实例，分别连接到 VSPI 和 HSPI
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  // 时钟、MISO、MOSI、SS 引脚

#ifndef ALTERNATE_PINS
  // 使用默认引脚初始化 vspi
  // SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
#else
  // 或者通过选择的 GPIO 引脚路由
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);  // SCLK, MISO, MOSI, SS
#endif

#ifndef ALTERNATE_PINS
  // 使用默认引脚初始化 hspi
  // SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin();
#else
  // 或者通过选择的 GPIO 引脚路由
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);  // SCLK, MISO, MOSI, SS
#endif

  // 设置从设备选择引脚为输出模式，因为 Arduino API 不会自动拉低 SS 引脚
  pinMode(vspi->pinSS(), OUTPUT);  // VSPI SS
  pinMode(hspi->pinSS(), OUTPUT);  // HSPI SS
}

// 循环函数，直到断电或复位一直重复执行
void loop() {
  // 使用 SPI 总线
  spiCommand(vspi, 0b01010101);  // 发送一些垃圾数据以说明用法
  spiCommand(hspi, 0b11001100);
  delay(100);
}

void spiCommand(SPIClass *spi, byte data) {
  // 按照常规 Arduino SPI API 的方式使用
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW);  // 拉低 SS 引脚以准备传输
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH);  // 拉高 SS 引脚以表示传输结束
  spi->endTransaction();
}