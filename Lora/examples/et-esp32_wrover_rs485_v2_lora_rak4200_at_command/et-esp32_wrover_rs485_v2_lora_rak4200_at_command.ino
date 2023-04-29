/*******************************************************************************
 * ET-ESP32(WROVER) RS485 V2 
 * Tools->Board:"ESP32 Wrover Module"
 *******************************************************************************
 * I2C Interface & I2C Bus
 * -> IO22                = I2C_SCL
 * -> IO21                = I2C_SDA
 * -> I2C RTC:DS3231      = I2C Address : 0x68:1100100(x)
 * -> I2C EEPROM 24LC16   = I2C Address : 0x50:1010000(x)
 * -> I2C ADC MCP3423     = I2C Address : 0x6D:1100101(x)
 * -> I2C Sensor:BME280   = I2C Address : 0x76:1110110(x)
 * -> I2C Sebsor:SHT31    = I2C Address : 0x44:1000100(x)/0x45:1010101(x)
 * SPI Interface SD Card
 * -> SD_CS               = IO4
 * -> SPI_MISO            = IO19
 * -> SPI_MOSI            = IO23
 * -> SPI_SCK             = IO18
 * UART2 RS485 Half Duplex Auto Direction
 * -> IO26                = RX2
 * -> IO27                = TX2
 * User Switch
 * -> IO36                = USER_SW
 * RTC Interrupt
 * -> IO39                = RTC_INT#
 *******************************************************************************/
 
//=================================================================================================
#include <Wire.h> 
//=================================================================================================

//=================================================================================================
// Start of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================
// Remap Pin USART -> C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//                    C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug           Serial                                                              // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN    16
#define SerialRS485_TX_PIN    17
#define SerialRS485           Serial2                                                             // Serial2(IO16=TXD,IO17=RXD)
//=================================================================================================
#define SerialLora_RX_PIN     14
#define SerialLora_TX_PIN     27
#define SerialLora            Serial1                                                             // Serial1(IO27=TXD,IO14=RXD)
//=================================================================================================
#define LORA_RES_PIN          33                                                                  // ESP32-WROVER :IO33(LoRa-RESET)
#define LORA_RES_PRESS        LOW
#define LORA_RES_RELEASE      HIGH
//=================================================================================================
#define I2C_SCL_PIN           22                                                                  // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN           21                                                                  // ESP32-WROVER : IO21(SDA1)
//=================================================================================================
#define LED_PIN               2                                                                   // ESP-WROVER  : IO2
#define LedON                 1
#define LedOFF                0
//=================================================================================================
#define USER_SW_PIN           36                                                                  // ESP32-WROVER :IO36
#define SW_PRESS              LOW
#define SW_RELEASE            HIGH 
//=================================================================================================
#define RTC_INT_PIN           39                                                                  // ESP32-WROVER :IO39
#define RTC_INT_ACTIVE        LOW
#define RTC_INT_DEACTIVE      HIGH 
//=================================================================================================
// End of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================

//=================================================================================================
String ack = "";
//=================================================================================================

void setup() 
{
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LedOFF);
  //===============================================================================================
  pinMode(USER_SW_PIN,INPUT_PULLUP);
  pinMode(RTC_INT_PIN,INPUT_PULLUP);
  //===============================================================================================
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);                                                      
  //===============================================================================================
  SerialDebug.begin(115200);
  while(!SerialDebug);
  //===============================================================================================
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V2.....Ready");
  //===============================================================================================
  //===============================================================================================
  SerialDebug.println("Start Config RAK4200...");
  //===============================================================================================
  
  //===============================================================================================
  SerialLora.begin(115200, SERIAL_8N1, SerialLora_RX_PIN, SerialLora_TX_PIN);
  while(!SerialLora);
  //===============================================================================================
  pinMode(LORA_RES_PIN, OUTPUT);
  digitalWrite(LORA_RES_PIN, LORA_RES_RELEASE);
  //===============================================================================================
  digitalWrite(LORA_RES_PIN, LORA_RES_PRESS);                                                     // active Reset
  delay(1000);
  digitalWrite(LORA_RES_PIN, LORA_RES_RELEASE);                                                   // release Reset
  //===============================================================================================
}

void loop() 
{
  //===============================================================================================
  // test at command initial RAK4200 Lora P2P Mode Sender
  // 1. Initial Mode = P2P Mode
  //    send>at+set_config=lora:work_mode:1<Cr><Lf>
  //    recv>LoRa (R) is a registered trademark or service mark of Semtech Corporation or its affiliates. LoRaWAN (R) is a licensed mark.
  //         
  //         RAK4200 version:3.2.0.15
  //         UART1 work mode: RUI_UART_NORMAL, 115200, N81
  //         UART2 work mode: RUI_UART_NORMAL, 115200, N81
  //         Current work_mode:P2P
  //         LoRa P2P Transfer_mode: Sender
  //         Initialization OK
  // 2. Initial P2P parameter
  //    send>at+set_config=lorap2p:923000000:12:0:1:8:20<Cr><Lf>
  //    recv>OK
  // 3. Initial P2P = Sender
  //    send>at+set_config=lorap2p:transfer_mode:2<Cr><Lf>
  //    recv>OK
  // 4. P2P Sens Data
  //    send>at+send=lorap2p:1234<Cr><Lf>
  //    recv>OK
  //===============================================================================================
  // test at command initial RAK4200 Lora P2P Mode Receiver
  // 1. Initial Mode = P2P Mode
  //    send>at+set_config=lora:work_mode:1<Cr><Lf>
  //    recv>LoRa (R) is a registered trademark or service mark of Semtech Corporation or its affiliates. LoRaWAN (R) is a licensed mark.
  //
  //         RAK4200 version:3.2.0.15
  //         UART1 work mode: RUI_UART_NORMAL, 115200, N81
  //         UART2 work mode: RUI_UART_NORMAL, 115200, N81
  //         Current work_mode:P2P
  //         LoRa P2P Transfer_mode: Sender
  //         Initialization OK
  // 2. Initial P2P parameter
  //    send>at+set_config=lorap2p:923000000:12:0:1:8:20<Cr><Lf>
  //    recv>OK
  // 3. Initial P2P = Receive
  //    send>at+set_config=lorap2p:transfer_mode:1<Cr><Lf>
  //    recv>OK
  //===============================================================================================
  if(SerialDebug.available()) 
  {
    SerialLora.write(SerialDebug.read());
  }
  //===============================================================================================

  //===============================================================================================
  //
  //===============================================================================================
  if(SerialLora.available()) 
  {
    SerialDebug.write(SerialLora.read());
  }
  //===============================================================================================
}
