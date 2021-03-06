// DHT_Weather-ThingSpeak-WifiManager-LCD includes

//==== WiFi Manager Includes ===============================
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <WiFiUdp.h>              // For NTP service
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager.git
//for LED status
#include <Ticker.h>

// This is actually managed by WifiManager now
const char* ssid = "ssid";
const char* password = "password";

//==== BME280 Includes =====================================
//  https://github.com/finitespace/BME280
#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>                 // Needed for legacy versions of Arduino.

//==== ThingSpeak Includes =================================
#include "ThingSpeak.h"
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

// ThingSpeak information
char * myChannelID = "123456";
char * myWriteAPIKey = "01234567890";

//==== 2.4in TFT LCD Includes ==============================
//  Setup latest Adafruit_GFX, Adafruit_ILI9341 and XPT2046_Touchscreen Library first:
//  
//  https://github.com/adafruit/Adafruit-GFX-Library
//  https://github.com/adafruit/Adafruit_ILI9341
//  https://github.com/PaulStoffregen/XPT2046_Touchscreen

#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS D0  //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TFT_DC D8  //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TFT_RST -1 //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TS_CS D3   //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)

// #define TFT_CS 14  //for D32 Pro
// #define TFT_DC 27  //for D32 Pro
// #define TFT_RST 33 //for D32 Pro

//==== Additional fonts ====================================
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

// #define TS_CS  12  //for D32 Pro



/* ==== General Defines ==== */
#define SERIAL_BAUD 115200
#define WIFIRESETBUTTON D3
/* ==== END Defines ==== */
