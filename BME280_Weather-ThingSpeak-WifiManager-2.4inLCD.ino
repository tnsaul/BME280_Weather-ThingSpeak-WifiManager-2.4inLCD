#include "Arduino.h"
//==========================================================
//
// Change History:
// v1 First cut from existing other code.  Using NodeMCU 8266 Board.
// v2 Forked from BME280_Weather-ThingSpeak-WifiManager-LCD Project and
//    replaced 2 Line LCD with LOLIN2.4in TCT LCD and changed to D1 Mini.
// v3 20190413 Minor updates; changed pressure to hPa.
// v4 Move to PlatformIO for development
// v5 Add TouchScreen basics
// v6 Adding Filesystem for storing ThingSpeak information rather than hard-coding it
// v7 Updated BME280 library to v3.0 with necessary alterations.
//
//==========================================================
//  D1 Mini GPIO Pins:
//    Pin   Function                        ESP-8266 Pin
//    TX    TXD                             TXD
//    RX    RXD                             RXD
//    A0    Analog input, max 3.3V input    A0
//    D0    IO                              GPIO16
//    D1    IO, SCL                         GPIO5
//    D2    IO, SDA                         GPIO4
//    D3    IO, 10k Pull-up                 GPIO0
//    D4    IO, 10k Pull-up, BUILTIN_LED    GPIO2
//    D5    IO, SCK                         GPIO14
//    D6    IO, MISO                        GPIO12
//    D7    IO, MOSI                        GPIO13
//    D8    IO, 10k Pull-down, SS           GPIO15
//    G     Ground                          GND
//    5V    5V                              -
//    3V3   3.3V                            3.3V
//    RST   Reset                           RST
//    All of the IO pins have interrupt/pwm/I2C/one-wire support except D0.
//    All of the IO pins run at 3.3V.
//==========================================================


#include "BME280_Weather-ThingSpeak-WifiManager-LCD.h"


//  Some or much of this code is from BME280 I2C Test.ino
//  This code shows how to record data from the BME280 environmental sensor
//  using I2C interface. This file is an example file, part of the Arduino
//  BME280 library.
//  Copyright (C) 2016  Tyler Glenn
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//  
//  Written: Dec 30 2015.
//  Last Updated: Sep 19 2016.
//  
//  Connecting the BME280 Sensor:
//  Sensor              ->  Board
//  -----------------------------
//  Vin (Voltage In)    ->  3.3V
//  Gnd (Ground)        ->  Gnd
//  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
//  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
//==========================================================
//  WiFi Manager: How It Works
//
//  when your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
//  if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a 
//  DNS and WebServer (default ip 192.168.4.1)
//  using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
//  because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get 
//  any domain you try to access redirected to the configuration portal
//  choose one of the access points scanned, enter password, click save
//  ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.



//===============================================================
//    1. !!!Make sure you using lastest ESP8266 core for Arduino, otherwise it may not work properly.
//      https://github.com/esp8266/Arduino
//      
//      (The pin D0(GPIO16) will be not work when you use older version ESP8266 core for Arduino, 
//      because the older version ESP8266 core for Arduino's digitalPinToBitMask(), portOutputRegister(), 
//      portInputRegister() and portModeRegister() fuction have some bugs which Adafruit_ILI9341 Library will use.
//      This bug was fixed after commit:  https://github.com/esp8266/Arduino/commit/799193888a553de8876052019842538396f92194 )
//        
//    
//    2. Setup latest Adafruit_GFX, Adafruit_ILI9341 and XPT2046_Touchscreen Library first:
//    
//        https://github.com/adafruit/Adafruit-GFX-Library
//    
//        https://github.com/adafruit/Adafruit_ILI9341
//    
//        https://github.com/PaulStoffregen/XPT2046_Touchscreen


//====  END Includes =======================================

/* ==== BME280 Global Variables ==== */
  /* Recommended Modes -
    Based on Bosch BME280I2C environmental sensor data sheet.

  Weather Monitoring :
    forced mode, 1 sample/minute
    pressure ×1, temperature ×1, humidity ×1, filter off
    Current Consumption =  0.16 μA
    RMS Noise = 3.3 Pa/30 cm, 0.07 %RH
    Data Output Rate 1/60 Hz
  */

  BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
  );


  const uint32_t I2C_ACK_TIMEOUT = 2000;

  //#define USING_BRZO
  //BME280I2C bme;
  BME280I2C bme(settings);  //Use the defaults
  bool metric = true;
  float temperature(NAN), humidity(NAN), pressure(NAN);
  float otemperature(NAN), ohumidity(NAN), opressure(NAN);

/* ==== BME280 WiFiManager Global Variables ==== */
  Ticker ticker;

/* ==== NTP  Global Variables ==== */
  unsigned int localPort = 2390;            // local port to listen for UDP packets
  
  // Don't hardwire the IP address or we won't get the benefits of the pool.
  // Lookup the IP address for the host name instead
  // IPAddress timeServer(129, 6, 15, 28);  // time.nist.gov NTP server
  IPAddress timeServerIP;                   // time.nist.gov NTP server address
  const char* ntpServerName = "time.nist.gov";
  const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
  const long TZOFFSET = 10 * 3600;          // TNS: Rough aproach to Timezone offset in seconds
                                            // for Austrlian EST ie GMT+10 hours
  byte packetBuffer[ NTP_PACKET_SIZE];      //buffer to hold incoming and outgoing packets
  
  WiFiUDP udp;                              // A UDP instance to let us send and receive packets over UDP

/* ====  ThingSpeak Global Variables ==== */
  WiFiClient client;                        // Need this for ThingSpeak?
  uint32_t delayMS;
  // Set up some time variables
  // 2^32 -1 - gives me about 24 days before it rolls over.
  unsigned long oldTime, newTime, clockTime;
  // 15 minutes = 1000 x 60 x 15 = 300000
  #define THINGSPEAKDELAY 900000            // This is how long between measures.

/* ====  WiFiManager Variables ==== */
  // Really not pretty passing around globals like this.  Should review and do as pointers etc.
  WiFiManager wifiManager;

/* ==== 2.4in TFT LCD Display Variables ==== */
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TS_CS);  
  
/* ==== END Global Variables ==== */





//==========================================================
//===== SETUP ==============================================
//==========================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {} // Wait

  // Start the TFT Display
  tft.begin();
  displayDiagnostics();
  tft.setFont(&FreeSans9pt7b);
  displayMakeBlack();
  tft.setRotation(3);
  tft.setCursor(0, 19);

  // Start the TouchScreen
  ts.begin();
  ts.setRotation(3);

  // Start the BME280 Sensor
  // Use the template begin(int SDA, int SCL);
  // SDA = D2 = GPIO4
  // SCL = D1 = GPIO5
  //brzo_i2c_setup(SDA,SCL,I2C_ACK_TIMEOUT);
  Wire.begin();
  while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");    
    tft.println("Could not find BME280 sensor! v7.4");
    delay(1000);
  }

  // starttime = millis();//get the current time;

  // set a button input to know when to reset the WiFi
  //pinMode(WIFIRESETBUTTON, INPUT);
  
  // set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);

  // Print a message to the LCD
  tft.println("Starting! 7.4");

    
  // This loops until the WIFI is configured
  tft.println("Configuring WiFi...");
  tft.println("Check WebServer (default ip 192.168.4.1) if no WiFi connection.");
  // Try and get any saved credentials
  getThingsSpeakCreds();
  
  // id/name, placeholder/prompt, default, length
  WiFiManagerParameter thingspeakchannelID("channelID", "ThingSpeak Channel ID", myChannelID, 10);
  wifiManager.addParameter(&thingspeakchannelID);          
  WiFiManagerParameter thingspeakchannelAPIKey("APIKey", "ThingSpeak API Key", myWriteAPIKey, 20);
  wifiManager.addParameter(&thingspeakchannelAPIKey);   
  
  configureWIFI(false);

//  tft.clear();
  tft.println("Connected to:");
  tft.println(WiFi.localIP()); 

  // StartUDP handler for NTP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.remotePort());
  // Wait so we can read it!
  delay(5000);  

  // Instantiate ThingSpeak
  ThingSpeak.begin(client);

  // Note when we started
  oldTime = millis();

  // Take some initial readings to be sure the BME280 is being seen
  takeBME280Reading();
  printBME280Data(&Serial);
  printBME280CalculatedData(&Serial); 
  displayLCDBME280Data();

  // Do the initial write to ThingSpeak so it is easier to debug.
  // This is continually fail if we have not loaded the correct 
  // credentials during setting up.
  getThingsSpeakCreds();
  writeThingSpeak();
}
/* ==== END Setup ==== */

//==========================================================
//===== LOOP ===============================================
//==========================================================
void loop() {
  // LED OFF
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Measure the temperature etc every 5 minutes and send to ThingSpeak
  newTime = millis();

  if((newTime - oldTime) > THINGSPEAKDELAY){
    oldTime = newTime;
    clockTime = newTime;
    
    Serial.print(getNTPTime(0));
    
    // BME code, and we'll have a nap at the end
    takeBME280Reading();
    printBME280Data(&Serial);
    printBME280CalculatedData(&Serial);
    displayLCDBME280Data();
    // Write it to the CLoud
    writeThingSpeak();

  }
  else if((newTime - clockTime) > 1000*60){
    // Update the clock
    clockTime = newTime;
    displayTimeOfDay(getNTPTime(1));
  }

  // Check to see if the WiFi reset button is pressed (LOW)
  // I'm sure this can be done neater.
  //if (!digitalRead(WIFIRESETBUTTON)){
  // OK the reset button is set
  //  Serial.println("WIFIRESETBUTTON pressed");
  //  tft.setCursor(0, 19);
  // tft.println("WiFi Reset");
  //  tft.println("See 192.168.4.1");
  //  configureWIFI(true);
  //}

  // Last thing to do is check for someone touching me
  if (ts.touched()){
    // Let the user know they touched the screen THEN decide if they have held it
    Serial.println("TFT: Touched");
    tft.fillScreen(ILI9341_RED);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(&FreeSans24pt7b);
    tft.setCursor(20,150);
    tft.setTextSize(2);
    tft.println("Ouchie !");
    //TS_Point p = ts.getPoint();
    // Wait 2 secs to see if we are doing a reset of the WiFi
    delay(2000);
    if (!ts.touched() ){
      // OK fun over - rewrite the display.
      Serial.println("TFT: Short Touch");
      displayLCDBME280Data();
    } else{
      // We assume that we have touched the screen long enough
      Serial.println("TFT: Long Touch");
      tft.fillScreen(ILI9341_RED);
      tft.setTextColor(ILI9341_WHITE);
      tft.setFont(&FreeSans24pt7b);
      tft.setCursor(20,150);
      tft.setTextSize(2);
      tft.println("Reset !");
      delay(1000);
      //configureWIFI(true);
      // OK fun over - rewrite the display.
      displayLCDBME280Data();      
    }

  }
  


}
/* ==== End Loop ==== */


//==========================================================
//===== FUNCTIONS ==========================================
//==========================================================
// ThingSpeak uploader routine
// PRE:  Uses globals of temperature, humidity.
// POST: sent current_temp; current_humidity to ThingSpeak "myChannelID"
//==========================================================
void writeThingSpeak(){
  char *eptr;  // Needed only fro strtoul function; not used in anger.
 
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  
  // Set the fields to be set in ThingSpeak
  ThingSpeak.setField(1,temperature);
  ThingSpeak.setField(2,humidity);

  // Write the fields to ThingSpeak
  ThingSpeak.writeFields(strtoul(myChannelID, &eptr, 10), myWriteAPIKey);
  //  Possible response codes:
  //  200: OK / Success
  //  404: Incorrect API key (or invalid ThingSpeak server address)
  //  -101: Value is out of range or string is too long (> 255 characters)
  //  -201: Invalid field number specified
  //  -210: setField() was not called before writeFields()
  //  -301: Failed to connect to ThingSpeak
  //  -302: Unexpected failure during write to ThingSpeak
  //  -303: Unable to parse response
  //  -304: Timeout waiting for server to respond
  //  -401: Point was not inserted (most probable cause is the rate limit of once every 15 seconds)  
  //String message = ThingSpeak.readStringField(myChannelID, 1);
  int resultCode = ThingSpeak.getLastReadStatus();
  if(resultCode == 200  || resultCode == 400)
  //if(resultCode == 200)
  {
    //Serial.print("Latest message is: "); 
    //Serial.println(message);
    Serial.print("Sent to ThingSpeak:: ");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("  Humidity: ");
    Serial.println(humidity);  
  }
  // This should pick up an API key error, but is not, so removing for the moment.
  //else if(resultCode == 400)
  //{
  //  Serial.print("Incorrect API key (or invalid ThingSpeak server address)");
  //  tft.println("Incorrect API key (or invalid ThingSpeak server address)");
  //}
  else
  {
    Serial.print("Error reading message.  Status was: "); 
    Serial.println(resultCode);
  }
   
}

//==========================================================
// WiFiManager Main Function
// PRE: Generalised function to reset the WiFi if needed
//      ClearWIFI = true will force clearing of the WIFI settings
//
//      Fetches ssid and password that has been previously collected and tries to connect
//      if it does not connect it starts an access point with the specified name
//      here  "AutoConnectAP"
//      and goes into a blocking loop awaiting configuration.
//==========================================================
void configureWIFI(boolean ClearWIFI){
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);  

  if (ClearWIFI){
    // reset settings
    Serial.println("Clearing WIFI settings");
    tft.println("Clearing WIFI settings");
    wifiManager.resetSettings();
    ESP.reset();
  }


  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    tft.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  //if you get here you have connected to the WiFi
  Serial.println("Connected...");
  ticker.detach();

}

//==========================================================
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

//==========================================================
// Simple LED Toggle function used in WiFiManager
void tick(){
  //toggle state
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}




//==========================================================
void takeBME280Reading(void){
  // TNS - Notionally we are in "forced mode" which means we need to trigger the read then wait 8ms
  //bme.setMode(0x01);
  delay(10);

  // Save previous readings for comparison
  opressure = pressure;
  otemperature = temperature;
  ohumidity = humidity; 
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
  bme.read(pressure, temperature, humidity, tempUnit, presUnit);   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0) 
}



//==========================================================
void printBME280Data(Stream* client){
  client->print("Temp: ");
  client->print(temperature);
  client->print("°"+ String(metric ? 'C' :'F'));
  client->print("\t\tHumidity: ");
  client->print(humidity);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(pressure);
  client->print(" hPa");
}


/* =============================================== */
void printBME280CalculatedData(Stream* client){
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  /// To get correct local altitude/height (QNE) the reference Pressure
  ///    should be taken from meteorologic messages (QNH or QFF)
  //float altitude = EnvironmentCalculations::Altitude(pressure, envAltUnit, referencePressure, outdoorTemp, envTempUnit);

  //float dewPoint = EnvironmentCalculations::DewPoint(temperature, humidity, envTempUnit);

  /// To get correct seaLevel pressure (QNH, QFF)
  ///    the altitude value should be independent on measured pressure.
  /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
  //  float seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temperature, pressure, envAltUnit, envTempUnit);

  float absHum = EnvironmentCalculations::AbsoluteHumidity(temperature, humidity, envTempUnit);

  client->print("\t\tHeat Index: ");
  float heatIndex = EnvironmentCalculations::HeatIndex(temperature, humidity, envTempUnit);
  client->print(heatIndex);
  client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));

  client->print("\t\tAbsolute Humidity: ");
  client->println(absHum);
}

//==========================================================
void displayLCDBME280Data(void){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 28);
  // Cyan is the base colour; used for reducing numbers also
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setRotation(3);
  if(temperature > otemperature) {tft.setTextColor(ILI9341_MAGENTA);}else{tft.setTextColor(ILI9341_CYAN);}
  tft.setFont(&FreeSans18pt7b);
  tft.print("T:");
  tft.print(temperature,1);
  tft.setFont(&FreeSans12pt7b);
  tft.println("*C");
  if(humidity > ohumidity) {tft.setTextColor(ILI9341_MAGENTA);}else{tft.setTextColor(ILI9341_CYAN);}
  tft.setFont(&FreeSans18pt7b);
  tft.print("H:");
  tft.print(humidity,1);
  tft.setFont(&FreeSans12pt7b);
  tft.println("% RH");  
  if(pressure > opressure) {tft.setTextColor(ILI9341_MAGENTA);}else{tft.setTextColor(ILI9341_CYAN);}
  tft.setFont(&FreeSans18pt7b);
  tft.print("P:");
  tft.print(pressure,1);
  tft.setFont(&FreeSans12pt7b);
  tft.println(" hPa");
  displayTimeOfDay(getNTPTime(1));
}

// Simple TFT function to display a ToD string value
void displayTimeOfDay(const char* currenttime){
  tft.fillRect(20,150,200,75,ILI9341_BLACK);
  tft.setCursor(20,200);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.println(currenttime);
}

/* =============================================== */
// getNTPTime reads the time from a NTP source (internet required)
// Input format:
//  fmt = 0 -> return as hh:mm:ss
//  fmt = 1 -> return as hh:mm

const char* getNTPTime(int fmt)
{
  // Allocate some memory for the return string
  static char timeofday[20];  
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(500);
  
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
    sprintf(timeofday,"No Time");
  }
  else {
    //Serial.print("packet received, length=");
    //Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = " );
    //Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears + TZOFFSET;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The local time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    // TNS: Using casts relatively safely given the calculation being asked for and the 
    // expected return values.
    //hours = static_cast<int>((epoch  % 86400L) / 3600);
    //minutes = static_cast<int>((epoch  % 3600) / 60);
    //seconds = static_cast<int>(epoch % 60);
    if (fmt == 0){
      sprintf(timeofday,"%02lu:%02lu:%02lu",(epoch  % 86400L) / 3600, (epoch  % 3600) / 60, epoch % 60);
    }
    else{
      sprintf(timeofday,"%02lu:%02lu",(epoch  % 86400L) / 3600, (epoch  % 3600) / 60);      
    }
  }
  return timeofday;
}

/* =============================================== */
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}




// read diagnostics (optional but can help debug problems)
// Uses global Adafruit_ILI9341 tft
void displayDiagnostics(void)
{
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x");
  Serial.println(x, HEX);

  Serial.println(F("Done!"));
}

// Clear the display
// Uses global Adafruit_ILI9341 tft
void displayMakeBlack(void)
{
  tft.fillScreen(ILI9341_BLACK);
}  

 
void getThingsSpeakCreds(void)
{
  File configFile;
  if (SPIFFS.begin()) 
  {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) 
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      configFile = SPIFFS.open("/config.json", "r");
      if (configFile) 
      {
        Serial.println("Opened config file.");
        // size_t size = configFile.size();

        DynamicJsonDocument jsonBuffer(500);
        // Parse directly from file
        DeserializationError err= deserializeJson(jsonBuffer, configFile);
        if(err) 
        {
          Serial.print(F("DeserializeJson() failed with code "));
          Serial.println(err.c_str());
        }else {
          // Loop through all the elements of the array
          for(JsonObject repo: jsonBuffer.as<JsonArray>()) 
          {
            // Print the information for debugging.
            Serial.println(repo["TS_ChannelID"].as<char*>());
            Serial.println(repo["TS_APIKey"].as<char*>());
            // Now actually set it.
            strcpy(myChannelID, repo["TS_ChannelID"].as<char*>());
            strcpy(myWriteAPIKey, repo["TS_APIKey"].as<char*>());
          }
        }
        configFile.close();
      }
    } else{
      Serial.println("Failed to open config file.");
      configFile = SPIFFS.open("/config.json","w");
      if(!configFile){
        Serial.println("Failed to open config file.");
      } else {
        Serial.println("Work to do here ...");
        SPIFFS.remove("/config.json");
      }
    }
  } else {
    Serial.println("Failed to mount FS.");
  }
}

/* ==== END Functions ==== */
