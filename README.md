# BME280_Weather-ThingSpeak-WifiManager-2.4in TFT LCD
A weather station using the BME280, and LCD on I2C and results to ThingSpeak.

The display is based on the LOLIN 2.4in LCD mentioned here:
https://wiki.wemos.cc/products:d1_mini_shields:tft_2.4_shield

This project is using the WEMOS (LOLIN) D1 MINI based ESP8266 WiFi Board.

WiFiManager is used for initial configuration. 
##WiFi Manager: How It Works (per Library Notes)
  Git Repo: https://github.com/tzapu/WiFiManager.git
  When your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
  if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a 
  DNS and WebServer (default ip 192.168.4.1)
  using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
  because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get 
  any domain you try to access redirected to the configuration portal
  choose one of the access points scanned, enter password, click save
  ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.

## Library Dependencies  
This programme should be compiled using the Arduino IDE.
The following libraries and their dependencies should be loaded:
1. Adafruit_GFX 1.10.6
2. Adafruit_ILI9341 1.5.6
3. XPT2046_Touchscreen v1.3.0 by PaulS toffregen 
4. BME280 3.0.0 by Tyler Glenn (finitespace)
5. ArduinoJson 6.17.3 by Benoit Blanchon https://arduinojson.org/?utm_source=meta&utm_medium=library.properties