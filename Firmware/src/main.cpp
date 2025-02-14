#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Preferences.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <NTPClient.h>
#include <time.h>
#include <movingAvg.h>

// User Memory - WiFi SSID, PSK, Clock Configuration bits.
Preferences preferences;

// AccessPoint Credentials. Default UserMem WiFi Value.
const char* APID = "NiceClock";
const char* APSK = "MinesBigger";
const char* GARBAGE_STRING = "C!pbujKY2#4HXbcm5dY!WJX#ns29ff#vEDWmbZ9^d!QfBW@o%Trfj&sPENuVe&sx";

// Global Variables
bool softAPActive = false;
bool showIP = false;

// Server Stuff
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);  // Initially uses default server; can be reinitialized.
AsyncWebServer server(80);

// PWM properties for brightness control
#define freq 2000
#define ledChannel 0
#define ledPWMResolution 8

// Pins used for segment displays (unchanged from your base code)
const int hours_pin[8] = {10, 8, 3, 9, 14, 11, 12, 13};
const int minutes_pin[8] = {7, 4, 5, 6, 18, 15, 16, 17};

// Additional Control Pins
const int ledPin = 1;       // Power pin for segment displays. Common Anode.
const int decimalPin = 35;  // Decimal point which separates hours and minutes.
// Other decimal points on pins 36, 37, 38.

// Global variable for smooth dimming PWM output.
uint8_t currentPWM = 50;

// Smooth transition helper: move current value one step toward target.
uint8_t smoothTransition(uint8_t current, uint8_t target) {
  if (current < target) return current + 1;
  if (current > target) return current - 1;
  return current;
}

/// @brief Prints the time to the 7-segment displays.
/// Splits hours and minutes into tens and ones and writes the bits.
void printTime(int hours, int minutes) {
  bool timeFormat = preferences.getBool("timeFormat", 1);
  if (!timeFormat) {
    hours = (hours % 12) ?: 12;
  }
  int hoursT = hours / 10;
  int hoursO = hours % 10;
  int minutesT = minutes / 10;
  int minutesO = minutes % 10;
  
  // Hours tens
  digitalWrite(hours_pin[4], (hoursT >> 0) & 1);
  digitalWrite(hours_pin[5], (hoursT >> 1) & 1);
  digitalWrite(hours_pin[6], (hoursT >> 2) & 1);
  digitalWrite(hours_pin[7], (hoursT >> 3) & 1);
  // Hours ones
  digitalWrite(hours_pin[0], (hoursO >> 0) & 1);
  digitalWrite(hours_pin[1], (hoursO >> 1) & 1);
  digitalWrite(hours_pin[2], (hoursO >> 2) & 1);
  digitalWrite(hours_pin[3], (hoursO >> 3) & 1);
  // Minutes tens
  digitalWrite(minutes_pin[4], (minutesT >> 0) & 1);
  digitalWrite(minutes_pin[5], (minutesT >> 1) & 1);
  digitalWrite(minutes_pin[6], (minutesT >> 2) & 1);
  digitalWrite(minutes_pin[7], (minutesT >> 3) & 1);
  // Minutes ones
  digitalWrite(minutes_pin[0], (minutesO >> 0) & 1);
  digitalWrite(minutes_pin[1], (minutesO >> 1) & 1);
  digitalWrite(minutes_pin[2], (minutesO >> 2) & 1);
  digitalWrite(minutes_pin[3], (minutesO >> 3) & 1);
}

// Interrupt handler: sets flag to display the IP.
static void IRAM_ATTR showIPFlag(){
  showIP = true;
}

/// @brief Displays the IP address by showing each octet in turn.
void displayIP(IPAddress localIP) {
  pinMode(38, OUTPUT);
  digitalWrite(decimalPin, 1);
  digitalWrite(38, 0);
  
  int octet0 = localIP[0];
  int octet1 = localIP[1];
  int octet2 = localIP[2];
  int octet3 = localIP[3];
  
  printTime(octet0 / 100, octet0 % 100);
  delay(1500);
  printTime(octet1 / 100, octet1 % 100);
  delay(1500);
  printTime(octet2 / 100, octet2 % 100);
  delay(1500);
  printTime(octet3 / 100, octet3 % 100);
  delay(1500);
  
  digitalWrite(decimalPin, 0);
  pinMode(38, INPUT);
}

/// @brief Initializes the BH1730 Ambient Light Sensor.
void initLightSensor() {
  Wire.beginTransmission(0x29);
  Wire.write(0x80);
  Wire.write(0x3);
  Wire.endTransmission();
}

/// @brief Reads ambient light sensor data and maps it between stored min and max brightness.
uint8_t readAmbientLightData(){
  Wire.beginTransmission(0x29);
  Wire.write(0b10010100);
  Wire.endTransmission();
  Wire.requestFrom(0x29, 2);
  byte highByte = Wire.read();
  byte lowByte = Wire.read();
  int lightLevel = (highByte << 8) | lowByte;
  return map(lightLevel, 0, 65535, preferences.getInt("minBrightness", 10), preferences.getInt("maxBrightness", 255));
}

// Moving average to smooth ambient light changes.
movingAvg ambientLight(10);

void setup() {
  Serial.begin(9600);
  Serial.println("Entered setup");
  preferences.begin("usermem");
  SPIFFS.begin();
  
  setenv("TZ", preferences.getString("timezone", "UTC").c_str(), 1);
  tzset();
  
  ledcSetup(ledChannel, freq, ledPWMResolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 50);
  
  Wire.begin(34, 33);
  initLightSensor();
  ambientLight.begin();
  for (int i = 0; i < 10; i++) {
    ambientLight.reading(readAmbientLightData());
  }
  
  pinMode(decimalPin, OUTPUT);
  digitalWrite(decimalPin, 0);
  for (int i = 0; i < 8; i++) {
    pinMode(hours_pin[i], OUTPUT);
    pinMode(minutes_pin[i], OUTPUT);
  }
  
  // ---------- WiFi Setup ----------
  Serial.println("Starting Access Point");
  WiFi.softAPsetHostname("niceclock");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(APID, APSK);
  softAPActive = true;
  
  if (preferences.getString("WiFiSSID", GARBAGE_STRING) == GARBAGE_STRING ||
      preferences.getString("WiFiPSK", GARBAGE_STRING) == GARBAGE_STRING) {
    Serial.println("WiFi not configured. Skipping network connection.");
  }
  else {
    Serial.println("WiFi configured. Attempting connection.");
    WiFi.setHostname("niceclock");
    WiFi.begin(preferences.getString("WiFiSSID", GARBAGE_STRING).c_str(),
               preferences.getString("WiFiPSK", GARBAGE_STRING).c_str());
    while(!WiFi.isConnected() && millis() < 5000) {
      Serial.print(". ");
      delay(100);
    }
    Serial.println();
    if (WiFi.isConnected()){
      Serial.println("Connection Successful. Tearing down AP.");
      WiFi.softAPdisconnect();
      WiFi.mode(WIFI_STA);
      softAPActive = false;
      displayIP(WiFi.localIP());
    }
    else {
      Serial.println("Connection Failed. Please use the web portal to enter valid information.");
    }
  }
  
  digitalWrite(decimalPin, 0);
  
  timeClient.begin();
  timeClient.update();
  
  // ---------- Web Configuration Endpoints ----------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!SPIFFS.exists("/index.html")){
      request->send(404, "text/plain", "File not found");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.on("/milligram.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Loading CSS");
    request->send(SPIFFS, "/milligram.min.css", "text/css");
  });
  
  server.on("/zones.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Loading zones");
    request->send(SPIFFS, "/zones.csv", "text/csv");
  });
  
  server.on("/updateWiFi", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("ssid", true) && request->hasParam("psk", true)){
      String ssid = request->getParam("ssid", true)->value();
      String psk = request->getParam("psk", true)->value();
      if(ssid != ""){
        Serial.println("Updating WiFi Credentials to SSID: " + ssid);
        preferences.putString("WiFiSSID", ssid);
        preferences.putString("WiFiPSK", psk);
        WiFi.setHostname("niceclock");
        WiFi.begin(preferences.getString("WiFiSSID", GARBAGE_STRING).c_str(),
                   preferences.getString("WiFiPSK", GARBAGE_STRING).c_str());
      }
    }
    request->send(200, "text/plain", "WiFi credentials updated");
  });
  
  server.on("/updateTimeFormat", HTTP_POST, [](AsyncWebServerRequest *request) {
    if(request->hasArg("isChecked")){
      String isChecked = request->arg("isChecked");
      if(isChecked == "true"){
        preferences.putBool("timeFormat", 0);
      } else {
        preferences.putBool("timeFormat", 1);
      }
    }
    request->send(200, "text/plain", "Time format updated");
  });
  
  server.on("/getTimeFormat", HTTP_GET, [](AsyncWebServerRequest *request){
    String checkboxStateStr = preferences.getBool("timeFormat", 1) ? "false" : "true";
    request->send(200, "text/plain", checkboxStateStr);
  });
  
  server.on("/getWiFiSSID", HTTP_GET, [](AsyncWebServerRequest *request){
    String wiFiSSID = preferences.getString("WiFiSSID", "");
    request->send(200, "text/plain", wiFiSSID);
  });
  
  server.on("/getMinBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    String minB = String(preferences.getInt("minBrightness", 10));
    request->send(200, "text/plain", minB);
  });
  
  server.on("/getMaxBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    String maxB = String(preferences.getInt("maxBrightness", 255));
    request->send(200, "text/plain", maxB);
  });
  
  server.on("/updateBrightness", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("minBrightnessSlider", true) && request->hasParam("maxBrightnessSlider", true)){
      int minVal = request->getParam("minBrightnessSlider", true)->value().toInt();
      int maxVal = request->getParam("maxBrightnessSlider", true)->value().toInt();
      preferences.putInt("minBrightness", minVal);
      preferences.putInt("maxBrightness", maxVal);
    }
    request->send(200, "text/plain", "Brightness limits updated");
  });
  
  server.on("/setTZ", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasArg("timezone")){
      String timezone = request->arg("timezone");
      preferences.putString("timezone", timezone);
      configTzTime(timezone.c_str(), "pool.ntp.org");
      timeClient.update();
    }
    request->send(200, "text/plain", "Timezone updated");
  });
  
  // New endpoint to update the NTP server.
  server.on("/updateNTP", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasArg("ntpServer")){
      String newNtp = request->arg("ntpServer");
      // Reinitialize the NTP client with the new server.
      timeClient = NTPClient(ntpUDP, newNtp.c_str());
      timeClient.begin();
      timeClient.update();
      request->send(200, "text/plain", "NTP server updated to " + newNtp);
    } else {
      request->send(400, "text/plain", "Missing ntpServer parameter");
    }
  });
  
  server.onNotFound([](AsyncWebServerRequest *request){
    String path = request->url();
    if (path.endsWith("/")) path += "index.html";
    String contentType = "text/plain";
    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".csv")) contentType = "text/csv";
    if(SPIFFS.exists(path)){
      request->send(SPIFFS, path, contentType);
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });
  
  server.begin();
  
  // Use an interrupt on digital pin 0 to flag display of IP.
  attachInterrupt(digitalPinToInterrupt(0), showIPFlag, FALLING);
}

void loop() {
  if(softAPActive && WiFi.isConnected()){
    Serial.println("Internet Connected. Tearing down AP.");
    WiFi.softAPdisconnect();
    WiFi.mode(WIFI_STA);
    softAPActive = false;
    timeClient.update();
  }
  if(!softAPActive && !WiFi.isConnected()) {
    Serial.println("Lost Internet. Restarting AP.");
    WiFi.softAPsetHostname("niceclock");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(APID, APSK);
    softAPActive = true;
  }
  if((timeClient.getHours() % 6) == 0 && timeClient.getMinutes() == 0 && timeClient.getSeconds() == 0){
    timeClient.update();
    delay(1000);
  }
  
  time_t now = timeClient.getEpochTime();
  struct tm *timeinfo = localtime(&now);
  printTime(timeinfo->tm_hour, timeinfo->tm_min);
  
  if ((millis() % 25) == 0) {
    uint8_t desiredPWM = ambientLight.reading(readAmbientLightData());
    currentPWM = smoothTransition(currentPWM, desiredPWM);
    ledcWrite(ledChannel, currentPWM);
  }
  
  if (showIP){
    displayIP(WiFi.localIP());
    showIP = false;
  }
}