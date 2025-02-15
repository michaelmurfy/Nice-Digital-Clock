#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <NTPClient.h>
#include <time.h>
#include <movingAvg.h>

// Prototypes:
uint8_t readAmbientLightData();
static void IRAM_ATTR showIPFlag();
void updateBrightnessSmooth();

// User Memory for WiFi/clock configuration.
Preferences preferences;

// Access Point Credentials.
const char* APID = "NiceClock";
const char* APSK = "MinesBigger";
const char* GARBAGE_STRING = "C!pbujKY2#4HXbcm5dY!WJX#ns29ff#vEDWmbZ9^d!QfBW@o%Trfj&sPENuVe&sx";

// Global variables.
bool softAPActive = false;
bool showIP = false;

// Server Stuff.
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);  // Uses default server ("time.cloudflare.com") initially; can be updated.
AsyncWebServer server(80);

// PWM properties for brightness control.
#define freq 2000
#define ledChannel 0
#define ledPWMResolution 8

// Pins for 7-segment displays.
const int hours_pin[8]   = {10, 8, 3, 9, 14, 11, 12, 13};
const int minutes_pin[8] = {7, 4, 5, 6, 18, 15, 16, 17};

// Additional control pins.
const int ledPin = 1;       // Power pin for segment displays (common anode)
const int decimalPin = 35;  // Decimal point separating hours and minutes
// Other decimal points on pins 36, 37, 38.

// ----- Brightness Smoothing Variables -----
// We accumulate ambient light readings over 500ms, then interpolate from the previous PWM value to the new target over that 500ms period.
uint8_t currentPWM = 50;
unsigned long lastAvgTime = 0;
unsigned long transitionStartTime = 0;
uint8_t prevPWM = 50;
uint8_t targetPWM = 50;
uint32_t lightSum = 0;
uint16_t sampleCount = 0;

// Create moving average object for ambient light (if desired).
movingAvg ambientLight(10);

// ----- Function: readAmbientLightData -----
// Reads the BH1730 sensor data and maps it between the stored min and max brightness values.
uint8_t readAmbientLightData() {
  Wire.beginTransmission(0x29);
  Wire.write(0b10010100);
  Wire.endTransmission();
  Wire.requestFrom(0x29, 2);
  byte highByte = Wire.read();
  byte lowByte = Wire.read();
  int lightLevel = (highByte << 8) | lowByte;
  return map(lightLevel, 0, 65535, preferences.getInt("minBrightness", 10), preferences.getInt("maxBrightness", 255));
}

// ----- Function: showIPFlag -----
// Interrupt handler: sets flag so that the IP will be displayed.
static void IRAM_ATTR showIPFlag() {
  showIP = true;
}

// ----- Function: updateBrightnessSmooth -----
// Accumulates ambient light readings over 500ms, computes an average,
// then linearly interpolates the PWM output from the previous value to the new target over 500ms.
void updateBrightnessSmooth() {
  unsigned long now = millis();
  uint8_t reading = ambientLight.reading(readAmbientLightData());
  lightSum += reading;
  sampleCount++;
  
  if (now - lastAvgTime >= 500) {
    uint8_t avgPWM = sampleCount ? (lightSum / sampleCount) : reading;
    lastAvgTime = now;
    lightSum = 0;
    sampleCount = 0;
    prevPWM = currentPWM;
    targetPWM = avgPWM;
    transitionStartTime = now;
  }
  
  uint32_t elapsed = now - transitionStartTime;
  if (elapsed < 500) {
    float fraction = (float)elapsed / 500.0;
    uint8_t newPWM = prevPWM + (int)((targetPWM - prevPWM) * fraction);
    ledcWrite(ledChannel, newPWM);
    currentPWM = newPWM;
  } else {
    ledcWrite(ledChannel, targetPWM);
    currentPWM = targetPWM;
  }
}

// ----- Function: printTime -----
// Splits hours and minutes into tens and ones and writes the bits to the display pins.
void printTime(int hours, int minutes) {
  bool timeFormat = preferences.getBool("timeFormat", 1);
  if (!timeFormat) {
    hours = (hours % 12) ?: 12;
  }
  int hoursT = hours / 10;
  int hoursO = hours % 10;
  int minutesT = minutes / 10;
  int minutesO = minutes % 10;
  
  digitalWrite(hours_pin[4], (hoursT >> 0) & 1);
  digitalWrite(hours_pin[5], (hoursT >> 1) & 1);
  digitalWrite(hours_pin[6], (hoursT >> 2) & 1);
  digitalWrite(hours_pin[7], (hoursT >> 3) & 1);
  
  digitalWrite(hours_pin[0], (hoursO >> 0) & 1);
  digitalWrite(hours_pin[1], (hoursO >> 1) & 1);
  digitalWrite(hours_pin[2], (hoursO >> 2) & 1);
  digitalWrite(hours_pin[3], (hoursO >> 3) & 1);
  
  digitalWrite(minutes_pin[4], (minutesT >> 0) & 1);
  digitalWrite(minutes_pin[5], (minutesT >> 1) & 1);
  digitalWrite(minutes_pin[6], (minutesT >> 2) & 1);
  digitalWrite(minutes_pin[7], (minutesT >> 3) & 1);
  
  digitalWrite(minutes_pin[0], (minutesO >> 0) & 1);
  digitalWrite(minutes_pin[1], (minutesO >> 1) & 1);
  digitalWrite(minutes_pin[2], (minutesO >> 2) & 1);
  digitalWrite(minutes_pin[3], (minutesO >> 3) & 1);
}

// ----- Function: displayIP -----
// Displays the IP address on the display by showing each octet in turn.
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

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");
  preferences.begin("usermem");
  SPIFFS.begin();
  
  // Set the timezone from preferences.
  setenv("TZ", preferences.getString("timezone", "UTC").c_str(), 1);
  tzset();
  
  // Setup PWM for brightness.
  ledcSetup(ledChannel, freq, ledPWMResolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 50);
  
  // Initialize I2C on pins SDA=34, SCL=33.
  Wire.begin(34, 33);
  // Initialize ambient light sensor.
  Wire.beginTransmission(0x29);
  Wire.write(0x80);
  Wire.write(0x3);
  Wire.endTransmission();
  
  ambientLight.begin();
  // Perform several dummy readings.
  for (int i = 0; i < 10; i++) {
    readAmbientLightData();
  }
  
  pinMode(decimalPin, OUTPUT);
  digitalWrite(decimalPin, 0);
  for (int i = 0; i < 8; i++) {
    pinMode(hours_pin[i], OUTPUT);
    pinMode(minutes_pin[i], OUTPUT);
  }
  
  // ----- WiFi Setup -----
  Serial.println("Starting Access Point");
  WiFi.softAPsetHostname("niceclock");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(APID, APSK);
  softAPActive = true;
  
  if (preferences.getString("WiFiSSID", GARBAGE_STRING) == GARBAGE_STRING ||
      preferences.getString("WiFiPSK", GARBAGE_STRING) == GARBAGE_STRING) {
    Serial.println("WiFi not configured. Skipping connection.");
  } else {
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
      Serial.println("Connection successful. Tearing down AP.");
      WiFi.softAPdisconnect();
      WiFi.mode(WIFI_STA);
      softAPActive = false;
      displayIP(WiFi.localIP());
    } else {
      Serial.println("Connection failed. Use web portal to enter credentials.");
    }
  }
  
  digitalWrite(decimalPin, 0);
  
  timeClient.begin();
  timeClient.update();
  
  // ----- Web Configuration Endpoints -----
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!SPIFFS.exists("/index.html")){
      request->send(404, "text/plain", "File not found");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.on("/zones.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/zones.csv", "text/csv");
  });
  
  server.on("/updateWiFi", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("ssid", true) && request->hasParam("psk", true)){
      String ssid = request->getParam("ssid", true)->value();
      String psk = request->getParam("psk", true)->value();
      if(ssid != ""){
        Serial.println("Updating WiFi: " + ssid);
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
      preferences.putBool("timeFormat", (isChecked == "true") ? 0 : 1);
    }
    request->send(200, "text/plain", "Time format updated");
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
      configTzTime(timezone.c_str(), "time.cloudflare.com");
      timeClient.update();
    }
    request->send(200, "text/plain", "Timezone updated");
  });
  
  server.on("/updateNTP", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasArg("ntpServer")){
      String newNtp = request->arg("ntpServer");
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
  if(!softAPActive && !WiFi.isConnected()){
    Serial.println("Lost Internet. Restarting AP.");
    WiFi.softAPsetHostname("niceclock");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(APID, APSK);
    softAPActive = true;
  }
  if((timeClient.getHours() % 6) == 0 &&
     timeClient.getMinutes() == 0 &&
     timeClient.getSeconds() == 0){
    timeClient.update();
    delay(1000);
  }
  
  time_t now = timeClient.getEpochTime();
  struct tm *timeinfo = localtime(&now);
  printTime(timeinfo->tm_hour, timeinfo->tm_min);
  
  updateBrightnessSmooth();
  
  if (showIP){
    displayIP(WiFi.localIP());
    showIP = false;
  }
}