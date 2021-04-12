/*
 * Nov 25, 2020, Binh Nguyen
 * Lithium battery testing with INA3221 and ESP32
 * with data publish to MQTT server
 *
 */

#include <Wire.h>
#include "SDL_Arduino_INA3221.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "MAX44009.h"
#include "Adafruit_MCP9808.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_NeoPixel.h>

OneWire oneWire(32);
DallasTemperature ds18(&oneWire);
Adafruit_NeoPixel ws(1, 33, NEO_GRB + NEO_KHZ800);

MAX44009 maxlux(0x4A);
SDL_Arduino_INA3221 ina3221;
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define MOSFET 25
#define v_bat_ok 12.0

bool mosfet_on = false;
static float v_delta = 0.2;

WiFiClient espClient;
PubSubClient mqtt(espClient);
bool DEBUG = true;
Adafruit_MCP9808 mcp = Adafruit_MCP9808();

/*_____________________WIFI and MQTT_______________________*/
#define wifi_ssid "wifi_name"
#define wifi_password "wifi_pass"
#define mqtt_server "192.168.1.2"
#define mqtt_user "mqtt_user" 
#define mqtt_password "mqtt_pass"
#define mqtt_port 1883
#define publish_topic "sensors/solar"
#define OTApassword "OTAPass"
#define SENSORNAME "solar_20wp"
char HOSTNAME[30];

uint16_t INVL = 60; //60 seconds, only first run
int32_t lastPush=-INVL;
float bat_vol = 0;

/*_____________________SETUP_______________________*/
void setup(){
  Serial.begin(115200);
  Serial.printf("\n\nStarting\n");
  Wire.begin();
  pinMode(MOSFET, OUTPUT);
  digitalWrite(MOSFET, LOW);
  
  mcp.begin(0x18);
  mcp.setResolution(3);

  ina3221.begin();
  ds18.begin();
  ws.begin();
  ws.setBrightness(100); //max 255
  maxlux.Begin(0, 18800);
    
  setup_wifi();
  mqtt.setServer(mqtt_server, mqtt_port);
  setup_OTA();
  led_bat(12.0);
  delay(2000);
}

/*_____________________MAIN_______________________*/
void loop(){
  // main program here
  ArduinoOTA.handle();
  uint32_t now_ = millis()/1000;
  if ((now_-lastPush)>=INVL){
    DynamicJsonDocument doc(1024);
    doc["sensor"] = SENSORNAME;
    doc["uptime"] = now_;
    read_INA3221(doc);
    read_MCP9808(doc);
    read_DS18B20(doc);
    read_MAX44009(doc);
    ctrl_mosfet(bat_vol); // control mosfet
    doc["expt"] = mosfet_on;
    push_Data(doc);
    lastPush = now_;
    
    JsonObject obj = doc.to<JsonObject>(); // empty doc object
    led_bat(bat_vol); // show the voltage by color
    
    if (now_ > 86400L){
      ESP.restart();
    }
  }
  delay(1000);
}

/*_____________________READING SENSORS_______________________*/
void read_DS18B20(DynamicJsonDocument &doc){
  ds18.requestTemperatures(); 
  float tempC = ds18.getTempCByIndex(0);
  doc["DS18"] = tempC;
  Serial.printf("\nDS18B20 Temperature, *C, %.1f\n", tempC);
}

void read_MCP9808(DynamicJsonDocument &doc){
  mcp.wake();
  float c = mcp.readTempC();
  doc["MCP9808"] = c;
  delay(100);
  mcp.shutdown_wake(1);
  Serial.printf("MCP9808 *C: %.1f", c); 
}

void read_MAX44009(DynamicJsonDocument &doc) {
    int8_t i = 0;
    float lux = maxlux.GetLux();
    while (lux >=188000L){
      delay(100);
      lux = maxlux.GetLux();
      i ++;
      if (i>3){
        lux=0;
        break;
      }
    }
    Serial.printf("Lux: %.1f\n", lux);
//    Serial.printf("W: %.1f\n", maxlux.GetWpm());
    doc["max44099"]["lux"] = lux;
    if (lux==0){
      doc["max44099"]["W"] = 0;
    } else {
      doc["max44099"]["W"] =  maxlux.GetWpm();  
    }
    if (lux <100){
      INVL = 300; // 5 minutes
    } else {
      INVL = 30; // 30 seconds;
    }
  }

void read_INA3221(DynamicJsonDocument &doc){
    
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;

  busvoltage1 = ina3221.getBusVoltage_V(CHANNEL_1);
  shuntvoltage1 = ina3221.getShuntVoltage_mV(CHANNEL_1);
  current_mA1 = ina3221.getCurrent_mA(CHANNEL_1); 
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
  doc["CH1"]["busV"] = busvoltage1;
  doc["CH1"]["shmV"] =  shuntvoltage1;
  doc["CH1"]["loadV"] =  loadvoltage1;
  doc["CH1"]["mA"] =  current_mA1;
  
  float shuntvoltage2 = 0;
  float busvoltage2 = 0;
  float current_mA2 = 0;
  float loadvoltage2 = 0;

  busvoltage2 = ina3221.getBusVoltage_V(CHANNEL_2);
  bat_vol = busvoltage2;
  shuntvoltage2 = ina3221.getShuntVoltage_mV(CHANNEL_2);
  current_mA2 = ina3221.getCurrent_mA(CHANNEL_2);
  loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

  doc["CH2"]["busV"] = busvoltage2;
  doc["CH2"]["shmV"] =  shuntvoltage2;
  doc["CH2"]["loadV"] =  loadvoltage2;
  doc["CH2"]["mA"] =  current_mA2;
  
  float shuntvoltage3 = 0;
  float busvoltage3 = 0;
  float current_mA3 = 0;
  float loadvoltage3 = 0;

  busvoltage3 = ina3221.getBusVoltage_V(CHANNEL_3);
  shuntvoltage3 = ina3221.getShuntVoltage_mV(CHANNEL_3);
  current_mA3 = ina3221.getCurrent_mA(CHANNEL_3);
  loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000);
  
  doc["CH3"]["busV"] = busvoltage3;
  doc["CH3"]["shmV"] =  shuntvoltage3;
  doc["CH3"]["loadV"] =  loadvoltage3;
  doc["CH3"]["mA"] =  current_mA3;
  
  if (DEBUG){
    Serial.println("------------------------------");
    Serial.printf("CHANNEL_1 Bus Voltage:   %.1f V\n",busvoltage1 );
    Serial.printf("CHANNEL_1 Shunt Voltage: %.1f mV\n", shuntvoltage1);
    Serial.printf("CHANNEL_1 Load Voltage:  %.1f V\n", loadvoltage1);
    Serial.printf("CHANNEL_1 Current 1:     %.1f mA\n", current_mA1);
    Serial.println("");

    Serial.printf("CHANNEL_2 Bus Voltage 2:   %.1f V\n",busvoltage2);
    Serial.printf("CHANNEL_2 Shunt Voltage 2: %.1f mV\n", shuntvoltage2);
    Serial.printf("CHANNEL_2 Load Voltage 2:  %.1f V\n", loadvoltage2);
    Serial.printf("CHANNEL_2 Current 2:       %.1f mA\n", current_mA2);
    Serial.println("");    
    
    Serial.printf("CHANNEL_3 Bus Voltage 3:   %.1f V\n",busvoltage3);
    Serial.printf("CHANNEL_3 Shunt Voltage 3: %.1f mV\n", shuntvoltage2);
    Serial.printf("CHANNEL_3 Load Voltage 3:  %.1f V\n", loadvoltage2);
    Serial.printf("CHANNEL_3 Current 3:       %.1f mA\n", current_mA2);
    Serial.println("");
  }
}

/*_____________________ NEOPIXEL______________________*/
void led_bat(float vol){
  Serial.printf("Batery Voltage: %f\n", vol);
  uint8_t r,g,b;
  if (vol>=11.5){
    g=255;
    b=r=0;
  } else if (vol>10.0){   
    r=255,
    g=140;
    b=0;
  } else {
    r=255;
    g=b=0;
  } 
  
  for (int i=0; i<=5; i++){
//    Serial.printf("RGB batery: r: %i\tg: %i\tr: %i\n", r, g, b);
    ws.clear();
    ws.setPixelColor(0, ws.Color(0, 0, 0));
    ws.show();
    delay(500);
    ws.clear();
    ws.setPixelColor(0, ws.Color(r, g, b));
    ws.show();
    delay(500); 
    ws.setPixelColor(0, ws.Color(0, 0, 0));
    ws.show();
    ws.clear();
  }
}

/*__________   control MOSFET  ___________*/
void ctrl_mosfet(float v_bat){
//  bat_vol, P MOSFET is closed circuit if VGS is -, 
// BC337 is a N transitor, closed if VBE is +
  Serial.printf("MOSFET status %d\n", mosfet_on);
  if ((v_bat > (v_bat_ok - v_delta)) & !mosfet_on){ //if battery voltage is higher and mosfet is off
    Serial.println("TURNON");
    digitalWrite(MOSFET, HIGH); // pull high 
    mosfet_on = true;   
  }

  if ((v_bat <= (v_bat_ok + v_delta)) & mosfet_on){
    digitalWrite(MOSFET, LOW);
    Serial.println("TURN OFF");
    mosfet_on = false;
  }
  return;
}

/*__________   setup WIFI  ___________*/
void create_host_name(){
  char MAC[6];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(MAC, "%02x%02x%02x", macAddr[3], macAddr[4], macAddr[5]);
//  Serial.println(MAC);
  String NAME_TMP;
  NAME_TMP = String(SENSORNAME) + "__" + String(MAC);
  NAME_TMP.trim();
  byte str_len = NAME_TMP.length() + 1; 
  NAME_TMP.toCharArray(HOSTNAME, str_len);
//  Serial.printf("Hostname: %s\n", HOSTNAME);
}

/*______________    SETUP WIFI     _______________*/
void setup_wifi() {
  delay(10);
  Serial.printf("Connecting to %s", wifi_ssid);
  WiFi.mode(WIFI_STA);
  
  create_host_name();
  bool tmp = WiFi.setHostname(HOSTNAME);
  Serial.printf("\t>>Hostname: %s\n", WiFi.getHostname());
  WiFi.begin(wifi_ssid, wifi_password);
  delay(3000); 
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    i++;
    Serial.printf(" %i ", i);
    if (i == 3){
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifi_ssid, wifi_password);
      delay(5000);
    }
    if (i >=31){
      ESP.restart();
      Serial.println("Resetting ESP");
    }
  }
  Serial.printf("\nWiFi connected: \t");
  Serial.print(WiFi.localIP());
  Serial.print("\twith MAC:\t");
  Serial.println(WiFi.macAddress());
}
/*__________________ PUSH DATA  ___________________*/
void push_Data(DynamicJsonDocument &doc) {
    
    size_t len = measureJson(doc)+ 1;
    char payload[len];
    serializeJson(doc, payload, sizeof(payload));
    
   if (!mqtt.connected()) {
    reconnect();
    delay(1000);
  }
  if (mqtt.publish(publish_topic, payload, false)){
    Serial.println("Success: " + String(payload));
  } else {
    Serial.println("Failed to push: " + String(payload));
  }
}

/*_____________________START RECONNECT_______________________*/
void reconnect() {
  while (WiFi.status() != WL_CONNECTED){
  setup_wifi();
  }
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect(SENSORNAME, mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
/*_____________________START OT_______________________*/
void setup_OTA(){
  ArduinoOTA.setPort(8266); 
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword((const char *)OTApassword);
  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
