// include lib
#include <Arduino.h>
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_JSON.h>

// include Font
#include "quark24.h"
#include "quark12.h"
#include "quark9.h"

// config network
const char* ssid = "GM";
const char* password = "gm969/13";

// config mqtt
const char* mqtt_server = "154.215.14.239";
const char* mqtt_user = "KMITL-02";
const char* mqtt_password = "KMITL-02";
const char* mqtt_port = "1883";
uint32_t chipId = 0;

// config topic
const char* topic = "KMITL-02/";

// define libs
WiFiClient client;
PubSubClient mqtt(client);
Adafruit_ADS1115 ads;

// millis
long long lastSend;
long long sendDelay=1000; // 100 ms
long long lastMqttReconnect;
long long mqttReconnectDelay=5000; // 5 s
long long lastReadPotentiometer;
long long readPotentiometerDelay=100; // 100 ms

// calibrate potentiometer
int finger0min = 27000;
int finger0max = 21970;
int finger1min = 27000;
int finger1max = 24190;
int finger2min = 27000;
int finger2max = 20350;
int finger3min = 27000;
int finger3max = 22400;

// global var
int finger0 = 0;
int finger1 = 0;
int finger2 = 0;
int finger3 = 0;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;


// funtion prototype
void update();
void getChipID();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void mqttSetup();
void updateMQTT();
void readPotentiometer();
void updateReadPotentiometer();
void sendMQTTfinger();
void updateSendMQTTfinger();

void setup() {
  M5.begin();
  
  // display startup
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Starting...");
  M5.Lcd.println("Connecting to " + String(ssid) + "...");

  // connect to wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  // display wifi connected
  M5.Lcd.println("Connected");
  M5.Lcd.println("IP address: " + WiFi.localIP().toString());

  // connect to mqtt
  mqttSetup();

  // display mqtt connected
  M5.Lcd.println("MQTT connected");
  M5.Lcd.println("Chip ID: " + String(chipId));

  // start ads1115
  ads.begin();

  // display ads1115 connected
  M5.Lcd.println("ADS1115 connected");

  // display ready
  M5.Lcd.println("Ready");
}

void loop() {
  update();
}

void update(){
  // M5.update();
  updateMQTT();
  // updateReadPotentiometer();
  updateSendMQTTfinger();
}

void getChipID(){
  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  if(((millis()-lastMqttReconnect)>=mqttReconnectDelay)) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect("M5-KMITL-02", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
    }
    lastMqttReconnect=millis();
  }
}

void mqttSetup(){
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
  if (mqtt.connect("M5-KMITL-02", mqtt_user, mqtt_password)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqtt.state());
    Serial.println(" try again in 5 seconds");
  }
}

void updateMQTT(){
  if (!mqtt.connected() && WiFi.isConnected()) {
    reconnect();
  }
  mqtt.loop();
}

void updateReadPotentiometer(){
  if(((millis()-lastReadPotentiometer)>=readPotentiometerDelay)) {
    readPotentiometer();
    lastReadPotentiometer=millis();
  }
}

void readPotentiometer(){
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  // // print raw value
  // Serial.print("Finger 0: ");
  // Serial.print(adc0);
  // Serial.print(" | Finger 1: ");
  // Serial.print(adc1);
  // Serial.print(" | Finger 2: ");
  // Serial.print(adc2);
  // Serial.print(" | Finger 3: ");
  // Serial.println(adc3);

  // map value to 0-1024
  finger0 = map(adc0, finger0min, finger0max, 0, 1024);
  finger1 = map(adc1, finger1min, finger1max, 0, 1024);
  finger2 = map(adc2, finger2min, finger2max, 0, 1024);
  finger3 = map(adc3, finger3min, finger3max, 0, 1024);

  // // print value
  // Serial.print("Finger 0: ");
  // Serial.println(finger0);
  // Serial.print("Finger 1: ");
  // Serial.println(finger1);
  // Serial.print("Finger 2: ");
  // Serial.println(finger2);
  // Serial.print("Finger 3: ");
  // Serial.println(finger3);
  // Serial.println("----------------");
  
}

void sendMQTTfinger(){

  readPotentiometer();

  // make json string
  JSONVar packageJSON;
  packageJSON["f1"] = finger0;
  packageJSON["f2"] = finger1;
  packageJSON["f3"] = finger2;
  packageJSON["f4"] = finger3;
  String packageString = JSON.stringify(packageJSON);
  Serial.println(packageString);
  mqtt.publish("KMITL-02/finger", packageString.c_str());
  Serial.println("Send finger");
}

void updateSendMQTTfinger(){
  if(((millis()-lastSend)>=sendDelay)) {
    sendMQTTfinger();
    lastSend=millis();
  }
}

void updateSendMQTTGyro(){
  if(((millis()-lastSend)>=sendDelay)) {
    sendMQTTGyro();
    lastSend=millis();
  }
}

void sendMQTTGyro(){
  // get gyro data
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  
  // make json string
  JSONVar packageJSON;
  packageJSON["x"] = gyroX;
  packageJSON["y"] = gyroY;
  packageJSON["z"] = gyroZ;
  String packageString = JSON.stringify(packageJSON);
  Serial.println(packageString);
  mqtt.publish("KMITL-02/gyro", packageString.c_str());
  Serial.println("Send gyro");
}


