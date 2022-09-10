// include lib
#include <Arduino.h>
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_JSON.h>
#include <Adafruit_PWMServoDriver.h>
#include "Madgwick_Quaternion.h"

// include Font
#include "quark24.h"
#include "quark12.h"
#include "quark9.h"

// servo 
#define SERVOMIN 150// Minimum pulse length count out of 4096.
#define SERVOMAX 600 // Maximum pulse length count out of 4096.

// config network
const char* ssid = "your ssid";
const char* password = "your password";

// config mqtt
const char* mqtt_server = "your mqtt server";
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
Madgwick_Quaternion M_Q_Filter;
Adafruit_PWMServoDriver pwmServo = Adafruit_PWMServoDriver();

// millis
long long lastSend;
long long sendDelay=400; // 100 ms
long long lastMqttReconnect;
long long mqttReconnectDelay=5000; // 5 s
long long lastReadPotentiometer;
long long readPotentiometerDelay=100; // 100 ms
long long lastReadIMU;
long long readIMUDelay=10; // 10 ms
long long lastCheckGlove;
long long checkDelay = 700;
long long lastCheckLock;
long long checkLockDelay = 100;

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
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float quat_w = 0.0F;
float quat_x = 0.0F;
float quat_y = 0.0F;
float quat_z = 0.0F;

// sensor imu
float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];
float magOffset[3];
float magmax[3];
float magmin[3];
uint8_t setup_flag = 1;
uint8_t action_flag = 1;
float heading = 0;
uint8_t smoothen = 100;
float strength = 1;

// servo
int servoNo0 = 0;      // Defines a counter for servos. count0-n
int servoNo1 = 1;      // Defines a counter for servos. count0-n
int servoNo2 = 4;      // Defines a counter for servos. count0-n
int servoNo3 = 3;      // Defines a counter for servos. count0-n
int servoNoMax = 5;   // maximum n servo
int freqServo = 60;
int checkStatusServo = 0;

// // calibrate servo
// int servo0min = 0;
// int servo0max = 180;
// int servo1min = 0;
// int servo1max = 180;
// int servo2min = 0;
// int servo2max = 180;
// int servo3min = 0;
// int servo3max = 180;

// grasp
boolean robotGraspState = false;
boolean gloveGraspState = false;
boolean servoState = false;

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
void sendMQTTGyro();
void calibrate6886();
void calibrate_waiting(uint32_t timeout);
void applycalibration();
void IMU_Update();
void updateSendMQTTGyro();
void updateReadIMU();
void updateReCalibrate();
void updateGrasp();
void servoLock();
void servoUnLock();
void updateLockServo();

void setup() {

  // init M5StickCPlus
  M5.begin();
  M5.Imu.Init();   

  pwmServo.begin();
  pwmServo.setPWMFreq(freqServo);

  
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

  // calibrate imu
  calibrate6886();

  // begin imu
  M_Q_Filter.begin(smoothen); //100Hz

  // calibrate imu again
  calibrate_waiting(10);

  // set servo to 0
  for (int pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
 {
    pwmServo.setPWM(servoNo0, 0, pulselen);
    pwmServo.setPWM(servoNo1, 0, pulselen);
    pwmServo.setPWM(servoNo2, 0, pulselen);
    pwmServo.setPWM(servoNo3, 0, pulselen);
  }

  // delay(5000);

  // //set servo to 90
  // for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  // {
  //   pwmServo.setPWM(servoNo0, 0, pulselen);
  //   pwmServo.setPWM(servoNo1, 0, pulselen);
  //   pwmServo.setPWM(servoNo2, 0, pulselen);
  //   pwmServo.setPWM(servoNo3, 0, pulselen);
  // }

  // display ready
  M5.Lcd.println("Ready");
}

void loop() {
  update();
}

void update(){
  M5.update();
  updateMQTT();
  updateReadPotentiometer();
  // updateSendMQTTfinger();
  updateReCalibrate();
  updateSendMQTTGyro();
  updateReadIMU();
  updateGrasp();
  updateLockServo();
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

  // payload -> string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // topic -> string
  String topicStr = String(topic);

  if(topicStr == "KMITL-02/grippingStatus"){
    if(message == "graspOk"){
      robotGraspState = true;
    }
    else if(message == "graspFail"){
      robotGraspState = false;
    }
  }
  if(topicStr == "KMITL-02/grippingStatus"){
    if(message == "releaseOk"){
      robotGraspState = false;
    }
    // else if(message == "releaseFail"){
    //   robotGraspState = false;
    // }
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

    // subscribe to topic
    mqtt.subscribe("KMITL-02/grippingStatus");

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

  // map value to 0-1024
  finger0 = map(adc0, finger0min, finger0max, 0, 1024);
  finger1 = map(adc1, finger1min, finger1max, 0, 1024);
  finger2 = map(adc2, finger2min, finger2max, 0, 1024);
  finger3 = map(adc3, finger3min, finger3max, 0, 1024);

  // print value
  // Serial.print("Finger 0: ");
  // Serial.print(finger0);
  // Serial.print(" | Finger 1: ");
  // Serial.print(finger1);
  // Serial.print(" | Finger 2: ");
  // Serial.print(finger2);
  // Serial.print(" | Finger 3: ");
  // Serial.println(finger3);
  
}

void sendMQTTfinger(){

  readPotentiometer();

  // make json string
  JSONVar packageJSON2;
  packageJSON2["f1"] = finger0;
  packageJSON2["f2"] = finger1;
  packageJSON2["f3"] = finger2;
  packageJSON2["f4"] = finger3;
  String packageString = JSON.stringify(packageJSON2);
  // Serial.println(packageString);
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
  
  // print value
  // Serial.print("quat_x: ");
  // Serial.print(quat_x);
  // Serial.print("\tquat_y: ");
  // Serial.print(quat_y);
  // Serial.print("\tquat_z: ");
  // Serial.println(quat_z);

  // up
  if (quat_x > 0.35 && quat_y < 0.35 && quat_y > -0.35 && quat_z < 0.35 && quat_z > -0.35) {
    Serial.println("up");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0;
    packageJSON["dY"] = 0;
    packageJSON["dZ"] = 0.01;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // down
  else if (quat_x < -0.35 && quat_y < 0.35 && quat_y > -0.35 && quat_z < 0.35 && quat_z > -0.35) {
    Serial.println("down");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0;
    packageJSON["dY"] = 0;
    packageJSON["dZ"] = -0.01;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // left
  else if (quat_y < -0.35 && quat_x < 0.35 && quat_x > -0.35 && quat_z < 0.35 && quat_z > -0.35) {
    Serial.println("left");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0;
    packageJSON["dY"] = 0.01;
    packageJSON["dZ"] = 0;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // right
  else if (quat_y > 0.35 && quat_x < 0.35 && quat_x > -0.35 && quat_z < 0.35 && quat_z > -0.35) {
    Serial.println("right");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0;
    packageJSON["dY"] = -0.01;
    packageJSON["dZ"] = 0;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // forward
  else if (quat_z > 0.35 && quat_x < 0.35 && quat_x > -0.35 && quat_y < 0.35 && quat_y > -0.35) {
    Serial.println("forward");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0.01;
    packageJSON["dY"] = 0;
    packageJSON["dZ"] = 0;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // backward
  else if (quat_z < -0.35 && quat_x < 0.35 && quat_x > -0.35 && quat_y < 0.35 && quat_y > -0.35) {
    Serial.println("backward");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = -0.01;
    packageJSON["dY"] = 0;
    packageJSON["dZ"] = 0;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
  // stop
  else {
    Serial.println("stop");
    // make json string
    JSONVar packageJSON;
    packageJSON["dX"] = 0;
    packageJSON["dY"] = 0;
    packageJSON["dZ"] = 0;
    String packageString = JSON.stringify(packageJSON);

    mqtt.publish("KMITL-02/arm", packageString.c_str());
  }
}

void calibrate6886(){
  double gyroSum[3];
  double accSum[3];
  int counter = 5;

  Serial.println("Calibrating...");

  M5.Imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.Imu.getAccelData(&acc[0], &acc[1], &acc[2]);

  gyroOffset[0] = gyro[0];
  gyroOffset[1] = gyro[1];
  gyroOffset[2] = gyro[2];
  accOffset[0] = acc[0];
  accOffset[1] = acc[1];
  accOffset[2] = acc[2] - 1.0;  //Gravitational Acceleration 1G, assuming that the M5 button is facing upward

  Serial.println("Calibration Done!");
  Serial.print("gyroOffset[0]: ");
  Serial.print(gyroOffset[0]);
  Serial.print(" | gyroOffset[1]: ");
  Serial.print(gyroOffset[1]);
  Serial.print(" | gyroOffset[2]: ");
  Serial.println(gyroOffset[2]);
  Serial.print("accOffset[0]: ");
  Serial.print(accOffset[0]);
  Serial.print(" | accOffset[1]: ");
  Serial.print(accOffset[1]);
  Serial.print(" | accOffset[2]: ");
  Serial.println(accOffset[2]);

}

void calibrate_waiting(uint32_t timeout){
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    if (digitalRead(M5_BUTTON_HOME) == LOW) {
      setup_flag = 1;
      while (digitalRead(M5_BUTTON_HOME) == LOW);
      break;
    }
    delay(100);
  }

}

void applycalibration(){
  M5.Imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.Imu.getAccelData(&acc[0], &acc[1], &acc[2]);

  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];

  //fake magnetometer data cuz MPU6886 doesn't come with BMM 150 chip
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
}

void IMU_Update() {
  applycalibration();

  heading = atan2(mag[0], mag[1]);
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;

  M_Q_Filter.update(gyro[0]*strength, gyro[1]*strength, gyro[2]*strength, acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);

  roll = M_Q_Filter.getRoll();
  pitch = M_Q_Filter.getPitch();
  yaw   = M_Q_Filter.getYaw();

  //for quaternion output (by zeketan)
  quat_w = M_Q_Filter.getQuat_W();
  quat_x = M_Q_Filter.getQuat_X();
  quat_y = M_Q_Filter.getQuat_Y();
  quat_z = M_Q_Filter.getQuat_Z(); //-0.005 anti yaw drifting

  // Serial.print("quat_x: ");
  // Serial.print(quat_x);
  // Serial.print("\tquat_y: ");
  // Serial.print(quat_y);
  // Serial.print("\tquat_z: ");
  // Serial.println(quat_z);
}

void updateReadIMU(){
  if ((millis() - lastReadIMU) >= readIMUDelay) {
    lastReadIMU = millis();
    IMU_Update();
  }
}

void updateReCalibrate(){
  // Button A
  if (M5.BtnA.wasReleased()) {
    // reset esp32
    Serial.println("Reset ESP32");
    ESP.restart();
  }
}

void updateGrasp(){
  // Serial.println(finger0+finger1+finger2+finger3);
  if ((millis()-lastCheckGlove)>=checkDelay){
    // Serial.println(finger0+finger1+finger2+finger3);
    if ((finger0+finger1+finger2+finger3)>=1400){
      gloveGraspState = true;
      Serial.println("Lockkkkkkkk");

      // test only
      // robotGraspState = true;

      // Serial.println("grasp!");
      // make json
      JSONVar packageJSON;
      packageJSON["gripper"] = "on";
      String packageString = JSON.stringify(packageJSON);

      mqtt.publish("KMITL-02/gripper", packageString.c_str());
    }
    else {
      if (gloveGraspState){
        gloveGraspState = false;

        // test only
        // robotGraspState = false;

        // Serial.println("release!");
        // make json
        JSONVar packageJSON;
        packageJSON["gripper"] = "off";
        String packageString = JSON.stringify(packageJSON);
        

        mqtt.publish("KMITL-02/gripper", packageString.c_str());
      }
      // // make json
      // JSONVar packageJSON;
      // packageJSON["gripper"] = "off";
      // String packageString = JSON.stringify(packageJSON);

      // mqtt.publish("KMITL-02/gripper", packageString.c_str());
    }
    lastCheckGlove = millis();
  }
}

void updateLockServo(){
  if ((millis()-lastCheckLock)>=checkDelay){
    if (robotGraspState && !servoState){
      servoLock();
    }
    else if (!robotGraspState && servoState){
      servoUnLock();
    }
    lastCheckLock = millis();
  }

}

void servoLock(){
  servoState = true;
  for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
    pwmServo.setPWM(servoNo0, 0, pulselen);
    pwmServo.setPWM(servoNo1, 0, pulselen);
    pwmServo.setPWM(servoNo2, 0, pulselen);
    pwmServo.setPWM(servoNo3, 0, pulselen);
  }
}

void servoUnLock(){
  servoState = false;
  for (int pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
    pwmServo.setPWM(servoNo0, 0, pulselen);
    pwmServo.setPWM(servoNo1, 0, pulselen);
    pwmServo.setPWM(servoNo2, 0, pulselen);
    pwmServo.setPWM(servoNo3, 0, pulselen);
  }
}


