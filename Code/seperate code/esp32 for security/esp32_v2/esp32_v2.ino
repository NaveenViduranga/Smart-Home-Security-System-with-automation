#include <Arduino.h>
#include <WiFi.h>               //we are using the ESP32
#include <Firebase_ESP_Client.h>
#include <string.h>
#include <Servo.h>
#include <HardwareSerial.h>
HardwareSerial mySerial(2); // Assuming SIM900 Tx & Rx is connected to ESP32's UART2 (GPIO 17 & GPIO 16)

Servo myServo; 

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "SLT-4G_16421C"
#define WIFI_PASSWORD "12345678"

// Insert Firebase project API Key
#define API_KEY "IzaSyDpWg63KqXMBLvKHNWejw30"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://smart-homeult-rtdb.firebaseio.com/" 

String Regulator1, relay_systemarm, relay_gate_controll ,relay_gate_light,relay_power, relay_sos ;
String Regulator2, relay_light_living_room, relay_Kitchen_light, relay_room_light, relay_Fan1 ;


#define power 15 
#define living_room_door_sensor 5 //
#define kitchen_door_sensor 4 
#define ldr_zone1 26
#define ldr_zone2 18
#define motion_sensor_1 19
#define motion_sensor_2 21
#define servo 22
#define light_gate 32
#define light_living_room 33
#define light_kitchen 27
#define light_room 2
#define buzzor 25
#define WifiLed 12
#define FAN 13
#define gas 14
#define gate_closed 34
#define gate_opened 35


// 34 35 input only
// 23
volatile bool motionInterrupt1 = false;
volatile bool motionInterrupt2 = false;
volatile bool doorInterrupt_1 = false;
volatile bool doorInterrupt_2 = false;
volatile bool laserInterrupt_1 = false;
volatile bool laserInterrupt_2 = false;
volatile bool gateopenInterrupt= false;
volatile bool gatecloseInterrupt= false;
volatile bool gasInterrupt= false;

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 5000; //


//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;               


// interrupt serving root for sensors
void IRAM_ATTR motionSensorISR1() {
  motionInterrupt1 = true;
    Serial.println("Motion detected on sensor 1");
    buzzorON();
}

void IRAM_ATTR motionSensorISR2() {
  motionInterrupt2 = true;
    Serial.println("Motion detected on sensor 2");
    buzzorON();
}

void IRAM_ATTR doorDetected_livingRoomISR() {
  doorInterrupt_1 = true;
    Serial.println("Door detected 1");
    buzzorON();
}

void IRAM_ATTR doorDetected_kitchenISR() {
  doorInterrupt_2 = true;
    Serial.println("Door detected 2");
    buzzorON();
}
void IRAM_ATTR gasDetectedISR() {
  gasInterrupt = true;
    Serial.println("Gas Detected");
    buzzorON();
}

void IRAM_ATTR Laser1_ISR() {
  laserInterrupt_1 = true;
  Serial.println("laser detected zone 1");
  buzzorON();
}
void IRAM_ATTR Laser2_ISR() {
  laserInterrupt_2 = true;
  Serial.println("laser detected zone 2");
  buzzorON();
}



void setup(){
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
//sim900
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // Assuming ESP32's UART2 is used

  Serial.println("SIM 900 Initializing..."); 
  delay(1000);

  mySerial.println("AT"); // Handshaking with SIM900
  updateSerial();

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  
  mySerial.println("AT+CMGS=\"+9476\""); // Change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  
  mySerial.print("POwer ON or Reset"); // Text content
  updateSerial();
  mySerial.write(26); // End of message character

  // Assign the api key 
  config.api_key = API_KEY;

  // Assign the RTDB URL 
  config.database_url = DATABASE_URL;

  //
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("firebase connected");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if ( Firebase.RTDB.getString(&fbdo, "system_arm") ){
    if (fbdo.dataType() == "string"){
        relay_systemarm = fbdo.stringData();
    }
  }
  Firebase.RTDB.setString(&fbdo, "sos", "false");
  Firebase.RTDB.setString(&fbdo, "gate_controll", "false"); 
  Firebase.RTDB.setString(&fbdo, "power", "false");
  Serial.println("Firebase done");   

  pinMode(power, INPUT);
  pinMode(living_room_door_sensor,INPUT_PULLUP);
  pinMode(kitchen_door_sensor,INPUT_PULLUP);
  pinMode(ldr_zone1, INPUT);
  pinMode(ldr_zone2,INPUT);
  pinMode(motion_sensor_1,INPUT);
  pinMode(motion_sensor_2,INPUT);
  pinMode(servo, OUTPUT);
  pinMode(light_gate, OUTPUT);
  pinMode(light_living_room, OUTPUT);
  pinMode(light_kitchen, OUTPUT);
  pinMode(light_room, OUTPUT);
  pinMode(buzzor, OUTPUT);
  pinMode(WifiLed, OUTPUT);
  pinMode(gas, INPUT);
  pinMode(gate_closed, INPUT);
  pinMode(gate_opened, INPUT);
  Serial.println("pinmode ok");

  myServo.attach(servo);
   //initialize interrupt for sensors 
  attachInterrupt(digitalPinToInterrupt(motion_sensor_1), motionSensorISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(motion_sensor_2), motionSensorISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(living_room_door_sensor), doorDetected_livingRoomISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kitchen_door_sensor), doorDetected_kitchenISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gas),gasDetectedISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ldr_zone1), Laser1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ldr_zone2), Laser2_ISR, FALLING);

  Serial.println("interrupt init ok");
  //sendMessage("power on");

}

void loop(){
  //check system arm or not
  systemArmCheck();

  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(WifiLed,HIGH);  
  }

  //  Regulator1 = Firebase.getString("Regulator1");//get the variables
  if ( Firebase.RTDB.getString(&fbdo, "system_arm") ){
    if (fbdo.dataType() == "string"){
        relay_systemarm = fbdo.stringData();
    }
  }
  if ( Firebase.RTDB.getString(&fbdo, "sos") ){
    if (fbdo.dataType() == "string"){
        relay_sos = fbdo.stringData();
        if(relay_sos == "true"){
          digitalWrite(buzzor, HIGH);
          sendMessage("+9476", "SOS!!! SOS!!!");
        }
        else{
          digitalWrite(buzzor, LOW);
        }
    }
  }

 // relay_Fan1 = Firebase.RTDB.getString(&fbdo, "Fan1");
      if ( Firebase.RTDB.getString(&fbdo, "power") ){
    if (fbdo.dataType() == "string"){
        relay_power = fbdo.stringData();
    }
  }
  //do main task

  if (buzzerActive && millis() - buzzerStartTime >= buzzerDuration) {
    // Turn off the buzzer
    digitalWrite(buzzor, LOW);
    buzzerActive = false;

    // Reset the trigger flags
    motionInterrupt1 = false;
    motionInterrupt2 = false;
    doorInterrupt_1 = false;
    doorInterrupt_2 = false;
  }
  handle_toDo();

}

void systemArmCheck(){
    if(relay_systemarm == "false"){
    Serial.print("Disarmed!!!");
    digitalWrite(buzzor, LOW);
    detachInterrupt(motion_sensor_1);
    detachInterrupt(motion_sensor_2);
    detachInterrupt(living_room_door_sensor);
    detachInterrupt(kitchen_door_sensor);
    detachInterrupt(ldr_zone1);
    detachInterrupt(ldr_zone2);
    detachInterrupt(gas);
  }
  else{
  Serial.println("working...");
  attachInterrupt(digitalPinToInterrupt(motion_sensor_1), motionSensorISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(motion_sensor_2), motionSensorISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(living_room_door_sensor), doorDetected_livingRoomISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kitchen_door_sensor), doorDetected_kitchenISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gas),gasDetectedISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ldr_zone1), Laser1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ldr_zone2), Laser2_ISR, FALLING);
  }

}


void handle_toDo(){
  if (motionInterrupt1) {
   // Serial.println("Motion detected on sensor 1");
   Firebase.RTDB.setString(&fbdo, "system_arm", "threat");
   Firebase.RTDB.setString(&fbdo, "Notification/motion_1", "true");
   sendMessage("+9476", "motion triggerd zone 1");
    motionInterrupt1 = false; // Reset flag
  } 

    if (motionInterrupt2) {
   // Serial.println("Motion detected on sensor 2");
   Firebase.RTDB.setString(&fbdo, "system_arm", "threat");
   Firebase.RTDB.setString(&fbdo, "Notification/motion_2", "true");
   sendMessage("+9476", "motion triggerd zone 2");
    motionInterrupt2 = false; // Reset flag
  } 
 if (doorInterrupt_1) {
  //  Serial.println("Door detected 1");
  Firebase.RTDB.setString(&fbdo, "system_arm", "threat");
  Firebase.RTDB.setString(&fbdo, "Notification/Door_living", "true");
  sendMessage("+9476", "Door triggerd zone 1");
    doorInterrupt_1 = false; // Reset flag
  } 


  if (doorInterrupt_2) {
   // Serial.println("Door detected 2");
   Firebase.RTDB.setString(&fbdo, "system_arm", "threat");
   Firebase.RTDB.setString(&fbdo, "Notification/Door_kitchen", "true");
   sendMessage("+9476", "Door triggerd zone 2");
    doorInterrupt_2 = false; // Reset flag
  } 

  if (gasInterrupt) {
   Firebase.RTDB.setString(&fbdo, "system_arm", "threat");
   Firebase.RTDB.setString(&fbdo, "Notification/Door_kitchen", "true");
   sendMessage("+9476", "Gas Leakage");
    gasInterrupt = false; // Reset flag
  } 

  if (laserInterrupt_1) {
   Firebase.RTDB.setString(&fbdo, "Notification/laser", "true");
   sendMessage("+9476", "Laser detect zone 1");
    laserInterrupt_1 = false; // Reset flag
  }
  if (laserInterrupt_2) {
   Firebase.RTDB.setString(&fbdo, "Notification/laser", "true");
     sendMessage("+9476", "Laser detect zone 2");
    laserInterrupt_2 = false; // Reset flag
  }

}


void buzzorON(){
    if ((motionInterrupt1 || motionInterrupt2 || doorInterrupt_1 || doorInterrupt_2 || laserInterrupt_1 || gasInterrupt || laserInterrupt_2)/* && !buzzerActive */) {
      buzzerStartTime =0;
      // Turn on the buzzer
      digitalWrite(buzzor, HIGH);
      buzzerStartTime = millis();
      buzzerActive = true;
  }
}

void gatecontroll(){

  if(gateopenInterrupt= true){
    Serial.println("Gate opened");
    myServo.write(90);
  }
  if(gatecloseInterrupt= true){
    myServo.write(90);
    Serial.println("Gate closed");
  }
}


void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read()); // Forward what Serial received to SIM900
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read()); // Forward what SIM900 received to Serial Port
  }
}

void sendMessage(const char* phoneNumber, const char* message) {
  mySerial.println("AT+CMGS=\"+9476\""); // Replace with phoneNumber
  delay(1000);
  mySerial.print(message);
  delay(100);
  mySerial.write(26); // End of message character
  delay(1000);
}
