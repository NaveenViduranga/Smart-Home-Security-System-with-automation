#include <Arduino.h>
//#include <WiFi.h>               //we are using the ESP32
#include <ESP8266WiFi.h>      // uncomment this line if you are using esp8266 and comment the line above
#include <Firebase_ESP_Client.h>
#include <string.h>
#include <Servo.h>

Servo myServo; 

String Regulator1, relay_systemarm, relay_gate_controll ,relay_gate_light,relay_power, relay_sos ;
String Regulator2, relay_light_living_room, relay_Kitchen_light, relay_room_light, relay_Fan1 ;

#define light_gate D1
#define light_living_room D2
#define light_kitchen D5
#define light_room D8
#define FAN D7
#define servo D3

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "SLT-4G_16421C"  // N_Binath_Dialog
#define WIFI_PASSWORD "12345678Ab"  //12345678Ab  n@161ANGA/!aS

// Insert Firebase project API Key
#define API_KEY "IzaSyDpWg63KrqoXMBLvKHNWejw0"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://smart-homefaut-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;                     //since we are doing an anonymous sign in 

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

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.print("right ");

  myServo.attach(servo);

  pinMode(light_gate, OUTPUT);
  
  pinMode(light_living_room, OUTPUT);
  pinMode(light_kitchen, OUTPUT);
  pinMode(light_room, OUTPUT);
  pinMode(FAN, OUTPUT);
  Serial.print("pinmodeok ");
  
}

void loop(){
 // if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
 //   sendDataPrevMillis = millis();

  if ( Firebase.RTDB.getString(&fbdo, "gate_light") ){
    if (fbdo.dataType() == "string"){
        relay_gate_light = fbdo.stringData();
        Serial.print("relay_gate_light: ");
        Serial.println(relay_gate_light);
    }
  }
   if ( Firebase.RTDB.getString(&fbdo, "living_room_light") ){
    if (fbdo.dataType() == "string"){
        relay_light_living_room = fbdo.stringData();
        Serial.print("relay_light_living_room: ");
        Serial.println(relay_light_living_room);
    }
  }

      if ( Firebase.RTDB.getString(&fbdo, "Kitchen_light") ){
    if (fbdo.dataType() == "string"){
        relay_Kitchen_light = fbdo.stringData();
        Serial.print("relay_Kitchen_light: ");
        Serial.println(relay_Kitchen_light);
    }
  }

      if ( Firebase.RTDB.getString(&fbdo, "room_light") ){
    if (fbdo.dataType() == "string"){
        relay_room_light = fbdo.stringData();
        Serial.print("relay_room_light :");
        Serial.println(relay_room_light);
    }
  }

      if ( Firebase.RTDB.getString(&fbdo, "gate_controll") ){
    if (fbdo.dataType() == "string"){
        relay_gate_controll = fbdo.stringData();
        Serial.print("relay_gate_controll : ");
        Serial.println(relay_gate_controll);
    }
  }

      if ( Firebase.RTDB.getString(&fbdo, "Fan1") ){
    if (fbdo.dataType() == "string"){
        relay_Fan1 = fbdo.stringData();
        Serial.print("relay_Fan1 :");
        Serial.println(relay_Fan1);
    }
  }

  //}

 automation();

}


void automation()
{
  
  if(relay_gate_controll == "true")
  {
    digitalWrite(light_gate,HIGH);
    myServo.write(10);
    delay(500);
    myServo.write(90);
    if (Firebase.RTDB.setString(&fbdo, "gate_controll", "none")){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    Serial.println("Gate_opening");
  }
  if(relay_gate_controll == "false")
  {
    digitalWrite(light_gate,HIGH);
    myServo.write(170);
    delay(500);
    myServo.write(90);
      if (Firebase.RTDB.setString(&fbdo, "gate_controll", "none")){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    Serial.println("Gate_closing");
  }
  if(relay_gate_controll == "none")
  {
    digitalWrite(light_gate,LOW);
    myServo.write(90);
    Serial.println("hold");
  }
  
  if(relay_gate_light == "true")
  {
    digitalWrite(light_gate,HIGH);
    Serial.println("Gate_light ON");
  }
  if(relay_gate_light == "false")
  {
    digitalWrite(light_gate,LOW);
    Serial.println("Gate_light OFF");
  }
   if(relay_light_living_room == "true")
  {
    digitalWrite(light_living_room,HIGH);
    Serial.println("light_living_room ON");
  }
  if(relay_light_living_room == "false")
  {
    digitalWrite(light_living_room,LOW);
    Serial.println("light_living_room OFF");
  }
   if(relay_Kitchen_light == "true")
  {
    digitalWrite(light_kitchen,HIGH);
    Serial.println("light_kitchen ON");
  }
  if(relay_Kitchen_light == "false")
  {
    digitalWrite(light_kitchen,LOW);
    Serial.println("light_kitchen OFF");
  }
   if(relay_room_light == "true")
  {
    digitalWrite(light_room,HIGH);
    Serial.println("light_room ON");
  }
  if(relay_room_light == "false")
  {
    digitalWrite(light_room,LOW);
    Serial.println("light_room OFF");
  }
  if(relay_Fan1 == "true")
  {
    digitalWrite(FAN,LOW);
    Serial.println("Fan -1 ON");
  }
  if(relay_Fan1 == "false")
  {
    digitalWrite(FAN,LOW);
    Serial.println("Fan -1 OFF");
  }

}

