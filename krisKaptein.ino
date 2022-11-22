#include <Arduino.h>
#include "UbidotsEsp32Mqtt.h"

//definerer pins------------------------------------------------------------------------

const int buttonPin = 23; //Pin koblet til pushbutton. Brukes i: pinInitialization()

//pins koblet til led lys
const int greenLedPin = 33; //Brukes i: pinInitialization()
const int redLedPin = 27; //Brukes i: pinInitialization()
const int potPin = 34; //Brukes i: pinInitialization(), motorControlHelp()

//pins koblet til motor
//const int enA = 25; //brukes i: pinInitialization(), motorControlHelp(), motorControl()
//const int in1 = 14; //brukes i: pinInitialization(), motorControlHelp(), motorControl()
//const int in2 = 12; //brukes i: pinInitialization(), motorControlHelp(), motorControl()
// fake motor
const int FakeMotor = 26;

//--------------------------------------------------------------------------------------


//definerer variabler---------------------------------------------------------------------------------------------------------

int mobStatus = 0; //kan endres i: callback(), buttonPress(). Brukt i: callback(), buttonPress(), ledControl(), motorControl()
unsigned long holdTime = 0;
unsigned long startTime = 0;
//definerer ubidots variabler. Alle brukes i ubidotsInitialization()
const char *UBIDOTS_TOKEN = "BBFF-H2Wsoo1L7XxRmN16prd6DuPAreozkW"; 
const char *WIFI_SSID = "Telenor9854din";      
const char *WIFI_PASS = "bmflipzvsktsu";      
const char *StopMOB = "StopMOB";   //brukes i: ubidotsInitialization(), publishToUbidots()
const char *MOB = "MOB"; //brukes i: ubidotsInitialization(), publishToUbidots()
const char *ACK = "ACK";
const char *StateButton = "Safety_button";

//----------------------------------------------------------------------------------------------------------------------------





//definerer timers-------------------------------------------------

unsigned long buttonTimer = 0; //timer for å forhindre knappeprell. Brukes i: buttonPress(). Endres i: buttonPress()
unsigned long blinkTimer = 0; //timer for blinking av led. Brukes i: ledControl(). Endres i: ledControl()

//-----------------------------------------------------------------





//instansiering av objekter------

Ubidots ubidots(UBIDOTS_TOKEN);

//--------------------------------


//funksjoner knyttet til ubidots ----------------------------------------------------

//funksjon som oppdaterer mobStatus hvis det skjer en endring i mobStatus via ubidots
void callback(char *topic, byte *payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  char state = payload[0];
  switch(state){
    case'0':
    mobStatus = 0;
    publishToUbidots();
    break;

    case'1':
    mobStatus = 1;
    publishToUbidots();
    break;

    case'2':
    mobStatus = 2;
    publishToUbidots();
    break;
  }
}

//initialiserer ubidots. Bruker: callback()
void ubidotsInitialization(){
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(StopMOB, MOB);
}

//funksjon som publiserer til mobStatus til ubidots
void publishToUbidots(){
    if (!ubidots.connected()){
      ubidots.connect();
    }
    if (ubidots.connected()){
      ubidots.add(ACK, mobStatus);
      ubidots.publish(StopMOB);
    }
    else{
      ubidots.disconnect();
    }
}

//-----------------------------------------------------------------------------------





//definerer andre funksjoner---------------------------------------------------------------------

//funksjon som intisialiserer pins
void pinInitialization(){
  //button pin
  pinMode(buttonPin, INPUT_PULLUP); 

  //led pins
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  //potentiometer pin
  pinMode(potPin, INPUT);

  //motor pins
  //pinMode(enA, OUTPUT);
  //pinMode(in1, OUTPUT);
  //pinMode(in2, OUTPUT);
  //Fake motor
  pinMode(FakeMotor, OUTPUT);
}

//funksjon som oppdaterer mob status i henold til knappetrykk. Bruker: pubishToUbidots()
void buttonPress(){
  int mobStatus1 = mobStatus;
  holdTime = 0;
  if ((millis() - buttonTimer) >= 100){
    if (digitalRead(buttonPin)){
      startTime = millis();
      while(digitalRead(buttonPin)) {
        holdTime = millis();
      }
      mobStatus += 1;
      if(mobStatus >= 3) {
        mobStatus = 0;
      }  
      if(holdTime - startTime >= 2000) {
        mobStatus = 3;
        publishToUbidots();
        while(millis() - startTime <= 12000) {
          digitalWrite(redLedPin, HIGH);
          digitalWrite(greenLedPin, HIGH);
          if (!ubidots.connected()){
            ubidots.connect();
          }
          ubidots.loop();
        }
        digitalWrite(redLedPin, LOW);
        digitalWrite(greenLedPin, LOW);
        mobStatus = 0;
        publishToUbidots();
      } 
    }
    buttonTimer = millis();
  }
  
  int mobStatus2 = mobStatus;

  if (mobStatus1 != mobStatus2){
    publishToUbidots();
  }
}


//funksjon som styrer led lysene.
void ledControl(){
  if (mobStatus == 0){
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
  } 
  else if (mobStatus == 1){
    digitalWrite(greenLedPin, LOW);

    if ((millis() - blinkTimer) >= 500){
      switch (digitalRead(redLedPin)){
        case 1:
        digitalWrite(redLedPin, LOW);
        break;
        
        case 0:
        digitalWrite(redLedPin, HIGH);
        break;
      }
      blinkTimer = millis();
    }
  }
  else{
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
}
/*
//funksjon som styrer motorens hastighet
void motorControlHelp(){
  int potReading = analogRead(potPin);

  if (potReading >= 2048){
    potReading = map(potReading,2048,4095,0,255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(enA,potReading);
  }
  else{
    potReading = map(potReading,0,2047,0,255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(enA,potReading);
  }
}
*/
void FakeMotorControlHelp() {
  int potReading = analogRead(potPin);
  
  potReading = map(potReading,2048,4095,0,255);
  analogWrite(FakeMotor, potReading);
}

//funksjon som sjekker hva mob status er, og styrer motoren deretter. Bruker: motorControlHelp()
/*
void motorControl(){
  switch(mobStatus){
    case 0:
    motorControlHelp();
    break;

    case 1:
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(enA,LOW);
    break;

    case 2:
    motorControlHelp();
    break;
  }
}
*/
void FakeMotorControl() {
  switch(mobStatus){
    case 0:
    FakeMotorControlHelp();
    break;

    case 1:
    analogWrite(FakeMotor, 0);
    break;

    case 2:
    FakeMotorControlHelp();
    break;
  }
}

//-----------------------------------------------------------------------------------------------

void setup() {

  Serial.begin(9600); //Starter seriekommunikasjon
  pinInitialization(); //initialiserer pins
  ubidotsInitialization(); //initiaiserer ubidots

}


void loop() {
  if (!ubidots.connected()){
    ubidots.connect();
  }
  ubidots.loop();
  buttonPress(); //lytter etter knappetrykk
  //Serial.println(digitalRead(buttonPin));
  //motorControl(); //styrer motor
  FakeMotorControl();
  
  ledControl(); //styrer led lys
    
}
