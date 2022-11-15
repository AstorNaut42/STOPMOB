#include <Arduino.h>
#include "UbidotsEsp32Mqtt.h"





//definerer pins------------------------------------------------------------------------

const int buttonPin = 32; //Pin koblet til pushbutton. Brukes i: pinInitialization()

//pins koblet til led lys
const int greenLedPin = 33; //Brukes i: pinInitialization()
const int redLedPin = 27; //Brukes i: pinInitialization()
const int potPin = 13; //Brukes i: pinInitialization(), motorControlHelp()

//pins koblet til motor
const int enA = 25; //brukes i: pinInitialization(), motorControlHelp(), motorControl()
const int in1 = 14; //brukes i: pinInitialization(), motorControlHelp(), motorControl()
const int in2 = 12; //brukes i: pinInitialization(), motorControlHelp(), motorControl()

//--------------------------------------------------------------------------------------





//definerer variabler---------------------------------------------------------------------------------------------------------

int mobStatus; //kan endres i: callback(), buttonPress(). Brukt i: callback(), buttonPress(), ledControl(), motorControl()

//definerer ubidots variabler. Alle brukes i ubidotsInitialization()
const char *UBIDOTS_TOKEN = ""; 
const char *WIFI_SSID = "";      
const char *WIFI_PASS = "";      
const char *DEVICE_LABEL = "";   //brukes i: ubidotsInitialization(), publishToUbidots()
const char *VARIABLE_LABEL = "MOB"; //brukes i: ubidotsInitialization(), publishToUbidots()

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
  char mobStatus1 = payload[0];
  switch(mobStatus1){
    case'0':
    mobStatus = 0;
    break;

    case'1':
    mobStatus = 1;
    break;

    case'2':
    mobStatus = 2;
    break;
  }
}

//initialiserer ubidots. Bruker: callback()
void ubidotsInitialization(){
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL);
}

//funksjon som publiserer til mobStatus til ubidots
void publishToUbidots(){
    if (!ubidots.connected()){
      ubidots.connect();
    }
    if (ubidots.connected()){
      int value = mobStatus;
      ubidots.add(VARIABLE_LABEL, value); 
      ubidots.publish(DEVICE_LABEL);
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
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

//funksjon som oppdaterer mob status i henold til knappetrykk. Bruker: pubishToUbidots()
void buttonPress(){
int mobStatus1 = mobStatus;

  if ((millis() - buttonTimer) >= 100){
    if (digitalRead(buttonPin) == 0 ){
      switch (mobStatus){
        case 0:
          mobStatus = 1;
        break;

        case 1:
          mobStatus = 2;
        break;

        case 2:
        mobStatus = 0;
        break;
      }      
      while(digitalRead(buttonPin) == 0){

      }

      buttonTimer = millis();
    }
    else{
      buttonTimer = millis();
    }
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

//funksjon som sjekker hva mob status er, og styrer motoren deretter. Bruker: motorControlHelp()
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

//-----------------------------------------------------------------------------------------------





void setup() {

  Serial.begin(115200); //Starter seriekommunikasjon
  
  pinInitialization(); //initialiserer pins

  ubidotsInitialization(); //initiaiserer ubidots

}





void loop() {

  buttonPress(); //lytter etter knappetrykk

  motorControl(); //styrer motor

  ledControl(); //styrer led lys
    
}