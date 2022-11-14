
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "UbidotsEsp32Mqtt.h"

// UBIDOTS
const char *TOKEN = "BBFF-H2Wsoo1L7XxRmN16prd6DuPAreozkW";  // Put here your Ubidots TOKEN
const char *WIFI = "MagnusTlf";      // Put here your Wi-Fi SSID
const char *WIFIpass = "qwertyui";      // Put here your Wi-Fi password
const char *StopMOB = "StopMOB";   // Put here your Device label to which data  will be published
Ubidots ubidots(TOKEN);
unsigned long timer;

// Varsling
const char *MOB = "MOB";
const char *ID = "MaggyJuicy";
//const char *ID = "MinVenn";
const int WarningState = 19;
const int WarningOFF = 18;
const int WarningON = 23;
bool allsafe = true;
bool healthCheck;
unsigned long timer2;
unsigned long timer3;
unsigned long timer4;
unsigned long timer5 = 0;
bool blinkState = false;
int count = 0;
float tempTot = 0;
int LEDcount = 0;


// Måling
const int photoPin = 34;
const char *LikelySurvivalTime = "LikelySurvivalTimeMinM";
//const char *LikelySurvivalTime = "LikelySurvivalTimeMinV";
const char *TEMPERATURE = "TEMPERATUREM";
//const char *TEMPERATURE = "TEMPERATUREV";
Adafruit_BME280 bme; // I2C


int howLong() {
  float temp = bme.readTemperature();
  float LikelySurvivalTime_mins;

  if(healthCheck == false) {
    timer2 = millis();
  }
  
  if(temp <= 0.3) {
    LikelySurvivalTime_mins = 15;
  } else if(0.3 < temp and temp <= 10) {
    LikelySurvivalTime_mins = 120;
  } else if(10 < temp and temp <= 15.5) {
    LikelySurvivalTime_mins = 240;
  } else if(15.5 < temp and temp <= 21) {
    LikelySurvivalTime_mins = 240;
  } else if(21 < temp) {
    LikelySurvivalTime_mins = 1020;
  }
  healthCheck = true;
  return (LikelySurvivalTime_mins - (((millis() - timer2)/1000)/60));
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  crewState(payload[0]);
  Serial.println();
}

void crewState(char state) {
  switch (state) {
    case '0':
    digitalWrite(WarningON, LOW);
    digitalWrite(WarningOFF, HIGH);
    blinkState = false;
    allsafe = true;
    count = 0;
    tempTot = 0;
    LEDcount = 0;
    break;

    case '1':
    digitalWrite(WarningOFF, LOW);
    blinkState = true;
    timer3 = millis();
    healthCheck = false;
    break;

    case '2':
    digitalWrite(WarningON, HIGH);
    digitalWrite(WarningOFF, LOW);
    blinkState = false;
    break;
  }
}

void blinkCrew() {
  if(millis() - timer3 >= 500) {
    digitalWrite(WarningON, !digitalRead(WarningState));
    timer3 = millis();
  }
}

void setup() {
  Serial.begin(9600);  

  // Varsling
  pinMode(WarningON, OUTPUT);
  pinMode(WarningOFF, OUTPUT);
  pinMode(WarningState, INPUT);
  digitalWrite(WarningOFF, HIGH);
  digitalWrite(WarningON, LOW);

  // Måling
  pinMode(photoPin, INPUT);
  
  if(!bme.begin(0x76)) {
    Serial.println("Ingen tilkobling til BME");
    while(!bme.begin(0x76)) {
      Serial.print("... ");
      delay(1000);
    }
  }
  
  
  // Ubidots
  ubidots.connectToWifi(WIFI, WIFIpass);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(StopMOB, MOB);

  timer = millis();
}


void loop() {
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  // Måling og Varsling
  if(allsafe == false) {
    if(count == 0 and millis() - timer5 >= 1000) {
      ubidots.add(ID, 2);
      timer5 = millis();
      ubidots.publish(StopMOB);
    }
    if((millis() - timer4 >= 30000) and count < 10) {
      tempTot += (float)bme.readTemperature();
      timer4 = millis();
      count += 1;
      ubidots.add(TEMPERATURE, (float)bme.readTemperature());
      ubidots.publish(StopMOB);
    }
    if(count == 10) {
      ubidots.add(TEMPERATURE, tempTot / (float)10);
      count += 1;
    }
    if(millis() - timer >= 60000) {
      ubidots.add(LikelySurvivalTime, howLong());
      timer = millis();
      ubidots.publish(StopMOB);
    }  
  } 
  else if(millis() - timer >= 5000) {
    if(LEDcount > 10) {
      ubidots.add(ID, 1);
    } else {
      ubidots.add(ID, 0);
    }
    timer = millis();
    ubidots.publish(StopMOB);
  }
  
  if(digitalRead(WarningState) and LEDcount <= 10) {
    LEDcount += 1;
  }
      
  ubidots.loop();

  // Erstatning for enkel simulering istedenfor trykk
  if(analogRead(photoPin) <= 3000 and allsafe != false) {
    allsafe = false;
    timer4 = millis();
  }
  
  /*
  // BME unsafe trigger
  float pressure = (float)bme.readPressure() / (float)1000; 
  if(pressure >= 1020) {
    allsafe = false;
    timer4 = millis();
  }
  */

  if(blinkState) {
    blinkCrew();
  }
}
