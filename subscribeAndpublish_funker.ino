/******************************************

   This example works for both Industrial and STEM users.

   Developed by Jose Garcia, https://github.com/jotathebest/

 * ****************************************/

/****************************************
   Include Libraries
 ****************************************/
#include "UbidotsEsp32Mqtt.h"

/****************************************
   Define Constants
 ****************************************/
//const char *UBIDOTS_TOKEN = "BBFF-H2Wsoo1L7XxRmN16prd6DuPAreozkW";            // Magge sin TOKEN
const char *UBIDOTS_TOKEN = "BBFF-uKIaLP4bGwYjKTfTJLI10IhRdlwCva";            // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Daniel's Galaxy S10";                // Put here your Wi-Fi SSID
const char *WIFI_PASS = "Passord1337";                // Put here your Wi-Fi password
const char *PUBLISH_DEVICE_LABEL = "StopMOB";     // Put here your Device label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL = "ACK";   // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL_2 = "safety_button";
const char *SUBSCRIBE_DEVICE_LABEL = "StopMOB";   // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "mob"; // Replace with the variable label to subscribe to

const int PUBLISH_FREQUENCY = 5000; // Update rate in millisecondsx

const int buttonPin = 21;


unsigned long timer;
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.

bool recieved;

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
   Auxiliar Functions
 ****************************************/
void callback(char *topic, byte *payload, unsigned int length)
{

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (payload[0] == '0') {
    recieved = 0;
    ubidots.add(PUBLISH_VARIABLE_LABEL, recieved); // Insert your variable Labels and the value to be sent
    ubidots.publish(PUBLISH_VARIABLE_LABEL);
  }
  else if (payload[0] == '1') {
    recieved = 1;

    ubidots.add(PUBLISH_VARIABLE_LABEL, recieved); // Insert your variable Labels and the value to be sent
    ubidots.publish(PUBLISH_VARIABLE_LABEL);
  }
}

/****************************************
   Main Functions
 ****************************************/

void setup()
{
  pinMode(buttonPin, INPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);
  ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively

  timer = millis();
}

void loop()
{
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == 0 )
    {
      while(buttonState == 0){
        buttonState = digitalRead(buttonPin);
        }
      ubidots.add(PUBLISH_VARIABLE_LABEL_2, buttonState); // Insert your variable Labels and the value to be sent
      ubidots.publish(PUBLISH_DEVICE_LABEL);
    }

  /*
    if(buttonState){
    while(buttonState){}
    Serial.print("KNAPP!");
    }*/

  // put your main code here, to run repeatedly:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
  }
  if (millis() - timer > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    //float value = analogRead(analogPin);
    //float value = rand() %100;
    //ubidots.add(PUBLISH_VARIABLE_LABEL, recieved); // Insert your variable Labels and the value to be sent
    //ubidots.publish(PUBLISH_DEVICE_LABEL);
    timer = millis();
  }



  ubidots.loop();
}
