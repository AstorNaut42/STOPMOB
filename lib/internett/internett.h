#pragma once

/****************************************
   Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-H2Wsoo1L7XxRmN16prd6DuPAreozkW"; // Magge sin TOKEN
// const char *UBIDOTS_TOKEN = "BBFF-uKIaLP4bGwYjKTfTJLI10IhRdlwCva";            // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Daniel's Galaxy S10"; // Put here your Wi-Fi SSID
const char *WIFI_PASS = "Passord1337";         // Put here your Wi-Fi password
const char *PUBLISH_DEVICE_LABEL = "StopMOB";  // Put here your Device label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL = "ACK";    // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL_2 = "safety_button";
const char *SUBSCRIBE_DEVICE_LABEL = "StopMOB"; // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "mob";   // Replace with the variable label to subscribe to

const int PUBLISH_FREQUENCY = 5000; // Update rate in millisecondsx

const int buttonPin = 21;
const int ledPin = 18;

unsigned long timer;
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.

bool recieved;

/****************************************
   Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length);
void ubiSetup();
void ubiLoop();
