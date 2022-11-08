#include "internett.h"

/******************************************

   This example works for both Industrial and STEM users.

   Developed by Jose Garcia, https://github.com/jotathebest/

 * ****************************************/

/****************************************
   Include Libraries
 ****************************************/
#include "C:\Program Files (x86)\Arduino\libraries\esp32-mqtt-main\src\UbidotsEsp32Mqtt.h"

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

    if (payload[0] == '0')
    {
        recieved = 0;
        ubidots.add(PUBLISH_VARIABLE_LABEL, recieved); // Insert your variable Labels and the value to be sent
        ubidots.publish(PUBLISH_VARIABLE_LABEL);
    }
    else if (payload[0] == '1')
    {
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
    // ubidots.setDebug(true); // uncomment this to make debug messages available
}

void ubiSetup()
{
    ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
    ubidots.setCallback(callback);
    ubidots.setup();
    ubidots.reconnect();
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively

    timer = millis();
} // ubiSetup

void ubiLoop()
{
    bool buttonState = digitalRead(buttonPin);
    if (buttonState == 0)
    {
        while (buttonState == 0)
        {
            buttonState = digitalRead(buttonPin);
        }
        ubidots.add(PUBLISH_VARIABLE_LABEL_2, buttonState); // Insert your variable Labels and the value to be sent
        ubidots.publish(PUBLISH_DEVICE_LABEL);
    }

    // put your main code here, to run repeatedly:
    if (!ubidots.connected())
    {
        ubidots.reconnect();
        ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
    }
    if (millis() - timer > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
        timer = millis();
    }
    ubidots.loop();
}