
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "UbidotsEsp32Mqtt.h"


// Motionsensor shit
//#include <Adafruit_ICM20X.h>
//#include <Adafruit_ICM20948.h>
//Adafruit_ICM20948 icm;
//Adafruit_Sensor *accel;
float x = 0;
float y = 0;
float z = 0;

#define ICM_CS 10
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11


// Deepsleep shit
RTC_DATA_ATTR int bootCount = 0;
//const int buttonPin = 33;
bool buttonState;
bool motion = false;
int motion_counter = 0;
int awake_state = 0;
unsigned long prevMillis = 0;

#define AWAKE 0
#define DEEPSLEEP 1
#define SEMISLEEP 2


void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wake_up_source;

  wake_up_source = esp_sleep_get_wakeup_cause();

  switch (wake_up_source)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wake-up from external signal with RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wake-up from external signal with RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wake up caused by a timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wake up caused by a touchpad");
      break;
    default:
      Serial.printf("Wake up not caused by Deep Sleep: %d\n", wake_up_source);
      break;
  }
}

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

  if (healthCheck == false) {
    timer2 = millis();
  }

  if (temp <= 0.3) {
    LikelySurvivalTime_mins = 15;
  } else if (0.3 < temp and temp <= 10) {
    LikelySurvivalTime_mins = 120;
  } else if (10 < temp and temp <= 15.5) {
    LikelySurvivalTime_mins = 240;
  } else if (15.5 < temp and temp <= 21) {
    LikelySurvivalTime_mins = 240;
  } else if (21 < temp) {
    LikelySurvivalTime_mins = 1020;
  }
  healthCheck = true;
  return (LikelySurvivalTime_mins - (((millis() - timer2) / 1000) / 60));
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
  if (millis() - timer3 >= 500) {
    digitalWrite(WarningON, !digitalRead(WarningState));
    timer3 = millis();
  }
}

void setup() {
  Serial.begin(115200);

  // DEEPSLEEP SHIT
  /*
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  if (!icm.begin_I2C())
  {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20X chip");
    while (1)
    {
      delay(10);
    }
  }
  */
  ++bootCount;
  Serial.println("----------------------");
  Serial.println(String(bootCount) + "th Boot ");
  

  // Displays the wake-up source
  print_wakeup_reason();

  // MOTIONSENSOR SHIT
  //icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  // Get an Adafruit_Sensor compatible object for the ICM20X's accelerometer
  //accel = icm.getAccelerometerSensor();



  // Varsling
  pinMode(WarningON, OUTPUT);
  pinMode(WarningOFF, OUTPUT);
  pinMode(WarningState, INPUT);
  digitalWrite(WarningOFF, HIGH);
  digitalWrite(WarningON, LOW);

  // Måling
  pinMode(photoPin, INPUT);

  if (!bme.begin(0x76)) {
    Serial.println("Ingen tilkobling til BME");
    while (!bme.begin(0x76)) {
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


/*
  bool checkMotionSensor(float prev_x, float prev_y, float prev_z)
  {
  // Get a new normalized sensor event
  sensors_event_t a;
  // fill the event with the most recent data
  accel->getEvent(&a);
  if (millis() - prevMillis > 1000)
  {
    prevMillis = millis();
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;
    if (((prev_x + 0.05 > x) && (x > prev_x - 0.05)) && ((prev_y + 0.05 > y) && (y > prev_y - 0.05)) && ((prev_z + 0.05 > z) && (z > prev_z - 0.05)))
    {
      motion = false;
    }
    else
    {
      motion = true;
    }
    Serial.println(motion);
  }
  return motion;
  }
*/


void awake_func(int awake_time) // sekunder
{
  if (millis() - prevMillis >= awake_time * 1000)
  {
    Serial.println("SOVNER!");
    prevMillis = millis();
    awake_state = SEMISLEEP;
  }

}

void deepsleep(int wake_time)
{
  // Setter modus til semisleep slik at når enheten våkner igjen vil den være i tilstanden semisleep
  awake_state = SEMISLEEP;
  // Aktiverer vekking etter wake_time sekunder
  esp_sleep_enable_timer_wakeup(wake_time * 1000000);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1); // 1 = High, 0 = Low
  // Setter enheten i dyp søvn
  esp_deep_sleep_start();
}


void semisleep(int awake_time)
{
  /*checkMotionSensor(); 
  if (motion){
    awake_state = AWAKE;
    }
    */
  if (analogRead(photoPin) <= 3000) {
    awake_state = AWAKE;
    Serial.println("Våkner fordi du tok rart på meg");
  }
  if (millis() - prevMillis >= awake_time * 1000)
  {
    prevMillis = millis();
    awake_state = DEEPSLEEP;
    Serial.println("legger meg");
  }
}


void keepSafe() {
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  // Måling og Varsling
  if (allsafe == false) {
    if (count == 0 and millis() - timer5 >= 1000) {
      ubidots.add(ID, 2);
      timer5 = millis();
      ubidots.publish(StopMOB);
    }
    if ((millis() - timer4 >= 30000) and count < 10) {
      tempTot += (float)bme.readTemperature();
      timer4 = millis();
      count += 1;
      ubidots.add(TEMPERATURE, (float)bme.readTemperature());
      ubidots.publish(StopMOB);
    }
    if (count == 10) {
      ubidots.add(TEMPERATURE, tempTot / (float)10);
      count += 1;
    }
    if (millis() - timer >= 60000) {
      ubidots.add(LikelySurvivalTime, howLong());
      timer = millis();
      ubidots.publish(StopMOB);
    }
  }
  else if (millis() - timer >= 5000) {
    if (LEDcount > 10) {
      ubidots.add(ID, 1);
    } else {
      ubidots.add(ID, 0);
    }
    timer = millis();
    ubidots.publish(StopMOB);
  }

  if (digitalRead(WarningState) and LEDcount <= 10) {
    LEDcount += 1;
  }

  ubidots.loop();

  // Erstatning for enkel simulering istedenfor trykk
  if (analogRead(photoPin) <= 3000 and allsafe != false) {
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

  if (blinkState) {
    blinkCrew();
  }
}

void loop() {

  /*
    float prev_x = x;
    float prev_y = y;
    float prev_z = z;
  */

  if (awake_state >= 3)
  {
    awake_state = 0;
  }


  switch (awake_state)
  {
    case AWAKE:
      // Case hvor enheten er koblet på internett. Er stort sett alltid i denne casen så lenge enheten ikke sover.
      awake_func(60); // gitt i sekunder
      keepSafe();

      /*
        int motion_registered = checkSensor(prev_x, prev_y, prev_z);
        if (motion_registered)
        {
        motion_counter++;
        }
        else if (motion_registered == false && motion_counter > 0)
        {
        motion_counter--;
        }
        if (motion_counter == 0)
        {
        awake_state = SEMISLEEP;
        }
      */
      break;

    case DEEPSLEEP:
      if(allsafe == false) {
        awake_state = AWAKE;
        Serial.println("shit jeg kan ikke sovne nå, noe er feil");
        break;
      }
      // Case hvor enheten er i dyp søvn, MEN vekkes med jevne mellomrom for å sjekke om bevegelsessensoren har registrert bevegelse
      Serial.println("DEEPSLEEP");
      deepsleep(30); // sekunder
      break;

    case SEMISLEEP:
      if(allsafe == false) {
        awake_state = AWAKE;
        Serial.println("shit jeg kan ikke sovne nå, noe er feil");
        break;
      }
      // Case hvor bevegselsessensoren kjøres en liten periode for å sjekke om personen beveger seg

      //checkSensor(prev_x, prev_y, prev_z);
      semisleep(15); // sekunder
      break;
  }
}
