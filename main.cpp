#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 icm;

Adafruit_Sensor *accel;

float x = 0;
float y = 0;
float z = 0;

/*

#define ICM_CS 10
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11
*/

RTC_DATA_ATTR int bootCount = 0;

//const int buttonPin = 33;
bool awake = true;
bool buttonState;
bool motion = false;
int motion_counter = 0;

int awake_state = 2;
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

void setup()
{
  //pinMode(buttonPin, INPUT);
  Serial.begin(115200);

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
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);

  // Get an Adafruit_Sensor compatible object for the ICM20X's accelerometer
  accel = icm.getAccelerometerSensor();
  /*
  ++bootCount;
  Serial.println("----------------------");
  Serial.println(String(bootCount) + "th Boot ");
  */

  // Displays the wake-up source
  // print_wakeup_reason();
}

/*
bool checkSensor(float prev_x, float prev_y, float prev_z)
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

/*

void awake_func(int awake_time) // sekunder
{
  while (awake_state == AWAKE)
  {
    buttonState = digitalRead(buttonPin);

    if (millis() - prevMillis >= awake_time * 1000)
    {
      Serial.println("SOVNER!");
      prevMillis = millis();
      awake_state = DEEPSLEEP;
      break;
    }

    if (buttonState)
    {
      awake_state = DEEPSLEEP;
      break;
    }
  }
}
*/

void awake_func(int awake_time) // sekunder
{
  if (millis() - prevMillis >= awake_time * 1000)
  {
    Serial.println("SOVNER!");
    prevMillis = millis();
    awake_state = DEEPSLEEP;
  }

  if (buttonState)
  {
    awake_state = DEEPSLEEP;
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

  if (millis() - prevMillis >= awake_time * 1000)
  {
    prevMillis = millis();
    awake_state = DEEPSLEEP;
    Serial.println("legger meg");
  }
}

/*
void checkButtonPress(){

  buttonState = digitalRead(buttonPin);

  if (buttonState)
  {
    while (buttonState)
    {
      buttonState = digitalRead(buttonPin);
    }
    awake_state = AWAKE;
  }

}
*/

void loop()
{

  float prev_x = x;
  float prev_y = y;
  float prev_z = z;

  if (awake_state >= 3)
  {
    awake_state = 0;
  }

  //checkButtonPress();

  switch (awake_state)
  {
  case AWAKE:
    // Case hvor enheten er koblet på internett. Er stort sett alltid i denne casen så lenge enheten ikke sover.
    Serial.println("AWAKE");
    awake_func(20); // gitt i sekunder

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
    // Case hvor enheten er i dyp søvn, MEN vekkes med jevne mellomrom for å sjekke om bevegelsessensoren har registrert bevegelse
    Serial.println("DEEPSLEEP");
    deepsleep(30); // sekunder
    break;

  case SEMISLEEP:
    // Case hvor bevegselsessensoren kjøres en liten periode for å sjekke om personen beveger seg
    
    //checkSensor(prev_x, prev_y, prev_z);
    semisleep(15); // sekunder
    break;
  }

  // bool buttonState = digitalRead(buttonPin);
  // Serial.println(buttonState);
}
