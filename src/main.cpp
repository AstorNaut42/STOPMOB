#include <Arduino.h>
#include "../StopMOB.h"

void setup()
{
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);

  ++bootCount;
  Serial.println("----------------------");
  Serial.println(String(bootCount) + "th Boot ");

  // Displays the wake-up source
  print_wakeup_reason();

  // Configure GPIO33 as a wake-up source when the voltage is 3.3V
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, HIGH);

  // Rentre en mode Deep Sleep
  Serial.println("Goes into Deep Sleep mode");
  Serial.println("----------------------");
  // esp_light_sleep_start();
}

void awake_func()
{
  awake = true;
  Serial.println("ups");
  while (awake)
  {
    bool buttonState = digitalRead(buttonPin);

    Serial.println("VÅKEN!");

    if (buttonState)
    {
      awake = false;
      break;
    }
  }
}

bool checkSensor()
{
  bool motion = false;

  return motion;
}

void loop()
{
  awake_func();
  Serial.println("light_sleep_enter");
  // esp_sleep_enable_timer_wakeup(2000000); //1 seconds
  esp_light_sleep_start();

  print_wakeup_reason();

  switch (awake_state)
  {
  case AWAKE:
    // Case hvor enheten er koblet på internett. Er stort sett alltid i denne casen så lenge enheten ikke sover.
    break;

  case DEEPSLEEP:
    // Case hvor enheten er i dyp søvn, MEN vekkes med jevne mellomrom for å sjekke om bevegelsessensoren har registrert bevegelse
    deepsleep();
    break;

  case SEMISLEEP:
    // Case hvor bevegselsessensoren kjøres en liten periode for å sjekke om personen beveger seg
    semisleep(10);

    break;
  }

  // bool buttonState = digitalRead(buttonPin);
  // Serial.println(buttonState);
}
