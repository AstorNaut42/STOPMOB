#include <Arduino.h>
#include "../StopMOB.h"

RTC_DATA_ATTR int bootCount = 0;

const int buttonPin = 33;
bool awake = true;

int awake_state = 0;
unsigned long prevMillis = 0;

#define AWAKE 0
#define DEEPSLEEP 1
#define SEMISLEEP 2

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

void deepsleep()
{
    // Setter modus til semisleep slik at når enheten våkner igjen vil den være i tilstanden semisleep
    awake_state = SEMISLEEP;
    // Aktiverer vekking
    int vekketid = 10; // sekunder
    esp_sleep_enable_timer_wakeup(vekketid * 1000000);
    // Setter enheten i dyp søvn
    esp_deep_sleep_start();
}

bool checkSensor()
{
    bool motion = false;

    return motion;
}

void semisleep(int awake_time)
{

    if (millis() - prevMillis >= awake_time * 1000)
    {
        prevMillis = millis();

        checkSensor();
    }
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