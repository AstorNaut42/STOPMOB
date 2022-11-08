#include "sleep.h"

void awake_func(int button_pin)
{
    awake = true;
    Serial.println("ups");
    while (awake)
    {
        bool buttonState = digitalRead(button_pin);

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

// Funksjon som kjøres under semisleep modus.
void semisleep(int awake_time, unsigned long prevMillis)
{

    if (millis() - prevMillis >= awake_time * 1000)
    {
        prevMillis = millis();

        // checkSensor();
    }
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