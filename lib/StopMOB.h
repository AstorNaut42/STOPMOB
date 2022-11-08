#pragma once

#include <Arduino.h>
#include "bevegelsesdeteksjon/bevegelsesdeteksjon.h"
#include "internett/internett.h"
#include "sensorinnsamling/sensorinnsamling.h"
#include "sleep/sleep.h"

const int buttonPin = 33;
unsigned long prevMillis = 0;

RTC_DATA_ATTR int bootCount = 0;