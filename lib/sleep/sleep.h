#pragma once

#include <Arduino.h>

void awake_func(int button_pin);
void deepsleep();
void semisleep(int awake_time);
void print_wakeup_reason();

bool awake = true;
int awake_state = 0;

#define AWAKE 0
#define DEEPSLEEP 1
#define SEMISLEEP 2
