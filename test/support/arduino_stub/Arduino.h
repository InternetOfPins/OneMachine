// Just enough of Arduino.h for the delivery components' compile checks on the host (never run).
#pragma once
#include <stdint.h>
#define IRAM_ATTR
#define INPUT 0
#define INPUT_PULLUP 2
#define FALLING 2
inline void pinMode(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 1; }
inline uint8_t digitalPinToInterrupt(uint8_t p) { return p; }
inline void attachInterrupt(uint8_t, void (*)(), int) {}
