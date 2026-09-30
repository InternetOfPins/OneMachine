// ONEMACHINE_STATE_PIN on the wire net (net2.h). PIN is the hash the host computes; the AVR must agree, unless the net declares a field as a plain int (-DSLOPPY).
#include "net2.h"
#ifndef PIN
  #define PIN 0x6ef5004du
#endif
ONEMACHINE_STATE_PIN(Net2, PIN);
int main() {}
