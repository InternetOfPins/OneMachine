// How an interrupt line reaches the loop on an ESP8266 (the Delivery of rc522::Interrupt, or any driver's interrupt part):
//   Sampled<Pin>  the line is read where the loop asks; for a pin without an interrupt (GPIO16)
//   IsrFlag<Pin>  a falling edge sets a flag in the ISR, which does nothing else; the loop reads the flag
// ready() is true while the line is asserted (low); arm() runs before the source is enabled.
// GPIO0, GPIO2 and GPIO15 (D3, D4, D8) are sampled by the chip at reset to choose the boot mode: a source that keeps its request
// across a reset of the board and drives one of them low stops the next boot.
#pragma once
#include <Arduino.h>
#include <chips/esp8266/esp8266Pins.h>   // the pins' facts: strap(), hasIrq(), gpio()

namespace irq {

  template<uint8_t Pin> struct Esp8266Line {
    using Pins = hw::esp8266::Esp8266Pins;
    static_assert(Pins::gpio(Pin), "an ESP8266 has GPIO0 to GPIO16");
    static_assert(!Pins::strap(Pin),
                  "an interrupt line on GPIO0, GPIO2 or GPIO15 (D3, D4, D8): a request pending at reset holds the pin low and the board does not boot");
  };

  template<uint8_t Pin> struct Sampled : Esp8266Line<Pin> {
    static void begin() { pinMode(Pin, INPUT); }
    static void arm() {}
    static bool ready() { return !digitalRead(Pin); }
  };

  template<uint8_t Pin> struct IsrFlag : Esp8266Line<Pin> {
    static_assert(hw::esp8266::Esp8266Pins::hasIrq(Pin), "GPIO16 has no interrupt: use Sampled");
    static inline volatile bool flag = false;
    static void IRAM_ATTR isr() { flag = true; }
    static void begin() { pinMode(Pin, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(Pin), isr, FALLING); }
    static void arm() { flag = false; }
    static bool ready() { return flag; }
  };

}
