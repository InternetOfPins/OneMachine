// A Nano as the machine of machine.h: the onboard LED (pin 13) is the role `led`, a PWM output (pin 9) is `lamp`, A0 is the report-only `vin`, and
// `step`, `mode`, `note`, `duty` and `ping` have no wiring. The role::Link runs over the USB serial port at 115200.
// There is no timeout: a value that was set stays set.
#include <Arduino.h>
#include "machine.h"
#include <oneMachine/role/link.h>

template<uint8_t P> struct Pin {
  static bool live() { return true; }
  static void set(bool v) { digitalWrite(P, v ? HIGH : LOW); }
  static bool get() { return digitalRead(P) == HIGH; }
  template<class Put> static void where(Put& put) { put('D'); state::put_dec(put, unsigned(P)); }
};

template<uint8_t P> struct Pwm {
  static constexpr uint16_t top = 255;
  static inline uint16_t v = 0;
  static bool live() { return true; }
  static void set(uint16_t x) { v = x > top ? top : x; analogWrite(P, int(v)); }
  static uint16_t get() { return v; }
  template<class Put> static void where(Put& put) { put('D'); state::put_dec(put, unsigned(P)); put('~'); }
};

struct Adc {                                  // report only: nothing is written
  static bool live() { return true; }
  static uint16_t get() { return uint16_t(analogRead(A0)); }
  template<class Put> static void where(Put& put) { put('A'); put('0'); }
};

using M = MachineOf<Pin<13>, Pwm<9>, Adc>;
struct Out { static void put(uint8_t b) { Serial.write(b); } };
static M::Command cmd{};
static M::Report rep{};
static role::Link<M, Out> link(rep, 0);

void setup() {
  pinMode(13, OUTPUT); pinMode(9, OUTPUT);
  Serial.begin(115200);
  M::apply(cmd, rep);
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) link.feed(uint8_t(Serial.read()), now);
  if (link.take(cmd)) { M::apply(cmd, rep); state::get<Ping>(cmd).fire = 0; }   // an action fires once per command, not again when the command is applied again
  M::sense(rep);
}
