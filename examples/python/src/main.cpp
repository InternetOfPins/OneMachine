// python -- a Nano whose outputs are driven by name from Python (drive.py) over the USB serial port. See README.md.
#include <Arduino.h>
#include "machine.h"
#include <oneMachine/role/link.h>

// the Nano's endpoints: pin 13 (the onboard LED) and PWM pin 9
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

using M = MachineOf<Pin<13>, Pwm<9>>;
struct Out { static void put(uint8_t b) { Serial.write(b); } };
static M::Command cmd{};
static M::Report rep{};
static role::Link<M, Out> link(rep, 2000);          // no accepted command for 2 s: every role to its safe command

void setup() {
  pinMode(13, OUTPUT); pinMode(9, OUTPUT);
  Serial.begin(115200);
  M::apply(cmd, rep);
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) link.feed(uint8_t(Serial.read()), now);
  // the cycle boundary: a new command, a quiet supervisor, then the report
  if (link.take(cmd)) M::apply(cmd, rep);
  if (link.quiet(now)) { M::safe(cmd, rep); M::apply(cmd, rep); }
  M::sense(rep);
}
