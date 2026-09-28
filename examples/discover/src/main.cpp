// discover -- the smallest real machine: find a real MPU6050 on the I2C bus by its own identity (WHO_AM_I), print
// the table, nothing else. Nano (ATmega328P), a GY-521 (MPU6050) on A4 (SDA) / A5 (SCL), Serial 115200.
//
// Use<Own, Mpu> asks the driver's own idReg (0x75, WHO_AM_I) to read back its own id (0x68) before it is accepted
// as a row: a device that answers the address but isn't a real MPU6050 is never claimed.
#include <Arduino.h>
#include <chips/avr/avrTwi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>

using discover::RowId;
using hapi::Chain;

using Twi = hw::avr::mega::Twi<100000UL, F_CPU>;

struct App;

// The driver declares only what discovery needs: the address range, the id it answers with, and where to read
// it from. No Produces, no read(): this stage never asks the device for data.
struct Mpu : discover::DriverBase<Mpu, App> {
  static constexpr uint8_t addrLo = 0x68, addrHi = 0x69;   // AD0 low / high
  static constexpr uint8_t id = 0x68, idReg = 0x75;        // WHO_AM_I
};

using Entries = Chain<discover::Use<discover::Own, Mpu>>;

struct App : discover::World<App, Twi, Chain<>, Entries, 4, discover::I2cScan> {};

void setup() {
  Serial.begin(115200);
  Twi::begin();
  App::discover();
  Serial.print(F("rows: ")); Serial.println(App::reg.count);
  for (RowId r = 0; r < App::reg.count; ++r) {
    Serial.print(F("  row ")); Serial.print(r);
    Serial.print(App::reg.rows[r].isBus ? F(" bus 0x") : F(" dev 0x"));
    Serial.println(App::reg.rows[r].busId, HEX);
  }
  Serial.println(App::reg.count > 1 ? F("MPU6050 found") : F("MPU6050 not found -- check the wiring"));
}

void loop() {}
