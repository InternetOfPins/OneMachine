// SPI discovery on the real AVR SPI core: the RC522 driver and SpiScan built with avr-g++ 7.3 -Os (the oldest
// compiler the library supports), so the size of an SPI world is on record and gcc 7 keeps compiling it.
#include <stdint.h>
#include <avr/io.h>
#include <hapi/hapi.h>
#include <chips/avr/avrSpi.h>
#include <oneMachine/discover/spi.h>
#include "../../examples/spi/src/rc522.h"

template<uint8_t Bit> struct PinB {   // CS pins on PORTB
  static void begin() { DDRB |= uint8_t(1u << Bit); }
  static void on()    { PORTB |= uint8_t(1u << Bit); }
  static void off()   { PORTB &= uint8_t(~(1u << Bit)); }
};
using Bus = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<PinB<1>, PinB<0>>, oneBus::SpiMaster<4000000>,
                        hw::avr::AvrSpiCore<16000000>>;

volatile uint32_t g_uid;
struct Sink {
  using Accepts = hapi::Chain<rc522::Card>;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T { void on(const discover::Sample<Cap>& s) { g_uid = s.value; } };
  };
};

struct App;
struct App : discover::World<App, Bus, hapi::Chain<Sink>, hapi::Chain<rc522::Rc522<App>>, 3, discover::SpiScan,
                             hapi::Chain<discover::SpiSlotIds<2>>> {};

int main() {
  Bus::begin();
  App::discover();
  for (;;) App::pump();
}
