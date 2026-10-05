// SPI discovery on the real AVR SPI core: the RC522 driver and SpiScan built with avr-g++ 7.3 -Os (the oldest
// compiler the library supports), so the size of an SPI world is on record and gcc 7 keeps compiling it.
//   (default)    the driver with no interrupt part: its image is the driver as it was before the interrupt part existed (build_spi1.sh checks the size)
//   -DWITH_IRQ=1 with rc522::Interrupt (the IRQ on PD2, sampled), service() folded over the rows
//   -DWITH_IRQ=2 and the LineCheck and PollOnLineFault parts
//   -DMACHINE    the driver configured by the RC522 machine (examples/spi/src/rc522_machine.h), with no interrupt part
#include <stdint.h>
#include <avr/io.h>
#include <hapi/hapi.h>
#include <chips/avr/avrSpi.h>
#include <oneMachine/discover/spi.h>
#include "../../examples/spi/src/rc522.h"
#ifdef MACHINE
#include "../../examples/spi/src/rc522_machine.h"
#endif
#ifdef WITH_IRQ
#include <oneMachine/fail/world.h>
#endif

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

#ifdef WITH_IRQ
struct IrqLine {   // PD2, pulled up, active low
  static void begin() { DDRD &= uint8_t(~(1u << 2)); PORTD |= uint8_t(1u << 2); }
  static void arm() {}
  static bool ready() { return !(PIND & (1u << 2)); }
};
struct IrqMode : rc522::NoFail {
  #if WITH_IRQ == 2
  using Irq = rc522::Interrupt<IrqLine, rc522::NoObserver, rc522::LineCheck, rc522::PollOnLineFault>;
  #else
  using Irq = rc522::Interrupt<IrqLine>;
  #endif
};
struct App;
using Rfid = rc522::Rc522<App, IrqMode>;
#elif defined(MACHINE)
struct App;
using Rfid = rc522m::Machine<App, rc522m::Slot<0>>::Driver;
#else
struct App;
using Rfid = rc522::Rc522<App>;
#endif
struct App : discover::World<App, Bus, hapi::Chain<Sink>, hapi::Chain<Rfid>, 3, discover::SpiScan,
                             hapi::Chain<discover::SpiSlotIds<2>>> {};

int main() {
  Bus::begin();
#ifdef WITH_IRQ
  Rfid::Irq::begin();
#endif
  App::discover();
#ifdef WITH_IRQ
  for (uint32_t now = 0;; ++now) { App::pump(); fail::Services<App, App::DriverList>::run(now); }
#else
  for (;;) App::pump();
#endif
}
