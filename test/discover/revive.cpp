// A row that went Gone is looked at again, at its declared address, by the same entries: a part that answers takes its row again (the slot is
// reused, the driver's init runs), a part that does not stays Gone. An app without `revive` is as before: Gone is final until discover().
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "../support/mockTwi.h"
#include "../support/sensors.h"

using discover::RowId;
using discover::Status;
using hapi::Chain;

struct Seen {
  using Accepts = Chain<Temperature>;
  inline static uint8_t n = 0;
  template<typename Cap> struct Body { template<typename T> struct Part : T { using T::T; void on(const discover::Sample<Cap>&) { ++n; } }; };
};

template<bool Revive> struct App;
template<bool Revive> using Drivers = Chain<SensorB<App<Revive>>>;
template<bool Revive> struct App : discover::World<App<Revive>, mock::Twi, Chain<Seen>, Drivers<Revive>, 4, discover::I2cScan> {
  static constexpr bool lifecycle = true, revive = Revive;
  static void release(RowId) {}
  static void unbindAll() {}
};

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)



int main() {
  using A = App<true>;
  auto& dev = mock::Bus::devs[1];                      // the SensorB at 0x40 on the root bus
  A::discover();
  CHECK(A::reg.count == 2 && A::reg.status(1) == Status::Alive && A::reg.rows[1].busId == 0x40);
  const uint8_t row0 = A::reg.count;

  dev.addr = 0x41;                                      // unplugged: nothing answers at 0x40
  A::setStatus(1, Status::Gone);
  CHECK(A::reg.status(1) == Status::Gone);

  A::reviveGone();                                      // still unplugged: it stays Gone, no row is added
  CHECK(A::reg.status(1) == Status::Gone && A::reg.count == row0);
  A::reviveGone(); A::reviveGone();
  CHECK(A::reg.status(1) == Status::Gone && A::reg.count == row0);

  dev.addr = 0x40;                                      // replugged
  A::reviveGone();
  CHECK(A::reg.status(1) == Status::Alive && A::reg.count == row0 && A::reg.rows[1].busId == 0x40 && A::reg.overflow == 0);
  Seen::n = 0; A::pump();
  CHECK(Seen::n == 1);                                  // polled again: its Temperature reaches the consumer

  // and again: gone, back, gone, back, always the same row
  for (int i = 0; i < 3; ++i) {
    dev.addr = 0x41; A::setStatus(1, Status::Gone); A::reviveGone();
    CHECK(A::reg.status(1) == Status::Gone && A::reg.count == row0);
    dev.addr = 0x40; A::reviveGone();
    CHECK(A::reg.status(1) == Status::Alive && A::reg.count == row0);
  }

  // another part at the address: the entries decide, as at discovery (SensorB's id is 0xB2)
  dev.addr = 0x41; A::setStatus(1, Status::Gone);
  const uint8_t was = dev.regs[0]; dev.regs[0] = 0x55; dev.addr = 0x40;
  A::reviveGone();
  CHECK(A::reg.status(1) == Status::Gone);
  dev.regs[0] = was;

  // without `revive` the status stays final: the registry is as it was
  using B = App<false>;
  B::discover();
  dev.addr = 0x41; B::setStatus(1, Status::Gone); dev.addr = 0x40;
  CHECK(B::reg.status(1) == Status::Gone);
  B::discover();
  CHECK(B::reg.status(1) == Status::Alive);

  if (failures) { std::printf("revive: %d failed\n", failures); return 1; }
  std::printf("OK: revive, a Gone row comes back at its declared address\n");
  return 0;
}
