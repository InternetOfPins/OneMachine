// discoverCompose Round 2: one discovered display row served at three binding strengths, plus sensor samples
// through a generic client (Format -> Transport).
//   native: both display bindings reach the same row with identical output; a row that is not Alive is not called;
//           JSON/CSV payloads byte-exact; MQTT QoS 0 packets captured (and sent to a broker with --broker host:port).
//   AVR:    build.sh links this without MQTT for atmega328p and counts indirect calls and bytes per binding.
#include <stdint.h>
#include <hapi/hapi.h>
#define DISCOVER_TEST_RAW_STATUS   // NEG_SHARED_CLIENT scenario reaches the registry directly
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/binding.h>
#include <oneMachine/discover/format.h>
#include "../support/mockTwi.h"
#include "../support/mockDisplay.h"
#include "../support/sensors.h"
#include "../support/display.h"
#if !defined(__AVR__) && !defined(R2_NO_MQTT)
  #include "mqttSink.h"
  #define R2_WITH_MQTT 1
#endif

using discover::RowId;
using discover::Sample;
using hapi::Chain;

#ifdef R2_NO_DIRECT
  static constexpr bool kDirect = false;
#else
  static constexpr bool kDirect = true;
#endif
#ifdef R2_NO_CLASS
  static constexpr bool kClass = false;
#else
  static constexpr bool kClass = true;
#endif
#ifdef R2_NO_JSON
  static constexpr bool kJson = false;
#else
  static constexpr bool kJson = true;
#endif
#ifdef R2_NO_CSV
  static constexpr bool kCsv = false;
#else
  static constexpr bool kCsv = true;
#endif

// ---- direct connection: needs the concrete display ---------------------------------------------------
#ifdef NEG_DIRECT_ABSENT
template<typename W> struct Ghost : discover::DriverBase<Ghost<W>, W> { static constexpr uint8_t addrLo = 0x30, addrHi = 0x30, id = 0x99; };
#endif

template<typename W>
struct Banner {
  using Accepts = Chain<Humidity>;
#ifdef NEG_DIRECT_ABSENT
  using Iface = Ghost<W>;
#else
  using Iface = TextDisplay<W>;
#endif
  inline static discover::Shell<W, Iface> lcd;

  template<typename Impl> static constexpr bool wants = kDirect && std::is_same<Impl, Iface>::value;
  template<typename D>    static constexpr bool served = !kDirect || hapi::Exists<hapi::SameAs<Iface>, D>::value;
  template<typename I> static void bind(RowId row, I* d) { lcd.bind(row, d); }
  static void unbind() { lcd.unbind(); }

  // clear, then print: the same two operations, and the same screen, as Readout::show
  static bool show(const char* s) {
    Iface* d = lcd.get();
    if (!d) return false;
    d->clear(lcd.row);
    d->print(lcd.row, s);
    return true;
  }

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) {
        if constexpr (kDirect) {
          if (Iface* d = lcd.get()) {
            char b[12];
            discover::fmt::putLabel(b, 'H', s.row, s.value, Cap::decimals);
            d->setCursor(lcd.row, 0, 1);      // beyond the class: a concrete operation
            d->print(lcd.row, b);
          }
        } else (void)s;
      }
    };
  };
};

// ---- capability-set output class: needs some output that can print and clear -------------------------------
#ifdef NEG_CLASS_NO_PROVIDER
struct Blink {};
using Out = discover::OutClass<Chain<discover::Print, discover::Clear, Blink>, Chain<discover::OpPrint, discover::OpClear>>;
#else
using Out = discover::TextOut;
#endif

template<typename W, typename Drivers>
struct Readout {
  using Accepts = Chain<Temperature>;
  using Handle  = discover::OutHandleT<W, Out, Drivers>;
  inline static Handle out;

  template<typename Impl> static constexpr bool wants = kClass && discover::Provides<Impl, typename Out::Needs>::value;
  template<typename D>    static constexpr bool served = true;
  template<typename I> static void bind(RowId row, I* d) { out.bind(row, d, false); }   // first provider row
  static void unbind() { out.unbind(); }

  static bool show(const char* s) { return out.clear() && out.print(s); }

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) {
        if constexpr (kClass) {
          char b[12];
          discover::fmt::putLabel(b, 'T', s.row, s.value, Cap::decimals);
          out.clear();
          out.print(b);
        } else (void)s;
      }
    };
  };
};

// the same class again, bound to the last provider row: a different driver type behind the same handle type
template<typename W, typename Drivers>
struct Mirror {
  using Accepts = Chain<Humidity>;
  using Handle  = discover::OutHandleT<W, Out, Drivers>;
  inline static Handle out;

  template<typename Impl> static constexpr bool wants = kClass && discover::Provides<Impl, typename Out::Needs>::value;
  template<typename D>    static constexpr bool served = true;
  template<typename I> static void bind(RowId row, I* d) { out.bind(row, d, true); }
  static void unbind() { out.unbind(); }

  static bool show(const char* s) { return out.clear() && out.print(s); }

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) {
        if constexpr (kClass) {
          char b[12];
          discover::fmt::putLabel(b, 'H', s.row, s.value, Cap::decimals);
          out.clear();
          out.print(b);
        } else (void)s;
      }
    };
  };
};

// ---- generic clients: Format x Transport ---------------------------------------------------------------
using JsonMem = discover::GenericClient<discover::fmt::Json, discover::MemSink, kJson, Temperature, Humidity>;
using CsvMem  = discover::GenericClient<discover::fmt::Csv,  discover::MemSink, kCsv,  Temperature, Humidity>;
#ifdef R2_WITH_MQTT
using JsonMqtt = discover::GenericClient<discover::fmt::Json, discover::Mqtt0, kJson, Temperature, Humidity>;
using CsvMqtt  = discover::GenericClient<discover::fmt::Csv,  discover::Mqtt0, kCsv,  Temperature, Humidity>;
#endif

// ---- application ---------------------------------------------------------------------------------------
template<uint8_t N> struct AppN;
template<uint8_t N> using DriversOf = Chain<SensorA<AppN<N>>, SensorB<AppN<N>>, Mux<AppN<N>>, TextDisplay<AppN<N>>, LineDisplay<AppN<N>>>;
template<uint8_t N> using BindersOf = Chain<Banner<AppN<N>>, Readout<AppN<N>, DriversOf<N>>, Mirror<AppN<N>, DriversOf<N>>>;
#ifdef R2_WITH_MQTT
template<uint8_t N> using ConsumersOf = Chain<JsonMem, CsvMem, JsonMqtt, CsvMqtt, Banner<AppN<N>>, Readout<AppN<N>, DriversOf<N>>, Mirror<AppN<N>, DriversOf<N>>>;
#else
template<uint8_t N> using ConsumersOf = Chain<JsonMem, CsvMem, Banner<AppN<N>>, Readout<AppN<N>, DriversOf<N>>, Mirror<AppN<N>, DriversOf<N>>>;
#endif

template<uint8_t N>
struct AppN : discover::World<AppN<N>, mockdisp::Twi, ConsumersOf<N>, DriversOf<N>, N, discover::I2cScan> {
  using Base = discover::World<AppN<N>, mockdisp::Twi, ConsumersOf<N>, DriversOf<N>, N, discover::I2cScan>;

  // called from DriverBase::found: the concrete driver type and the new row are known
  template<typename Impl> static void bind(RowId row) { discover::BinderSet<BindersOf<N>>::template bind<Impl>(row); }

  static void discoverAll() { discover::BinderSet<BindersOf<N>>::unbind(); Base::discover(); }
};

using App = AppN<10>;
using Rd  = Readout<App, DriversOf<10>>;
using Mr  = Mirror<App, DriversOf<10>>;
static_assert(discover::BinderSet<BindersOf<10>>::served<DriversOf<10>>, "a binding consumer names a driver that is not in the driver list");
// no per-row erasure was added: the binding types are plain data, and the fan-outs stay stateless
using FanT = discover::CapFanoutT<Temperature, ConsumersOf<10>>;
using FanH = discover::CapFanoutT<Humidity, ConsumersOf<10>>;
static_assert(!__is_polymorphic(FanT) && sizeof(FanT) == 1 && !__is_polymorphic(FanH) && sizeof(FanH) == 1, "fan-outs stay stateless");
static_assert(!__is_polymorphic(Rd::Handle) && !__is_polymorphic(decltype(Banner<App>::lcd)), "handle and shell are not polymorphic");
static_assert(discover::ProviderSet<hapi::Eval<hapi::Filter<discover::ProvidesAllOf<discover::TextOut::Needs>::Type>, DriversOf<10>>>::size == 2,
              "TextDisplay and LineDisplay both provide TextOut");
#ifdef __AVR__
static_assert(sizeof(App::reg.rows[0]) == 5, "row = busId + ptr + parent + flags: R1's layout");
#endif

// ---- table + display + payload bytes folded into 16 bits, identical arithmetic on native and AVR ------------
static uint16_t checksum() {
  uint16_t h = 0;
  for (RowId r = 0; r < App::reg.count; ++r) {
    const auto& row = App::reg.rows[r];
    h = uint16_t(h * 31u + row.busId + row.parent * 7u + row.isBus);
  }
  for (auto& r : mockdisp::Screen::cell) for (char c : r) h = uint16_t(h * 31u + uint8_t(c));
  for (uint8_t i = 0; i < mockdisp::Screen::logN; ++i) h = uint16_t(h * 31u + mockdisp::Screen::log[i]);
  for (auto& r : mockdisp::Screen2::cell) for (char c : r) h = uint16_t(h * 31u + uint8_t(c));
  for (uint8_t i = 0; i < mockdisp::Screen2::logN; ++i) h = uint16_t(h * 31u + mockdisp::Screen2::log[i]);
  for (uint16_t i = 0; i < discover::MemSink::n; ++i) h = uint16_t(h * 31u + discover::MemSink::buf[i]);
  return h;
}

// ---- AVR entry -----------------------------------------------------------------------------------------
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() {
  mockdisp::Screen::reset(); mockdisp::Screen2::reset();
  mock::Bus::mask = 2;
  App::discoverAll();
  App::pump();
  g_sum = checksum();
  done();
}
#elif defined(R2_PARITY)
#include <cstdio>
int main() {
  mockdisp::Screen::reset(); mockdisp::Screen2::reset();
  mock::Bus::mask = 2;
  App::discoverAll();
  App::pump();
  std::printf("checksum 0x%04X\n", checksum());
  return 0;
}
#else

// ---- native test -----------------------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <time.h>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

using discover::Status;
using discover::instOf;
using S1 = mockdisp::Screen;
using S2 = mockdisp::Screen2;

struct Msg { std::string topic, payload; };

static std::vector<Msg> sinkMsgs() {
  std::vector<Msg> v;
  for (unsigned i = 0; i < discover::MemSink::n;) {
    const uint8_t tl = discover::MemSink::buf[i], pl = discover::MemSink::buf[i + 1];
    v.push_back({std::string(reinterpret_cast<const char*>(discover::MemSink::buf + i + 2), tl),
                 std::string(reinterpret_cast<const char*>(discover::MemSink::buf + i + 2 + tl), pl)});
    i += 2u + tl + pl;
  }
  return v;
}
static std::vector<std::string> payloads(const std::vector<Msg>& m, const std::string& topicPrefix, const char* cap) {
  std::vector<std::string> v;
  for (auto& x : m) if (x.topic == topicPrefix + cap) v.push_back(x.payload);
  return v;
}

static const char* drvName(const discover::IDriver* d) {
  if (!d) return "-";
  if (d == instOf<SensorA<App>>())     return "SensorA";
  if (d == instOf<SensorB<App>>())     return "SensorB";
  if (d == instOf<Mux<App>>())         return "Mux";
  if (d == instOf<TextDisplay<App>>()) return "TextDisplay";
  if (d == instOf<LineDisplay<App>>()) return "LineDisplay";
  return "?";
}

template<typename S> static std::string lineOf(uint8_t line) { return std::string(S::cell[line], S::cols); }
template<typename S> static std::string logOf() { return std::string(reinterpret_cast<const char*>(S::log), S::logN); }
static void resetScreens() { S1::reset(); S2::reset(); }

// the command log a display receives for: clear, print(text) [per step]
struct Log {
  std::string s;
  Log& C() { s += 'C'; return *this; }
  Log& W(const char* t) { for (; *t; ++t) { s += 'W'; s += *t; } return *this; }
  Log& S(char x, char y) { s += 'S'; s += x; s += y; return *this; }
};

static std::string refFixed(int v, int dec) {
  char b[32];
  const unsigned a = v < 0 ? unsigned(-v) : unsigned(v);
  unsigned p = 1; for (int i = 0; i < dec; ++i) p *= 10;
  if (dec == 0) std::snprintf(b, sizeof b, "%u", a);
  else          std::snprintf(b, sizeof b, "%u.%0*u", a / p, dec, a % p);
  return std::string(v < 0 ? "-" : "") + b;
}

#ifdef R2_WITH_MQTT
static std::string hex(const uint8_t* p, size_t n) {
  static const char* d = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < n; ++i) { s += d[p[i] >> 4]; s += d[p[i] & 15]; }
  return s;
}

static void msleep(int ms) { timespec t{ms / 1000, (ms % 1000) * 1000000L}; nanosleep(&t, nullptr); }
#endif

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
#ifdef R2_WITH_MQTT
  const char* broker = nullptr;
  for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--broker") && i + 1 < argc) broker = argv[++i];
#endif

  resetScreens();
  mock::Bus::mask = 2;                     // stale bridge selection at power-up, as in R1
  App::discoverAll();
  auto& reg = App::reg;

  // ---- the table: R1's topology plus two displays (rows 4, 5) on the root bus ------------------------------------
  struct Expect { uint8_t busId; RowId parent; bool isBus; const char* drv; };
  const Expect want[] = {
    {0x00, discover::noRow, true, "-"},            // 0 root bus
    {0x70, 0, false, "Mux"},                       // 1 bridge on root
    {0x00, 1, true,  "Mux"},                       // 2 bridge ch0
    {0x01, 1, true,  "Mux"},                       // 3 bridge ch1
    {0x27, 0, false, "TextDisplay"},               // 4 display with a cursor
    {0x3F, 0, false, "LineDisplay"},               // 5 a different driver type, print + clear only
    {0x40, 0, false, "SensorB"},                   // 6
    {0x48, 2, false, "SensorA"},                   // 7 behind ch0
    {0x48, 3, false, "SensorA"},                   // 8 behind ch1
  };
  CHECK(reg.count == 9 && reg.overflow == 0);
  for (RowId r = 0; r < reg.count && r < 9; ++r) {
    CHECK(reg.rows[r].busId == want[r].busId && reg.rows[r].parent == want[r].parent);
    CHECK(bool(reg.rows[r].isBus) == want[r].isBus);
    CHECK(std::strcmp(drvName(reg.rows[r].drv), want[r].drv) == 0);
    std::printf("ROW %u %u %d %s %s\n", r, unsigned(reg.rows[r].busId),
                reg.rows[r].parent == discover::noRow ? -1 : int(reg.rows[r].parent),
                reg.rows[r].isBus ? "bus" : "dev", drvName(reg.rows[r].drv));
  }
  CHECK(mock::Bus::contention == 0);

  // ---- bindings resolved at discovery time, from the same table --------------------------------------------------
  CHECK(Banner<App>::lcd.row == 4 && Banner<App>::lcd.p == &discover::Inst<TextDisplay<App>>::obj);   // direct: the one concrete type
  CHECK(Rd::out.row == 4 && Rd::out.drv == instOf<TextDisplay<App>>());                                // class: first provider row
  CHECK(Mr::out.row == 5 && Mr::out.drv == instOf<LineDisplay<App>>());                               // class: last provider row, other driver type
  CHECK(Banner<App>::lcd.get() != nullptr);

  // ---- identical output: the same text through the direct shell, the class handle, and the class handle on the other driver type
  resetScreens();
  CHECK(Banner<App>::show("HELLO"));
  const std::string logDirect = logOf<S1>(), lineDirect = lineOf<S1>(0);
  resetScreens();
  CHECK(Rd::show("HELLO"));
  const std::string logClass = logOf<S1>(), lineClass = lineOf<S1>(0);
  resetScreens();
  CHECK(Mr::show("HELLO"));
  const std::string logOther = logOf<S2>(), lineOther = lineOf<S2>(0);
  CHECK(lineDirect == "HELLO           " && lineClass == lineDirect && lineOther == lineDirect);
  CHECK(logDirect == Log().C().W("HELLO").s && logClass == logDirect && logOther == logDirect);      // same commands, same order
  CHECK(logOf<S1>().empty());                                                                       // the other display was not touched

  // ---- the data path: samples reach each consumer kind ---------------------------------------------------------------
  resetScreens(); discover::MemSink::reset();
  App::pump();
  CHECK(logOf<S1>() == Log().C().W("T6=18.7").S(0, 1).W("H6=64").C().W("T7=21.5").C().W("T8=25.3").s);   // Readout (class) + Banner (direct)
  CHECK(logOf<S2>() == Log().C().W("H6=64").s);                                                       // Mirror (class) on the other driver
  CHECK(S1::logLost == 0 && S2::logLost == 0);
  CHECK(lineOf<S1>(0) == "T8=25.3         " && lineOf<S2>(0) == "H6=64           ");
  CHECK(mock::Bus::contention == 0);

  // ---- generic client: JSON and CSV over the same sink, byte-exact, distinct rows for the identical sensors ------
  const auto msgs = sinkMsgs();
  const std::vector<std::string> jsT = {"{\"row\":6,\"value\":18.7}", "{\"row\":7,\"value\":21.5}", "{\"row\":8,\"value\":25.3}"};
  const std::vector<std::string> jsH = {"{\"row\":6,\"value\":64}"};
  const std::vector<std::string> csT = {"6,18.7", "7,21.5", "8,25.3"};
  const std::vector<std::string> csH = {"6,64"};
  CHECK(msgs.size() == 8 && discover::MemSink::dropped == 0);
  CHECK(payloads(msgs, "iop/json/", "temperature") == jsT && payloads(msgs, "iop/json/", "humidity") == jsH);
  CHECK(payloads(msgs, "iop/csv/", "temperature")  == csT && payloads(msgs, "iop/csv/", "humidity")  == csH);
  for (auto& m : msgs) std::printf("SINK %s %s\n", m.topic.c_str(), m.payload.c_str());

  // ---- a row that is not Alive is not called through, by either binding; the other rows and the generic client go on --
  for (Status st : {Status::Stale, Status::Gone}) {
    discover::RawStatus::set(reg, 4, st);                          // the first display: an app without lifecycle, so not World::setStatus
    resetScreens(); discover::MemSink::reset();
    CHECK(Banner<App>::lcd.get() == nullptr);
    CHECK(!Banner<App>::show("X") && !Rd::show("X"));
    CHECK(Mr::show("X") && lineOf<S2>(0) == "X               ");     // the display on row 5 is unaffected
    resetScreens();
    App::pump();
    CHECK(S1::logN == 0 && lineOf<S1>(0) == "                ");
    CHECK(logOf<S2>() == Log().C().W("H6=64").s);
    CHECK(discover::MemSink::msgs == 8);
    discover::RawStatus::set(reg, 4, Status::Alive);

    discover::RawStatus::set(reg, 5, st);                          // the second display
    resetScreens(); discover::MemSink::reset();
    CHECK(!Mr::show("X") && Rd::show("X") && Banner<App>::show("Y"));
    resetScreens();
    App::pump();
    CHECK(S2::logN == 0);
    CHECK(logOf<S1>() == Log().C().W("T6=18.7").S(0, 1).W("H6=64").C().W("T7=21.5").C().W("T8=25.3").s);
    CHECK(discover::MemSink::msgs == 8);
    discover::RawStatus::set(reg, 5, Status::Alive);
  }
  resetScreens();
  CHECK(Banner<App>::show("BACK") && lineOf<S1>(0) == "BACK            ");   // Alive again: called through

  // ---- a rediscovery without the first display: the old bindings are stale, and the row indices hold other drivers ---
  S1::present = false;
  App::discover();                                                    // not discoverAll: bindings left as they were
  CHECK(reg.count == 8 && std::strcmp(drvName(reg.rows[4].drv), "LineDisplay") == 0 && reg.status(4) == Status::Alive);
  resetScreens();
  CHECK(Banner<App>::lcd.get() == nullptr);                            // Alive row, but no longer the driver it was bound to
  CHECK(!Banner<App>::show("Z") && !Rd::show("Z"));                    // stale: Alive rows, but no longer the drivers they were bound to
  CHECK(S1::logN == 0 && S2::logN == 0);
  CHECK(Mr::out.row == 4);                                             // bind() runs on every found(): the last-provider policy followed to the only provider left
  App::discoverAll();                                                 // unbinds first, then binds what is there
  CHECK(Banner<App>::lcd.row == discover::noRow);                     // the direct connection has nothing of its type
  CHECK(Rd::out.row == 4 && Mr::out.row == 4);                        // the class binds any provider: the only one left
  CHECK(Rd::show("Y") && lineOf<S2>(0) == "Y               ");
  S1::present = true;
  App::discoverAll();                                                 // and both come back
  CHECK(reg.count == 9 && Banner<App>::lcd.row == 4 && Rd::out.row == 4 && Mr::out.row == 5);

  // ---- the number formatter against printf with division, exhaustively ------------------------------------------
  {
    char b[16]; unsigned bad = 0;
    for (int dec = 0; dec <= 2; ++dec)
      for (int v = -32768; v <= 65535; ++v) {
        const uint8_t n = discover::fmt::putFixed(b, v, uint8_t(dec));
        if (std::string(b, n) != refFixed(v, dec)) { if (!bad++) std::printf("first mismatch v=%d dec=%d got '%s' want '%s'\n", v, dec, std::string(b, n).c_str(), refFixed(v, dec).c_str()); }
      }
    CHECK(bad == 0);
  }

#ifdef R2_WITH_MQTT
  // ---- MQTT QoS 0: the same samples through OneBus's client (packets captured; sent to a broker if one is given) ---
  {
    discover::MqttClient cli;
    discover::Mqtt0::client = &cli;
    bool live = false;
    if (broker) {
      std::string hp = broker; const size_t c = hp.find(':');
      live = cli.transport.tcp.connectTo(hp.substr(0, c).c_str(), uint16_t(std::atoi(hp.c_str() + c + 1)));
      cli.transport.live = live;
    }
    oneBus::mqtt::ConnectOpts co; co.clientId = "iopR2"; co.keepalive = 60;
    CHECK(cli.connect(co));
    if (live) {
      for (int t = 0; t < 400 && cli.state != decltype(cli)::State::Connected; ++t) { cli.poll(); msleep(5); }
      live = cli.state == decltype(cli)::State::Connected;
    }
    std::printf("LIVE %s\n", live ? "connected" : "none");

    discover::MemSink::reset();
    App::pump();
    CHECK(cli.disconnect());
    if (live) msleep(200);                                         // let the broker fan the messages out

    // walk the bytes the client wrote: CONNECT, 8 x PUBLISH, DISCONNECT
    const auto& w = cli.transport.written;
    size_t off = 0; unsigned nConnect = 0, nPublish = 0, nDisc = 0;
    std::vector<Msg> published;
    while (off < w.size()) {
      oneBus::mqtt::FixedHeader fh{};
      if (!oneBus::mqtt::parseFixedHeader(w.data() + off, w.size() - off, fh)) { CHECK(false); break; }
      const size_t total = fh.headerLen + fh.remaining;
      if (off + total > w.size()) { CHECK(false); break; }
      if (fh.type == oneBus::mqtt::Type::CONNECT) ++nConnect;
      else if (fh.type == oneBus::mqtt::Type::DISCONNECT) ++nDisc;
      else if (fh.type == oneBus::mqtt::Type::PUBLISH) {
        oneBus::mqtt::PubIn m;
        CHECK(oneBus::mqtt::decodePublish(w.data() + off, total, m));
        CHECK(m.qos == 0 && !m.retain && !m.dup);
        published.push_back({std::string(m.topic, m.topicLen), std::string(reinterpret_cast<const char*>(m.payload), m.payloadLen)});
        std::printf("PKT %s\t%s\t%s\n", published.back().topic.c_str(), hex(m.payload, m.payloadLen).c_str(), hex(w.data() + off, total).c_str());
        ++nPublish;
      }
      off += total;
    }
    CHECK(nConnect == 1 && nPublish == 8 && nDisc == 1);
    // what went over the transport is what the memory sink got: same topics, same payloads, same order per topic
    const auto memMsgs = sinkMsgs();
    for (const char* pre : {"iop/json/", "iop/csv/"})
      for (const char* cap : {"temperature", "humidity"})
        CHECK(payloads(published, pre, cap) == payloads(memMsgs, pre, cap));
    CHECK(payloads(published, "iop/json/", "temperature") == jsT && payloads(published, "iop/csv/", "humidity") == csH);
    discover::Mqtt0::client = nullptr;
  }
#endif

  resetScreens(); discover::MemSink::reset();
  App::discoverAll(); App::pump();
  std::printf("checksum 0x%04X\n", checksum());
  std::printf(failures ? "FAILED (%d)\n" : "OK: discoverCompose R2 native\n", failures);
  return failures != 0;
}
#endif
