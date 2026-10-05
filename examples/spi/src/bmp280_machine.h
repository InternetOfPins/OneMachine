// A BMP280/BME280 as a static machine of OneMenu ItemDef nodes, found by its chip id at 0x76/0x77 (0x58 BMP280, 0x60 BME280).
//
//   Machine<W>::Nodes   #0 temp   read-only, 0.01 C       a value that moves outside set(): a sensor read
//                       #1 press  read-only, 0.01 hPa
//                       #2 cal    device constants: read at found(), never state, never published
//                       #3 ctrl   a group of register mimics (get() reads the chip, set() writes it); the defaults are the init
//                                 #0 config 0xF5 = 0x90 (standby 500 ms, filter x4)   #1 ctrl_meas 0xF4 = 0x57 (T x2, P x16, normal mode)
//
// The device's data are static members of Machine<W>::Dev (a static machine is its own type), so a node refers to them by address.
// A node is a singleton: an ItemRef names an object, and a template argument cannot name a member of another object.
//
// The App names what it publishes from outside: a Published<Code, Node, node, OnSync<fn>> is an outer node with the code and a publish tag that
// refers to the inner node (ItemRef) without copying it. A sync pass over the published nodes calls fn(value) for each one that changed.
// describe() walks the machine and the published nodes and writes what a consumer needs: the codes, their path, their fields, which notify.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneData/oneData.h>
#include <oneMenu/oneMenu.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/state/face.h>

namespace bmpm {

  using discover::RowId;
  using oneMenu::ItemDef;
  using oneMenu::MenuDef;
  using oneMenu::StaticBody;

  // ---- what a node declares about itself, for the walk (components with no behaviour) -----------------------------------------
  // T is a tag with ONEMACHINE_STATE_NAME(name, "text"): the text is a flash array
  template<typename T> struct Label { template<typename O> struct Part : O { using O::O; static constexpr state::Name label() { return T::name(); } }; };
  template<uint8_t N> struct Scaled { template<typename O> struct Part : O { using O::O; static constexpr uint8_t decimals = N; }; };
  template<uint8_t A, uint8_t Def> struct RegAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t regAddr = A, regDef = Def; }; };
  template<uint8_t A, uint8_t N> struct ConstAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t constAddr = A, constLen = N; }; };
  // the publish tag of an outer node: the code it is published under and the machine node it refers to
  template<typename Code, typename Node> struct PubTag { template<typename O> struct Part : O { using O::O; using PubCode = Code; using Inner = Node; }; };

  struct TagTemp  { ONEMACHINE_STATE_NAME(name, "temp"); };
  struct TagPress { ONEMACHINE_STATE_NAME(name, "press"); };
  struct TagCal   { ONEMACHINE_STATE_NAME(name, "cal"); };
  struct TagCtrl  { ONEMACHINE_STATE_NAME(name, "ctrl"); };
  struct TagConfig   { ONEMACHINE_STATE_NAME(name, "config"); };
  struct TagCtrlMeas { ONEMACHINE_STATE_NAME(name, "ctrl_meas"); };
  struct TagBmp   { ONEMACHINE_STATE_NAME(name, "bmp280"); };

  // read-only: set() is deleted, everything else of W stays (oneData::ReadOnly inherits privately, which an ItemDef cannot compose)
  template<typename W> struct ReadOnly {
    template<typename O> struct Part : W::template Part<O> {
      using Base = typename W::template Part<O>;
      using Base::Base;
      template<typename V> void set(V&&) = delete;
    };
  };

  struct Cal { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; };

  template<typename W>
  struct Machine {
    // the device: its data and its register access. Static, as the machine is.
    struct Dev {
      inline static uint8_t addr = 0;
      inline static int32_t temp = 0, press = 0;   // the last sample: 0.01 C, Pa
      inline static Cal     cal{};                  // device constants

      static void rdN(uint8_t reg, uint8_t* out, uint8_t n) {
        using Twi = typename W::Twi;
        Twi::begin_write(addr); Twi::write_byte(reg); Twi::end_write();
        (void)Twi::request_from(addr, n);
        for (uint8_t i = 0; i < n; ++i) out[i] = Twi::read_byte();
      }
      static uint8_t rd(uint8_t reg) { uint8_t v = 0; rdN(reg, &v, 1); return v; }
      static void wr(uint8_t reg, uint8_t v) { using Twi = typename W::Twi; Twi::begin_write(addr); Twi::write_byte(reg); Twi::write_byte(v); Twi::end_write(); }

      // the calibration is read until two reads agree: a corrupted read here would skew every later sample
      static bool readCal(uint8_t* c) {
        uint8_t c2[24] = {};
        rdN(0x88, c, 24); rdN(0x88, c2, 24);
        for (uint8_t i = 0; i < 24; ++i) if (c[i] != c2[i]) return false;
        const uint16_t t1 = uint16_t(c[0] | (c[1] << 8)), p1 = uint16_t(c[6] | (c[7] << 8));
        return t1 != 0 && t1 != 0xFFFF && p1 != 0 && p1 != 0xFFFF;
      }
    };

    // a register mimic: get() reads the chip, set() writes it
    template<uint8_t A> struct RegSrc { static uint8_t get() { return Dev::rd(A); } static void set(uint8_t v) { Dev::wr(A, v); } };
    template<typename Tag, uint8_t A, uint8_t Def> using Reg = ItemDef<oneData::DataFn<RegSrc<A>>, RegAt<A, Def>, Label<Tag>>;

    using Temp  = ItemDef<Scaled<2>, Label<TagTemp>,  ReadOnly<oneData::Watch<oneData::DataRef<&Dev::temp>>>>;
    using Press = ItemDef<Scaled<2>, Label<TagPress>, ReadOnly<oneData::Watch<oneData::DataRef<&Dev::press>>>>;
    using Cal_  = ItemDef<ConstAt<0x88, 24>, Label<TagCal>>;
    using Config   = Reg<TagConfig,   0xF5, 0x90>;
    using CtrlMeas = Reg<TagCtrlMeas, 0xF4, 0x57>;                       // written after config: config is written in sleep mode
    using Ctrl  = MenuDef<ItemDef<Label<TagCtrl>>, StaticBody<Config, CtrlMeas>, Label<TagCtrl>>;
    using Nodes = hapi::Chain<Temp, Press, Cal_, Ctrl>;

    inline static Temp  temp;
    inline static Press press;
    inline static Cal_  cal;
    inline static Ctrl  ctrl{ItemDef<Label<TagCtrl>>{}, StaticBody<Config, CtrlMeas>{}};

    // ---- path access: a node of the machine by position, then a register of the group ---------------------------------------
    template<typename F> static void visit(uint8_t i, F&& fn) {
      switch (i) { case 0: fn(temp); break; case 1: fn(press); break; case 2: fn(cal); break; case 3: fn(ctrl); break; default: break; }
    }
    template<typename F> static void visitReg(uint8_t i, F&& fn) { ctrl.body.visit(i, fn); }

    // ---- the registers' defaults are the init: restore from defaults writes them, in order ------------------------------------
    template<typename... R> static void writeDefaults(hapi::Chain<R...>*) { (Dev::wr(R::regAddr, R::regDef), ...); }
    static void restoreDefaults() { writeDefaults(static_cast<typename Ctrl::Body::Types*>(nullptr)); }

    static void init(uint8_t addr) {
      Dev::addr = addr;
      Dev::wr(0xE0, 0xB6);   // soft reset: a warm restart finds the part as the last firmware left it
      for (uint8_t i = 0; i < 200; ++i) {   // status bit 0 (im_update) is set while the NVM is copied in
        if (!(Dev::rd(0xF3) & 1)) break;
      }
      uint8_t c[24] = {};
      bool ok = false;
      for (uint8_t i = 0; i < 5 && !(ok = Dev::readCal(c)); ++i) {}
      if (!ok) return;   // calibration stays zero: poll() produces nothing
      auto u = [&](uint8_t i) { return uint16_t(c[i] | (uint16_t(c[i + 1]) << 8)); };
      auto s = [&](uint8_t i) { return int16_t(u(i)); };
      auto& d = Dev::cal;
      d.T1 = u(0); d.T2 = s(2); d.T3 = s(4);
      d.P1 = u(6); d.P2 = s(8); d.P3 = s(10); d.P4 = s(12); d.P5 = s(14); d.P6 = s(16); d.P7 = s(18); d.P8 = s(20); d.P9 = s(22);
      restoreDefaults();
    }

    // one burst read of the measurement registers, compensated (datasheet 8.2, integer form) into the two values
    static void poll() {
      uint8_t b[6] = {};
      Dev::rdN(0xF7, b, 6);
      const int32_t adcP = int32_t((uint32_t(b[0]) << 12) | (uint32_t(b[1]) << 4) | (b[2] >> 4));
      const int32_t adcT = int32_t((uint32_t(b[3]) << 12) | (uint32_t(b[4]) << 4) | (b[5] >> 4));
      const auto& d = Dev::cal;
      if (d.T1 == 0) return;           // no calibration
      if (adcT == 0x80000) return;     // 0x80000: no conversion yet (reset value)

      const int32_t v1 = ((((adcT >> 3) - (int32_t(d.T1) << 1))) * int32_t(d.T2)) >> 11;
      const int32_t v2 = (((((adcT >> 4) - int32_t(d.T1)) * ((adcT >> 4) - int32_t(d.T1))) >> 12) * int32_t(d.T3)) >> 14;
      const int32_t tFine = v1 + v2;
      Dev::temp = (tFine * 5 + 128) >> 8;   // 0.01 C

      int64_t p1 = int64_t(tFine) - 128000;
      int64_t p2 = p1 * p1 * int64_t(d.P6);
      p2 = p2 + ((p1 * int64_t(d.P5)) << 17);
      p2 = p2 + (int64_t(d.P4) << 35);
      p1 = ((p1 * p1 * int64_t(d.P3)) >> 8) + ((p1 * int64_t(d.P2)) << 12);
      p1 = (((int64_t(1) << 47) + p1) * int64_t(d.P1)) >> 33;
      if (p1 == 0 || adcP == 0x80000) return;   // no pressure conversion yet
      int64_t p = 1048576 - adcP;
      p = (((p << 31) - p2) * 3125) / p1;
      p1 = (int64_t(d.P9) * (p >> 13) * (p >> 13)) >> 25;
      p2 = (int64_t(d.P8) * p) >> 19;
      p = ((p + p1 + p2) >> 8) + (int64_t(d.P7) << 4);
      Dev::press = int32_t(p >> 8);        // Pa (Q24.8 >> 8)
    }

    // ---- discovery: the machine is the driver's one device ----------------------------------------------------------------
    struct Driver : discover::DriverBase<Driver, W> {
      using B = discover::DriverBase<Driver, W>;
      static constexpr bool polled = true;
      static constexpr uint8_t addrLo = 0x76, addrHi = 0x77, idReg = 0xD0, id = 0x58;
      static void init(RowId row) { Machine::init(B::addrOf(row)); }
      static void read(RowId) { Machine::poll(); }
    };
    // the BME280 answers 0x60 at the same register: a second entry, the same driver
    using Entries = hapi::Chain<discover::Use<discover::Own, Driver>, discover::Use<discover::IdProbe<0xD0, 0x60, 0x76, 0x77>, Driver>>;

    // ---- the index of a node in the machine -------------------------------------------------------------------------------
    template<typename N, typename L> struct IndexOf;
    template<typename N, typename... T> struct IndexOf<N, hapi::Chain<T...>> {
      template<unsigned I, typename... R> struct Find { static constexpr int value = -1; };
      template<unsigned I, typename H, typename... R> struct Find<I, H, R...> { static constexpr int value = std::is_same<N, H>::value ? int(I) : Find<I + 1, R...>::value; };
      static constexpr int value = Find<0, T...>::value;
    };
  };

  // ---- publishing: an outer node with a code, referring to an inner node of a machine ---------------------------------------------
  // Notify: OnSync<fn> (the value moves outside set) or none. Outer node = Notify..., the reference, the code.
  template<typename Code, typename Node, Node& ref, typename... Notify>
  using Published = ItemDef<Notify..., oneMenu::ItemRef<Node, ref>, Label<Code>, PubTag<Code, Node>>;

  // which notify components an outer node carries
  template<typename T> struct OnSyncOf : std::false_type {};
  template<auto fn> struct OnSyncOf<oneData::OnSync<fn>> : std::true_type {};
  template<typename T> struct NotifiesSync : std::false_type {};
  template<typename... OO> struct NotifiesSync<ItemDef<OO...>> : std::bool_constant<(OnSyncOf<OO>::value || ...)> {};

  // one sync pass over the published nodes: each outer node that changed calls its fn and the inner node takes its copy
  template<typename L> struct PublishAll;
  template<typename... P> struct PublishAll<hapi::Chain<P...>> { static void sync() { (P{}.sync(), ...); } };

  // ---- the description walk ---------------------------------------------------------------------------------------------
  //   machine bmp280 at 0x76
  //     #0 temp  ro value scaled 2
  //     #2 cal  const 0x88 [24]
  //     #3 ctrl  group 2
  //       #0 config  reg 0xF5 default 0x90 rw
  //   published
  //     temp  -> 1/76/0  notify sync  ro scaled 2
  //     air   -> 1/76/3  group 2
  // A path is <bus>/<identity of the device>/<node>, then a register inside a group; the identity is the device's address.
  template<typename T, typename = void> struct HasDecimals : std::false_type {};
  template<typename T> struct HasDecimals<T, std::void_t<decltype(T::decimals)>> : std::true_type {};
  template<typename T, typename = void> struct HasReg : std::false_type {};
  template<typename T> struct HasReg<T, std::void_t<decltype(T::regAddr)>> : std::true_type {};
  template<typename T, typename = void> struct HasConst : std::false_type {};
  template<typename T> struct HasConst<T, std::void_t<decltype(T::constAddr)>> : std::true_type {};
  template<typename T, typename = void> struct HasSet : std::false_type {};
  template<typename T> struct HasSet<T, std::void_t<decltype(std::declval<T&>().set(0))>> : std::true_type {};
  template<typename T, typename = void> struct IsGroup : std::false_type {};
  template<typename T> struct IsGroup<T, std::void_t<typename T::Body>> : std::true_type {};

  template<typename P> struct Walk {
    P& put;
    void str(const char* s) { while (*s) put(*s++); }
    void name(state::Name n) { state::put_name(put, n); }
    void dec(uint32_t v) { state::put_dec(put, v); }
    void hex(uint8_t v) { static const char d[] = "0123456789ABCDEF"; put('0'); put('x'); put(d[v >> 4]); put(d[v & 15]); }

    // what one node is: its fields, on the rest of the line
    template<typename N> void fields() {
      if constexpr (IsGroup<N>::value) { str(" group "); dec(N::Body::size()); }
      else if constexpr (HasReg<N>::value) { str(" reg "); hex(N::regAddr); str(" default "); hex(N::regDef); str(HasSet<N>::value ? " rw" : " ro"); }
      else if constexpr (HasConst<N>::value) { str(" const "); hex(N::constAddr); str(" ["); dec(N::constLen); put(']'); }
      else { str(HasSet<N>::value ? " rw" : " ro"); str(" value"); if constexpr (HasDecimals<N>::value) { str(" scaled "); dec(N::decimals); } }
    }

    template<typename... N> void nodes(hapi::Chain<N...>*) { unsigned i = 0; ((str("  #"), dec(i++), put(' '), name(N::label()), fields<N>(), put('\n')), ...); }
    template<typename... R> void regs(hapi::Chain<R...>*) { unsigned i = 0; ((str("    #"), dec(i++), put(' '), name(R::label()), fields<R>(), put('\n')), ...); }

    template<typename M> void machine(uint8_t addr) {
      str("machine "); name(TagBmp::name()); str(" at "); hex(addr); put('\n');
      nodes(static_cast<typename M::Nodes*>(nullptr));
      regs(static_cast<typename M::Ctrl::Body::Types*>(nullptr));
    }

    template<typename M, typename Pub> void published(uint8_t bus, uint8_t addr) {
      using Inner = typename Pub::Inner;
      str("  "); name(Pub::PubCode::name()); str(" -> "); dec(bus); put('/'); dec(addr); put('/'); dec(unsigned(M::template IndexOf<Inner, typename M::Nodes>::value));
      str(NotifiesSync<Pub>::value ? " notify sync" : " silent"); fields<Inner>(); put('\n');
    }
    template<typename M, typename... Pub> void publishedAll(hapi::Chain<Pub...>*, uint8_t bus, uint8_t addr) { (published<M, Pub>(bus, addr), ...); }
  };

  // M: the machine; Pubs: Chain<published nodes>; bus: the machine's position in the App; addr: the device's identity on its bus
  template<typename M, typename Pubs, typename P> void describe(P& put, uint8_t bus, uint8_t addr) {
    Walk<P> w{put};
    w.template machine<M>(addr);
    w.str("published\n");
    w.template publishedAll<M>(static_cast<Pubs*>(nullptr), bus, addr);
  }

}
