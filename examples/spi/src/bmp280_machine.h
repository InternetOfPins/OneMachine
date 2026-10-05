// A BMP280/BME280 as a static machine of OneMenu ItemDef nodes, found by its chip id at 0x76/0x77 (0x58 BMP280, 0x60 BME280).
//
//   Machine<W>::Nodes   #0 temp   read-only, 0.01 C       a value that moves outside set(): a sensor read
//                       #1 press  read-only, 0.01 hPa
//                       #2 cal    device constants: read at found(), never state, never published
//                       #3 ctrl   a group of register mimics (get() reads the chip, set() writes it); the defaults are the init
//                                 #0 config 0xF5 = 0x90 (standby 500 ms, filter x4)   #1 ctrl_meas 0xF4 = 0x57 (T x2, P x16, normal mode)
//
// A machine takes its Criteria, which says which device it is: Machine<W, Addr<0x76>>. Two sensors (0x76 and 0x77) are two types, each with its
// own statics. The device's data are static members of Machine<W, Criteria>::Dev, so a node refers to them by address. A node is a singleton.
//
// The App names what it publishes from outside. An outer node carries the code and a publish tag that refers to the inner node without copying it:
//   PublishedAt<Code, PathRef<M, 3, 1>, OnSync<fn>>   by a compile-time path into the machine (node #3, child #1): any node, a leaf of a group too
//   Published<Code, Node, node, OnSync<fn>>           by an ItemRef, for a standalone object
// A sync pass over the published nodes calls fn(value) for each one that changed.
//
// Never touch a node from a static initialiser: discover, set, publish, sync. A machine's nodes are objects with constructors of their own, held in
// inline static members of a class template, and those are initialised in no defined order: a static initialiser that runs discovery writes into a
// node before its constructor has run, and the constructor then wipes what was written (the captured registers, the Watch copies). Discover from
// setup() or main().
// describe() walks the machine and the published nodes and writes what a consumer needs: the codes, their path, their fields, which notify.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneData/oneData.h>
#include <oneMenu/oneMenu.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/state/face.h>
#include <oneMachine/fail/devedge.h>

namespace bmpm {

  using discover::RowId;
  using oneMenu::ItemDef;
  using oneMenu::MenuDef;
  using oneMenu::StaticBody;

  // ---- the Criteria: which device a machine is -----------------------------------------------------------------------------------
  template<uint8_t A> struct Addr { static constexpr uint8_t addr = A; };

  // ---- what a node declares about itself, for the walk (components with no behaviour) -----------------------------------------
  // T is a tag with ONEMACHINE_STATE_NAME(name, "text"): the text is a flash array
  template<typename T> struct Label { template<typename O> struct Part : O { using O::O; static constexpr state::Name label() { return T::name(); } }; };
  template<uint8_t N> struct Scaled { template<typename O> struct Part : O { using O::O; static constexpr uint8_t decimals = N; }; };
  template<uint8_t A, uint8_t Def> struct RegAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t regAddr = A, regDef = Def; }; };
  // the values a node accepts, inclusive: a set outside them is refused (the description carries them, so a consumer checks before it sends)
  template<long Lo, long Hi> struct Limits { template<typename O> struct Part : O { using O::O; static constexpr long limLo = Lo, limHi = Hi; }; };
  template<uint8_t A, uint8_t N> struct ConstAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t constAddr = A, constLen = N; }; };
  // the publish tag of an outer node: the code it is published under and the machine node it refers to
  // Via: the PathRef it reaches the node by (void: an ItemRef)
  template<typename Code, typename Node, typename Via = void> struct PubTag { template<typename O> struct Part : O { using O::O; using PubCode = Code; using Inner = Node; using Path = Via; }; };

  // ---- capture and restore (D20, D22, D13) -------------------------------------------------------------------------------------
  // The tail of the restore chain: every component that restores calls Base::restore() after its own, and this ends it.
  struct RestoreEnd { template<typename O> struct Part : O { using O::O; void restore() {} void forget() {} }; };
  // Keeps the last set() and replays it on restore(), then calls the tail. A set while the device is gone is captured too: the last intent is what
  // comes back. forget() drops it (a part that is not the one that was here).
  struct Capture {
    template<typename O> struct Part : O {
      using Base = O;
      using O::O;
      using Type = typename Base::Type;
      Type last{}; bool have = false;
      void set(Type v) { last = v; have = true; Base::set(v); }
      void restore() { if (have) Base::set(last); Base::restore(); }
      void forget() { have = false; Base::forget(); }
      [[nodiscard]] bool known() const { return have; }
      [[nodiscard]] Type captured() const { return last; }
    };
  };

  // no failure handling: the driver polls and nothing is retried, probed or reported
  struct Plain {
    static constexpr bool checked = false, lifecycle = false, returnPath = false, idempotent = true;
    template<typename E> using DevStack = fail::Bare;
  };

  struct TagTemp  { ONEMACHINE_STATE_NAME(name, "temp"); };
  struct TagPress { ONEMACHINE_STATE_NAME(name, "press"); };
  struct TagCal   { ONEMACHINE_STATE_NAME(name, "cal"); };
  struct TagCtrl  { ONEMACHINE_STATE_NAME(name, "ctrl"); };
  struct TagConfig   { ONEMACHINE_STATE_NAME(name, "config"); };
  struct TagCtrlMeas { ONEMACHINE_STATE_NAME(name, "ctrl_meas"); };
  struct TagBmp   { ONEMACHINE_STATE_NAME(name, "bmp280"); };

  struct Cal { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; };

  template<typename W, typename Criteria = Addr<0x76>, typename Mode = Plain>
  struct Machine {
    static constexpr uint8_t addr = Criteria::addr;
    // the device: its data and its register access. Static, as the machine is.
    struct Dev {
      static constexpr uint8_t addr = Criteria::addr;
      inline static int32_t temp = 0, press = 0;   // the last sample: 0.01 C, Pa
      inline static Cal     cal{};                  // device constants
      inline static uint32_t ident = 0;             // the part that was here: a hash of its chip id and calibration
      inline static bool    known = false;          //   (ident is set)
      inline static uint16_t restored = 0, defaulted = 0;   // how often it came back and its captured state was replayed / the defaults were used

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
    template<typename Tag, uint8_t A, uint8_t Def> using Reg = ItemDef<Capture, oneData::DataFn<RegSrc<A>>, RegAt<A, Def>, Limits<0, 255>, Label<Tag>, RestoreEnd>;

    using Temp  = ItemDef<Scaled<2>, Label<TagTemp>,  oneData::ReadOnly<oneData::Watch<oneData::DataRef<&Dev::temp>>>>;
    using Press = ItemDef<Scaled<2>, Label<TagPress>, oneData::ReadOnly<oneData::Watch<oneData::DataRef<&Dev::press>>>>;
    using Cal_  = ItemDef<ConstAt<0x88, 24>, Label<TagCal>>;
    using Config   = Reg<TagConfig,   0xF5, 0x90>;
    using CtrlMeas = Reg<TagCtrlMeas, 0xF4, 0x57>;                       // written after config: config is written in sleep mode
    using Ctrl  = MenuDef<ItemDef<Label<TagCtrl>>, StaticBody<Config, CtrlMeas>, Label<TagCtrl>>;
    using Nodes = hapi::Chain<Temp, Press, Cal_, Ctrl>;

    inline static Temp  temp;
    inline static Press press;
    inline static Cal_  cal;
    inline static Ctrl  ctrl{ItemDef<Label<TagCtrl>>{}, StaticBody<Config, CtrlMeas>{}};

    // ---- path access: a node of the machine by position, then a child of a group, ... -----------------------------------------------
    // by compile-time path (D25: take the first #, call the child with the rest): the node's type, and the node itself
    template<typename L, unsigned I> struct TypeAt;
    template<unsigned I, typename H, typename... T> struct TypeAt<hapi::Chain<H, T...>, I> { using type = typename TypeAt<hapi::Chain<T...>, I - 1>::type; };
    template<typename H, typename... T> struct TypeAt<hapi::Chain<H, T...>, 0> { using type = H; };
    template<typename N, unsigned... P> struct At { using type = N; };
    template<typename N, unsigned I, unsigned... R> struct At<N, I, R...> { using type = typename At<typename TypeAt<typename N::Body::Types, I>::type, R...>::type; };
    template<unsigned I, unsigned... R> struct NodeAt { using type = typename At<typename TypeAt<Nodes, I>::type, R...>::type; };

    template<unsigned I> static auto& node() {
      if constexpr (I == 0) return temp; else if constexpr (I == 1) return press; else if constexpr (I == 2) return cal; else return ctrl;
    }
    template<unsigned I, typename B> static auto& child(B& b) { if constexpr (I == 0) return b.head; else return child<I - 1>(b.tail); }
    template<typename N, unsigned I, unsigned... R> static auto& below(N& n) {
      auto& c = child<I>(n.body);
      if constexpr (sizeof...(R) == 0) return c; else return below<std::remove_reference_t<decltype(c)>, R...>(c);
    }
    template<unsigned I, unsigned... R> static auto& resolve() {
      auto& n = node<I>();
      if constexpr (sizeof...(R) == 0) return n; else return below<std::remove_reference_t<decltype(n)>, R...>(n);
    }

    // by run-time index: a node of the machine, then a register of the group
    template<typename F> static void visit(uint8_t i, F&& fn) {
      switch (i) { case 0: fn(temp); break; case 1: fn(press); break; case 2: fn(cal); break; case 3: fn(ctrl); break; default: break; }
    }
    template<typename F> static void visitReg(uint8_t i, F&& fn) { ctrl.body.visit(i, fn); }

    // ---- the registers of the control group, in order -------------------------------------------------------------------------
    template<typename F> static void eachReg(F&& fn) { for (uint8_t i = 0; i < Ctrl::Body::size(); ++i) ctrl.body.visit(i, fn); }

    // ---- restore: capture replayed in composition order, and the defaults as the init (D13, D20, D22, D33) -----------------------
    // restore(): each register writes its captured value, then calls the tail. forget(): the captured values are dropped (not the part that was here).
    // restoreDefaults(): each register is set to its default, which captures it: restore from defaults is the device init.
    static void restore() { eachReg([](auto& r) { r.restore(); }); }
    static void forget() { eachReg([](auto& r) { r.forget(); }); }
    static void restoreDefaults() { eachReg([](auto& r) { using R = std::remove_reference_t<decltype(r)>; r.set(R::regDef); }); }

    // ---- which part is it: a hash of its chip id and its calibration (a replaced part has its own) -------------------------------
    static uint32_t identOf(uint8_t id, const uint8_t* c) {
      uint32_t h = 2166136261u; h = (h ^ id) * 16777619u;
      for (uint8_t i = 0; i < 24; ++i) h = (h ^ c[i]) * 16777619u;
      return h;
    }
    static void setCal(const uint8_t* c) {
      auto u = [&](uint8_t i) { return uint16_t(c[i] | (uint16_t(c[i + 1]) << 8)); };
      auto s = [&](uint8_t i) { return int16_t(u(i)); };
      auto& d = Dev::cal;
      d.T1 = u(0); d.T2 = s(2); d.T3 = s(4);
      d.P1 = u(6); d.P2 = s(8); d.P3 = s(10); d.P4 = s(12); d.P5 = s(14); d.P6 = s(16); d.P7 = s(18); d.P8 = s(20); d.P9 = s(22);
    }

    // The device is found, or is back (after it was gone, or lost its configuration). Validate: the same chip id and calibration as the part that was
    // here. Validated, the captured state is replayed; not validated (a different part, or the first time), the slot is dropped and the defaults are
    // the init. The calibration is device data, read again every time.
    enum class How : uint8_t { Defaults, Restored };
    static How bring() {
      uint8_t c[24] = {};
      bool ok = false;
      for (uint8_t i = 0; i < 5 && !(ok = Dev::readCal(c)); ++i) {}
      const uint32_t h = ok ? identOf(Dev::rd(0xD0), c) : 0;
      if (ok && Dev::known && h == Dev::ident) {
        setCal(c); restore(); ++Dev::restored;
        return How::Restored;
      }
      forget();
      if (ok) setCal(c); else Dev::cal = Cal{};      // no calibration: poll() produces nothing
      Dev::ident = h; Dev::known = ok;
      restoreDefaults(); ++Dev::defaulted;
      return How::Defaults;
    }

    static void init() {
      Dev::wr(0xE0, 0xB6);   // soft reset: a warm restart finds the part as the last firmware left it
      for (uint8_t i = 0; i < 200; ++i) {   // status bit 0 (im_update) is set while the NVM is copied in
        if (!(Dev::rd(0xF3) & 1)) break;
      }
      (void)bring();
    }

    static constexpr int64_t mul(int64_t v, unsigned n) { return v * (int64_t(1) << n); }

    // compensate the measurement registers (datasheet 8.2, integer form) into the two values
    static void compute(const uint8_t* b) {
      const int32_t adcP = int32_t((uint32_t(b[0]) << 12) | (uint32_t(b[1]) << 4) | (b[2] >> 4));
      const int32_t adcT = int32_t((uint32_t(b[3]) << 12) | (uint32_t(b[4]) << 4) | (b[5] >> 4));
      const auto& d = Dev::cal;
      if (d.T1 == 0) return;           // no calibration
      if (adcT == 0x80000) return;     // 0x80000: no conversion yet (reset value)

      const int32_t v1 = ((((adcT >> 3) - (int32_t(d.T1) << 1))) * int32_t(d.T2)) >> 11;
      const int32_t v2 = (((((adcT >> 4) - int32_t(d.T1)) * ((adcT >> 4) - int32_t(d.T1))) >> 12) * int32_t(d.T3)) >> 14;
      const int32_t tFine = v1 + v2;
      Dev::temp = (tFine * 5 + 128) >> 8;   // 0.01 C

      // the datasheet's shifts are multiplications here: left-shifting a negative value is undefined before C++20 (the same code)
      int64_t p1 = int64_t(tFine) - 128000;
      int64_t p2 = p1 * p1 * int64_t(d.P6);
      p2 = p2 + mul(p1 * int64_t(d.P5), 17);
      p2 = p2 + mul(int64_t(d.P4), 35);
      p1 = ((p1 * p1 * int64_t(d.P3)) >> 8) + mul(p1 * int64_t(d.P2), 12);
      p1 = (mul(1, 47) + p1) * int64_t(d.P1) >> 33;
      if (p1 == 0 || adcP == 0x80000) return;   // no pressure conversion yet
      int64_t p = 1048576 - adcP;
      p = ((mul(p, 31) - p2) * 3125) / p1;
      p1 = (int64_t(d.P9) * (p >> 13) * (p >> 13)) >> 25;
      p2 = (int64_t(d.P8) * p) >> 19;
      p = ((p + p1 + p2) >> 8) + mul(int64_t(d.P7), 4);
      Dev::press = int32_t(p >> 8);        // Pa (Q24.8 >> 8)
    }
    // one burst read of the measurement registers
    static void poll() { uint8_t b[6] = {}; Dev::rdN(0xF7, b, 6); compute(b); }

    // The status of the row this machine's device is bound to in W's registry (discover::Status: Alive 0, Stale 1, Gone 2); Gone when it was never found.
    // Every node of the machine has it: a part that is gone is Stale or Gone, not a register that reads 0xFF.
    static uint8_t status() {
      const discover::IDriver* d = discover::instOf<Driver>();
      for (RowId r = 0; r < W::reg.count; ++r)
        if (!W::reg.rows[r].isBus && W::reg.rows[r].drv == d) return uint8_t(W::reg.status(r));
      return 2;
    }

    // ---- discovery: the machine is the driver's one device ----------------------------------------------------------------
    // Under a failure edge (Mode::checked) each poll first reads the control registers back: a register that no longer holds what was captured
    // means the part was reset without the host knowing (Corrupt: Recover calls reinit(), which validates it and replays the capture). A part that
    // stops answering is Absent, retried and probed; when it answers again the edge calls reinit() the same way.
    struct Driver : discover::DriverBase<Driver, W>, fail::DevEdge<Driver, W, Mode, 1> {
      using B = discover::DriverBase<Driver, W>;
      using Edge = fail::DevEdge<Driver, W, Mode, 1>;
      static constexpr bool polled = true;
      static constexpr uint8_t addrLo = Criteria::addr, addrHi = Criteria::addr, idReg = 0xD0, id = 0x58;
      static constexpr uint8_t recoverMask = fail::bit(fail::Kind::Corrupt);
      static constexpr bool reinitOnBusReturn = true;     // a bus that comes back may have taken the part's supply: validate it again
      static void init(RowId) { Machine::init(); }
      static void reinit(RowId) { (void)Machine::bring(); }
      static void read(RowId row) { Edge::serve(row, fail::Cause::Fresh); }
      static fail::Outcome attempt(RowId row) {
        uint8_t b[6] = {};
        if constexpr (Mode::checked) {
          fail::Outcome o = fail::Outcome::Ok();
          bool lost = false; uint8_t which = 0;
          Machine::eachReg([&](auto& r) {
            using R = std::remove_reference_t<decltype(r)>;
            if (!o.isOk() || lost) return;
            uint8_t v = 0;
            o = Edge::checkedRead(row, R::regAddr, &v, 1);
            if (o.isOk() && r.known() && v != r.captured()) { lost = true; which = R::regAddr; }
          });
          if (!o.isOk()) return o;
          if (lost) return fail::Outcome::Fail(fail::Kind::Corrupt, which);
          o = Edge::checkedRead(row, 0xF7, b, 6);
          if (!o.isOk()) return o;
        } else {
          Dev::rdN(0xF7, b, 6);
        }
        compute(b);
        return fail::Outcome::Ok();
      }
    };
    // the BME280 answers 0x60 at the same register: a second entry, the same driver
    using UseBmp = discover::Use<discover::Own, Driver>;
    using UseBme = discover::Use<discover::IdProbe<0xD0, 0x60, Criteria::addr, Criteria::addr>, Driver>;
    using Entries = hapi::Chain<UseBmp, UseBme>;

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

  template<typename T, typename = void> struct HasCaptured : std::false_type {};
  template<typename T> struct HasCaptured<T, std::void_t<decltype(std::declval<T&>().captured())>> : std::true_type {};

  // a node of a machine by compile-time path: P is the position in the machine, then in each group below it. It reaches any node, a leaf of
  // a group too, by resolving the path to the node's singleton (or to the child inside a group) on use. get(), changed() and sync() are the node's.
  template<typename M, unsigned... P>
  struct PathRef {
    static_assert(sizeof...(P) > 0, "PathRef: a path names a node of the machine");
    using Node = typename M::template NodeAt<P...>::type;
    using Src = M;                                     // where its status comes from: the machine's device
    static constexpr unsigned depth = sizeof...(P);
    static constexpr unsigned path[sizeof...(P)] = {P...};
    template<typename O> struct Part : O {
      using O::O;
      static Node& ref() { return M::template resolve<P...>(); }
      static decltype(auto) get() { return ref().get(); }
      // the value of a part that is not Alive: a register's last set (the intent that comes back), otherwise the last value read
      static decltype(auto) last() { if constexpr (HasCaptured<Node>::value) return ref().captured(); else return ref().get(); }
      [[nodiscard]] static bool changed() { return ref().changed(); }
      static void sync() { ref().sync(); }
      template<typename V> static void set(V&& v) { ref().set(std::forward<V>(v)); }      // through the node: its limits, its capture, its register
    };
  };
  // an outer node reaching its inner node by path
  template<typename Code, typename Ref, typename... Notify>
  using PublishedAt = ItemDef<Notify..., Ref, Label<Code>, PubTag<Code, typename Ref::Node, Ref>>;

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
  //     temp  -> 1/118/0  notify sync  ro scaled 2
  //     air   -> 1/118/3  group 2
  // A path is <bus>/<identity of the device>/<node>, then a register inside a group; the identity is the device's address.
  template<typename T, typename = void> struct HasDecimals : std::false_type {};
  template<typename T> struct HasDecimals<T, std::void_t<decltype(T::decimals)>> : std::true_type {};
  template<typename T, typename = void> struct HasReg : std::false_type {};
  template<typename T> struct HasReg<T, std::void_t<decltype(T::regAddr)>> : std::true_type {};
  template<typename T, typename = void> struct HasLimits : std::false_type {};
  template<typename T> struct HasLimits<T, std::void_t<decltype(T::limLo)>> : std::true_type {};
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
      else if constexpr (HasReg<N>::value) { str(" reg "); hex(N::regAddr); str(" default "); hex(N::regDef); str(HasSet<N>::value ? " rw" : " ro");
        if constexpr (HasLimits<N>::value) { str(" range "); dec(uint32_t(N::limLo)); str(".."); dec(uint32_t(N::limHi)); } }
      else if constexpr (HasConst<N>::value) { str(" const "); hex(N::constAddr); str(" ["); dec(N::constLen); put(']'); }
      else { str(HasSet<N>::value ? " rw" : " ro"); str(" value"); if constexpr (HasDecimals<N>::value) { str(" scaled "); dec(N::decimals); } }
    }

    template<typename... N> void nodes(hapi::Chain<N...>*) { unsigned i = 0; ((str("  #"), dec(i++), put(' '), name(N::label()), fields<N>(), put('\n')), ...); }
    template<typename... R> void regs(hapi::Chain<R...>*) { unsigned i = 0; ((str("    #"), dec(i++), put(' '), name(R::label()), fields<R>(), put('\n')), ...); }

    template<typename M> void machine() {
      str("machine "); name(TagBmp::name()); str(" at "); hex(M::addr); put('\n');
      nodes(static_cast<typename M::Nodes*>(nullptr));
      regs(static_cast<typename M::Ctrl::Body::Types*>(nullptr));
    }

    template<typename M, typename Pub> void published(uint8_t bus) {
      using Inner = typename Pub::Inner;
      str("  "); name(Pub::PubCode::name()); str(" -> "); dec(bus); put('/'); dec(M::addr);
      if constexpr (std::is_void<typename Pub::Path>::value) { put('/'); dec(unsigned(M::template IndexOf<Inner, typename M::Nodes>::value)); }
      else for (unsigned i = 0; i < Pub::Path::depth; ++i) { put('/'); dec(Pub::Path::path[i]); }
      str(NotifiesSync<Pub>::value ? " notify sync" : " silent"); fields<Inner>();
      str(" status "); str(statusName(M::status())); put('\n');
    }
    static const char* statusName(uint8_t st) { return st == 0 ? "alive" : st == 1 ? "stale" : "gone"; }
    template<typename M, typename... Pub> void publishedAll(hapi::Chain<Pub...>*, uint8_t bus) { (published<M, Pub>(bus), ...); }
  };

  // M: the machine (its Criteria is the device's identity in the path); Pubs: Chain<published nodes>; bus: the machine's position in the App
  template<typename M, typename Pubs, typename P> void describe(P& put, uint8_t bus) {
    Walk<P> w{put};
    w.template machine<M>();
    w.str("published\n");
    w.template publishedAll<M>(static_cast<Pubs*>(nullptr), bus);
  }

}
