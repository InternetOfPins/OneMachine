// An RC522 (MFRC522) as a static machine of OneMenu ItemDef nodes, found by its VersionReg on an SPI slot (rc522.h is its driver).
//
//   Machine<W, Slot<0>, Mode>::Nodes   #0 card     the UID of the card in the field, 0 when it leaves: an event, read-only
//                                      #1 version  VersionReg: device data, read when the part is found, never state, never published
//                                      #2 rf       a group of register mimics (get() reads the chip, set() writes it); the defaults are the init
//                                                  #0 tmode 0x2A = 0x80    #1 tprescaler 0x2B = 0xA9   #2 treload_h 0x2C = 0x03   #3 treload_l 0x2D = 0xE8
//                                                  #4 txask 0x15 = 0x40    #5 mode 0x11 = 0x3D         #6 rfcfg 0x26 = 0x48 (receiver gain, bits 6..4)
//                                                  #7 txcontrol 0x14 = 0x83 (antenna on), written last
//                                      rfcfg and txcontrol are seen as a field of their register (Field): rfcfg bits 6..4 (the receiver gain, 0..7), txcontrol
//                                      bits 1..0 (the two TX drivers, 0..3). A set of the field keeps the other bits of the register as they are wanted.
//
// A machine takes its Criteria, which says which device it is: Machine<W, Slot<0>>, the position of its chip select in the bus's SpiSlots. A second
// reader on slot 1 is another type, with statics of its own. The device's data are static members of Machine<W, Criteria>::Dev, so a node refers to
// them by address; a node is a singleton. As for bmp280_machine.h, never touch a node from a static initialiser: discover from setup() or main().
//
// The registers of `rf` are what the chip is configured with (the whole register, also for a field), and the driver's canary: each poll reads them back, and a chip that no longer holds
// what is wanted of one (desired()) was reset behind the host's back (Corrupt). init() is a soft reset and the two waits, then the registers written
// until they read back. What is wanted of a register is the default until a set() changes it; a set while the part is gone changes it too, and a part
// that comes back (the same VersionReg) gets what was last wanted. A different VersionReg is another part: the defaults.
// The interrupt registers (ComIEnReg, DivIEnReg) are not nodes: they change during every command and belong to the driver's interrupt part. The
// counters (inits, initTries, bccErrors, collisions, missStreak) stay the driver's.
//
// Written against bmp280_machine.h's shape, in a header of its own: the parts marked (=bmpm) are the same text there. They are not shared yet, so that
// what both machines use can be seen when a second one exists.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneData/oneData.h>
#include <oneMenu/oneMenu.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/state/face.h>
#include "rc522.h"

namespace rc522m {

  using discover::RowId;
  using oneMenu::ItemDef;
  using oneMenu::MenuDef;
  using oneMenu::StaticBody;

  // ---- the Criteria: which device a machine is -----------------------------------------------------------------------------------
  template<uint8_t S> struct Slot { static constexpr uint8_t slot = S; };

  // ---- what a node declares about itself, for the walk (components with no behaviour) -----------------------------------------
  template<typename T> struct Label { template<typename O> struct Part : O { using O::O; static constexpr state::Name label() { return T::name(); } }; };   // (=bmpm)
  template<uint8_t A, uint8_t Def> struct RegAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t regAddr = A, regDef = Def; }; };   // (=bmpm)
  template<long Lo, long Hi> struct Limits { template<typename O> struct Part : O { using O::O; static constexpr long limLo = Lo, limHi = Hi; }; };         // (=bmpm)
  template<uint8_t A, uint8_t N> struct ConstAt { template<typename O> struct Part : O { using O::O; static constexpr uint8_t constAddr = A, constLen = N; }; };   // (=bmpm)
  // The node is a field of its register: bits Shift+Width-1..Shift. get() reads the chip and gives the field; set() writes the register with the field
  // replaced and every other bit as the register's desired() holds it. Above Reconcile, which keeps holding and checking the whole register: desired()
  // stays the register's; intent() is the field of it (what a code of a part that is not there answers).
  template<unsigned Shift, unsigned Width> struct Field {
    template<typename O> struct Part : O {
      using Base = O;
      using O::O;
      static constexpr unsigned fieldShift = Shift, fieldWidth = Width;
      static constexpr uint8_t fieldMask = uint8_t((1u << Width) - 1);
      static constexpr uint8_t fieldDef = uint8_t((Base::regDef >> Shift) & fieldMask);
      [[nodiscard]] uint8_t get() const { return uint8_t((Base::get() >> Shift) & fieldMask); }
      void set(uint8_t v) { Base::set(uint8_t((Base::desired() & ~(fieldMask << Shift)) | ((v & fieldMask) << Shift))); }
      [[nodiscard]] uint8_t intent() const { return uint8_t((Base::desired() >> Shift) & fieldMask); }
    };
  };
  // the node is an event: its value is told once when it happens (queued), it is never a state
  struct Event { template<typename O> struct Part : O { using O::O; static constexpr bool event = true; }; };
  // the publish tag of an outer node: the code it is published under and the machine node it refers to; Via: the PathRef it reaches the node by   (=bmpm)
  template<typename Code, typename Node, typename Via = void> struct PubTag { template<typename O> struct Part : O { using O::O; using PubCode = Code; using Inner = Node; using Path = Via; }; };

  // ---- desired and observed: one Reconcile part (=bmpm) ----------------------------------------------------------------------------
  // The register holds what is wanted of it (`want`), from the start: its default (RegAt's regDef, below it in the composition). set() changes what is
  // wanted and writes it; apply() writes what is wanted; reset() makes the default what is wanted again; the driver's canary compares the register it
  // reads back with desired().
  struct Reconcile {
    template<typename O> struct Part : O {
      using Base = O;
      using O::O;
      using Type = typename Base::Type;
      Type want = Type(Base::regDef);
      void set(Type v) { want = v; Base::set(v); }
      void apply() { Base::set(want); }
      void reset() { want = Type(Base::regDef); }
      [[nodiscard]] Type desired() const { return want; }
    };
  };

  struct TagCard      { ONEMACHINE_STATE_NAME(name, "card"); };
  struct TagVersion   { ONEMACHINE_STATE_NAME(name, "version"); };
  struct TagRf        { ONEMACHINE_STATE_NAME(name, "rf"); };
  struct TagTMode     { ONEMACHINE_STATE_NAME(name, "tmode"); };
  struct TagTPresc    { ONEMACHINE_STATE_NAME(name, "tprescaler"); };
  struct TagTReloadH  { ONEMACHINE_STATE_NAME(name, "treload_h"); };
  struct TagTReloadL  { ONEMACHINE_STATE_NAME(name, "treload_l"); };
  struct TagTxAsk     { ONEMACHINE_STATE_NAME(name, "txask"); };
  struct TagMode      { ONEMACHINE_STATE_NAME(name, "mode"); };
  struct TagRfCfg     { ONEMACHINE_STATE_NAME(name, "rfcfg"); };
  struct TagTxControl { ONEMACHINE_STATE_NAME(name, "txcontrol"); };

  template<typename W, typename Criteria = Slot<0>, typename Mode = rc522::NoFail>
  struct Machine {
    static constexpr uint8_t slot = Criteria::slot;
    // the device: its data and its register access, by slot. Static, as the machine is.
    struct Dev {
      static constexpr uint8_t slot = Criteria::slot;
      inline static uint32_t uid = 0;               // the card in the field: 0 when none
      inline static uint8_t  version = 0;           // VersionReg, as read when the part was found
      inline static uint32_t ident = 0;             // the part that was here: a hash of its VersionReg
      inline static bool     known = false;         //   (ident is set)
      inline static bool     pending = false;       // an initialisation is under way and has not read the part's VersionReg yet
      inline static uint16_t restored = 0, defaulted = 0;   // how often it came back and what was wanted was applied again / the defaults were used

      static void xfer(uint8_t* tx, uint8_t* rx, uint16_t n) {
        discover::SpiCfg<typename W::Bus>::use(rc522::spiHzOf, rc522::spiModeOf);
        W::Bus::xfer(slot, tx, rx, n);
      }
      static uint8_t rd(uint8_t reg) { uint8_t io[2] = {uint8_t(((reg << 1) & 0x7E) | 0x80), 0}; xfer(io, io, 2); return io[1]; }
      static void wr(uint8_t reg, uint8_t v) { uint8_t io[2] = {uint8_t((reg << 1) & 0x7E), v}; xfer(io, nullptr, 2); }
    };

    // a register mimic: get() reads the chip, set() writes it
    template<uint8_t A> struct RegSrc { static uint8_t get() { return Dev::rd(A); } static void set(uint8_t v) { Dev::wr(A, v); } };
    template<typename Tag, uint8_t A, uint8_t Def, typename Rule = Limits<0, 255>>
    using Reg = ItemDef<Reconcile, oneData::DataFn<RegSrc<A>>, RegAt<A, Def>, Rule, Label<Tag>>;
    template<typename Tag, uint8_t A, uint8_t Def, unsigned Shift, unsigned Width>
    using FieldReg = ItemDef<Field<Shift, Width>, Reconcile, oneData::DataFn<RegSrc<A>>, RegAt<A, Def>, Limits<0, (1 << Width) - 1>, Label<Tag>>;

    using Card    = ItemDef<Event, Label<TagCard>, oneData::ReadOnly<oneData::Watch<oneData::DataRef<&Dev::uid>>>>;
    using Version = ItemDef<ConstAt<rc522::VersionReg, 1>, Label<TagVersion>>;
    using TMode     = Reg<TagTMode,     rc522::TModeReg,     0x80>;       // timer starts at the end of a transmission
    using TPresc    = Reg<TagTPresc,    rc522::TPrescalerReg, 0xA9>;      // 40 kHz tick
    using TReloadH  = Reg<TagTReloadH,  rc522::TReloadRegH,  0x03>;       // 1000 ticks: a 25 ms receive timeout
    using TReloadL  = Reg<TagTReloadL,  rc522::TReloadRegL,  0xE8>;
    using TxAsk     = Reg<TagTxAsk,     rc522::TxASKReg,     0x40>;       // 100% ASK
    using ModeR     = Reg<TagMode,      rc522::ModeReg,      0x3D>;       // CRC preset 0x6363
    using RfCfg     = FieldReg<TagRfCfg,     rc522::RFCfgReg,     0x48, 4, 3>;   // receiver gain, bits 6..4 (0x48: gain 4, the chip's reset value)
    using TxControl = FieldReg<TagTxControl, rc522::TxControlReg, 0x83, 0, 2>;   // the TX drivers, bits 1..0 (0x83: both on; bit 7 inverts TX2 and is kept)
    using Rf    = MenuDef<ItemDef<Label<TagRf>>, StaticBody<TMode, TPresc, TReloadH, TReloadL, TxAsk, ModeR, RfCfg, TxControl>, Label<TagRf>>;
    using Nodes = hapi::Chain<Card, Version, Rf>;

    inline static Card    card;
    inline static Version version;
    inline static Rf      rf{ItemDef<Label<TagRf>>{}, StaticBody<TMode, TPresc, TReloadH, TReloadL, TxAsk, ModeR, RfCfg, TxControl>{}};

    // ---- path access: a node of the machine by position, then a child of a group, ... (=bmpm, with this machine's nodes) --------------
    template<typename L, unsigned I> struct TypeAt;
    template<unsigned I, typename H, typename... T> struct TypeAt<hapi::Chain<H, T...>, I> { using type = typename TypeAt<hapi::Chain<T...>, I - 1>::type; };
    template<typename H, typename... T> struct TypeAt<hapi::Chain<H, T...>, 0> { using type = H; };
    template<typename N, unsigned... P> struct At { using type = N; };
    template<typename N, unsigned I, unsigned... R> struct At<N, I, R...> { using type = typename At<typename TypeAt<typename N::Body::Types, I>::type, R...>::type; };
    template<unsigned I, unsigned... R> struct NodeAt { using type = typename At<typename TypeAt<Nodes, I>::type, R...>::type; };

    template<unsigned I> static auto& node() {
      if constexpr (I == 0) return card; else if constexpr (I == 1) return version; else return rf;
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
    template<typename F> static void visit(uint8_t i, F&& fn) {
      switch (i) { case 0: fn(card); break; case 1: fn(version); break; case 2: fn(rf); break; default: break; }
    }

    // ---- the registers of the group, in order ----------------------------------------------------------------------------------------
    template<typename F> static void eachReg(F&& fn) { for (uint8_t i = 0; i < Rf::Body::size(); ++i) rf.body.visit(i, fn); }
    static void apply() { eachReg([](auto& r) { r.apply(); }); }
    static void reset() { eachReg([](auto& r) { r.reset(); }); }
    static void restoreDefaults() { reset(); apply(); }

    // every register of the group reads back what is wanted of it
    static bool holds() {
      bool all = true;
      eachReg([&](auto& r) { using R = std::remove_reference_t<decltype(r)>; if (all && Dev::rd(R::regAddr) != r.desired()) all = false; });
      return all;
    }

    // ---- which part is it: a hash of its VersionReg (the part has no data of its own) ----------------------------------------------
    static uint32_t identOf(uint8_t id) { return (2166136261u ^ id) * 16777619u; }

    // The part is found, or is back (after it was gone, or lost its configuration): the same VersionReg as the part that was here, and what was wanted is
    // applied again; another one (or the first time), and the defaults are what is wanted. A part that does not answer its VersionReg yet (the oscillator
    // is starting) is asked again at the next write.
    static void identify() {
      const uint8_t v = Dev::rd(rc522::VersionReg);
      if (!rc522::Ids::has(v)) return;
      Dev::version = v; Dev::pending = false;
      const uint32_t h = identOf(v);
      if (Dev::known && h == Dev::ident) { ++Dev::restored; return; }
      reset(); Dev::ident = h; Dev::known = true; ++Dev::defaulted;
    }

    // what the driver's init() asks of the configuration (rc522::NoConf's interface)
    struct Conf {
      static constexpr bool on = true;
      static void begin(RowId) { Dev::pending = true; }
      static void write(RowId) { if (Dev::pending) identify(); apply(); }
      static bool ok(RowId) { return holds(); }
      static void seen(uint32_t uid) { Dev::uid = uid; }
    };

    // The status of the row this machine's device is bound to in W's registry (discover::Status: Alive 0, Stale 1, Gone 2); Gone when it was never found.
    // Every node of the machine has it.
    static uint8_t status();

    // the driver: rc522.h's, with this machine as its configuration; one entry of the bus's scan
    using Driver  = rc522::Rc522<W, Mode, 1, Conf>;
    using Entries = hapi::Chain<Driver>;

    template<typename N, typename L> struct IndexOf;
    template<typename N, typename... T> struct IndexOf<N, hapi::Chain<T...>> {
      template<unsigned I, typename... R> struct Find { static constexpr int value = -1; };
      template<unsigned I, typename H, typename... R> struct Find<I, H, R...> { static constexpr int value = std::is_same<N, H>::value ? int(I) : Find<I + 1, R...>::value; };
      static constexpr int value = Find<0, T...>::value;
    };
  };

  template<typename W, typename Criteria, typename Mode>
  uint8_t Machine<W, Criteria, Mode>::status() {
    const discover::IDriver* d = discover::instOf<Driver>();
    for (RowId r = 0; r < W::reg.count; ++r)
      if (!W::reg.rows[r].isBus && W::reg.rows[r].drv == d) return uint8_t(W::reg.status(r));
    return 2;
  }

  // ---- publishing: an outer node with a code, reaching its inner node by path (=bmpm) ----------------------------------------------
  template<typename T, typename = void> struct HasDesired : std::false_type {};
  template<typename T> struct HasDesired<T, std::void_t<decltype(std::declval<T&>().desired())>> : std::true_type {};
  template<typename T, typename = void> struct HasIntent : std::false_type {};
  template<typename T> struct HasIntent<T, std::void_t<decltype(std::declval<T&>().intent())>> : std::true_type {};

  // a node of a machine by compile-time path: P is the position in the machine, then in each group below it
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
      static decltype(auto) last() {
        if constexpr (HasIntent<Node>::value) return ref().intent(); else if constexpr (HasDesired<Node>::value) return ref().desired(); else return ref().get();
      }
      [[nodiscard]] static bool changed() { return ref().changed(); }
      static void sync() { ref().sync(); }
      template<typename V> static void set(V&& v) { ref().set(std::forward<V>(v)); }      // through the node: its limits, its register
    };
  };
  template<typename Code, typename Ref, typename... Notify>
  using PublishedAt = ItemDef<Notify..., Ref, Label<Code>, PubTag<Code, typename Ref::Node, Ref>>;

  template<typename T> struct OnSyncOf : std::false_type {};
  template<auto fn> struct OnSyncOf<oneData::OnSync<fn>> : std::true_type {};
  template<typename T> struct NotifiesSync : std::false_type {};
  template<typename... OO> struct NotifiesSync<ItemDef<OO...>> : std::bool_constant<(OnSyncOf<OO>::value || ...)> {};

  // ---- the description walk ---------------------------------------------------------------------------------------------------------
  //   machine rc522 at slot 0
  //     #0 card ro value u32
  //     #1 version const 0x37 [1]
  //     #2 rf group 8
  //       #6 rfcfg reg 0x26 default 0x48 rw range 0..112
  //   published
  //     card -> 0/0/0 notify event ro value u32 status alive
  //     rfid/gain -> 0/0/2/6 silent reg 0x26 default 0x48 rw range 0..112 status alive
  // A path is <bus>/<identity of the device>/<node>, then a register inside a group; the identity is the device's slot.
  template<typename T, typename = void> struct HasReg : std::false_type {};
  template<typename T> struct HasReg<T, std::void_t<decltype(T::regAddr)>> : std::true_type {};
  template<typename T, typename = void> struct HasLimits : std::false_type {};
  template<typename T> struct HasLimits<T, std::void_t<decltype(T::limLo)>> : std::true_type {};
  template<typename T, typename = void> struct HasField : std::false_type {};
  template<typename T> struct HasField<T, std::void_t<decltype(T::fieldMask)>> : std::true_type {};
  template<typename T, typename = void> struct HasConst : std::false_type {};
  template<typename T> struct HasConst<T, std::void_t<decltype(T::constAddr)>> : std::true_type {};
  template<typename T, typename = void> struct HasSet : std::false_type {};
  template<typename T> struct HasSet<T, std::void_t<decltype(std::declval<T&>().set(0))>> : std::true_type {};
  template<typename T, typename = void> struct IsGroup : std::false_type {};
  template<typename T> struct IsGroup<T, std::void_t<typename T::Body>> : std::true_type {};
  template<typename T, typename = void> struct IsEvent : std::false_type {};
  template<typename T> struct IsEvent<T, std::void_t<decltype(T::event)>> : std::bool_constant<T::event> {};

  // the default a code shows: the register's, or the field's of it
  template<typename N> constexpr uint8_t fieldOf() { if constexpr (HasField<N>::value) return N::fieldDef; else return N::regDef; }

  // Walk is the text the device sends (a published line ends with the status of its row); HashWalk writes the same lines without the status, as a
  // compile-time fold (the description's hash).
  template<typename P> struct Walk {
    P& put;
    void str(const char* s) { while (*s) put(*s++); }
    void name(state::Name n) { state::put_name(put, n); }
    void dec(uint32_t v) { state::put_dec(put, v); }
    void hex(uint8_t v) { static const char d[] = "0123456789ABCDEF"; put('0'); put('x'); put(d[v >> 4]); put(d[v & 15]); }

    template<typename N> void fields() {
      if constexpr (IsGroup<N>::value) { str(" group "); dec(N::Body::size()); }
      else if constexpr (HasReg<N>::value) {
        str(" reg "); hex(N::regAddr); str(" default "); hex(fieldOf<N>());
        if constexpr (HasField<N>::value) { str(" field "); dec(N::fieldShift + N::fieldWidth - 1); str(".."); dec(N::fieldShift); }
        str(HasSet<N>::value ? " rw" : " ro");
        if constexpr (HasLimits<N>::value) { str(" range "); dec(uint32_t(N::limLo)); str(".."); dec(uint32_t(N::limHi)); }
      }
      else if constexpr (HasConst<N>::value) { str(" const "); hex(N::constAddr); str(" ["); dec(N::constLen); put(']'); }
      else if constexpr (IsEvent<N>::value) str(" ro value u32");
      else { str(HasSet<N>::value ? " rw" : " ro"); str(" value"); }
    }

    template<typename... N> void nodes(hapi::Chain<N...>*) { unsigned i = 0; ((str("  #"), dec(i++), put(' '), name(N::label()), fields<N>(), put('\n')), ...); }
    template<typename... R> void regs(hapi::Chain<R...>*) { unsigned i = 0; ((str("    #"), dec(i++), put(' '), name(R::label()), fields<R>(), put('\n')), ...); }

    template<typename M> void machine() {
      str("machine "); str(rc522::Manifest::name); str(" at slot "); dec(M::slot); put('\n');
      nodes(static_cast<typename M::Nodes*>(nullptr));
      regs(static_cast<typename M::Rf::Body::Types*>(nullptr));
    }

    template<typename M, typename Pub> void published(uint8_t bus) {
      using Inner = typename Pub::Inner;
      str("  "); name(Pub::PubCode::name()); str(" -> "); dec(bus); put('/'); dec(M::slot);
      for (unsigned i = 0; i < Pub::Path::depth; ++i) { put('/'); dec(Pub::Path::path[i]); }
      str(IsEvent<Inner>::value ? " notify event" : NotifiesSync<Pub>::value ? " notify sync" : " silent"); fields<Inner>();
      str(" status "); str(statusName(M::status())); put('\n');
    }
    static const char* statusName(uint8_t st) { return st == 0 ? "alive" : st == 1 ? "stale" : "gone"; }
    template<typename M, typename... Pub> void publishedAll(hapi::Chain<Pub...>*, uint8_t bus) { (published<M, Pub>(bus), ...); }
  };

  template<typename P> struct HashWalk {
    P& put;
    constexpr void str(const char* s) { while (*s) put(*s++); }
    constexpr void name(state::Name n) { for (unsigned i = 0; n.at(i); ++i) put(n.at(i)); }
    constexpr void dec(uint32_t v) { char b[10] = {}; unsigned k = 0; do { b[k++] = char('0' + v % 10); v /= 10; } while (v); while (k) put(b[--k]); }
    constexpr void hex(uint8_t v) { const uint8_t hi = uint8_t(v >> 4), lo = uint8_t(v & 15); put('0'); put('x'); put(char(hi < 10 ? '0' + hi : 'A' + hi - 10)); put(char(lo < 10 ? '0' + lo : 'A' + lo - 10)); }
    template<typename N> constexpr void fields() {
      if constexpr (IsGroup<N>::value) { str(" group "); dec(N::Body::size()); }
      else if constexpr (HasReg<N>::value) {
        str(" reg "); hex(N::regAddr); str(" default "); hex(fieldOf<N>());
        if constexpr (HasField<N>::value) { str(" field "); dec(N::fieldShift + N::fieldWidth - 1); str(".."); dec(N::fieldShift); }
        str(HasSet<N>::value ? " rw" : " ro");
        if constexpr (HasLimits<N>::value) { str(" range "); dec(uint32_t(N::limLo)); str(".."); dec(uint32_t(N::limHi)); }
      }
      else if constexpr (HasConst<N>::value) { str(" const "); hex(N::constAddr); str(" ["); dec(N::constLen); put(']'); }
      else if constexpr (IsEvent<N>::value) str(" ro value u32");
      else { str(HasSet<N>::value ? " rw" : " ro"); str(" value"); }
    }
    template<typename... N> constexpr void nodes(hapi::Chain<N...>*) { unsigned i = 0; ((str("  #"), dec(i++), put(' '), name(N::label()), fields<N>(), put('\n')), ...); }
    template<typename... R> constexpr void regs(hapi::Chain<R...>*) { unsigned i = 0; ((str("    #"), dec(i++), put(' '), name(R::label()), fields<R>(), put('\n')), ...); }
    template<typename M> constexpr void machine() {
      str("machine "); str(rc522::Manifest::name); str(" at slot "); dec(M::slot); put('\n');
      nodes(static_cast<typename M::Nodes*>(nullptr));
      regs(static_cast<typename M::Rf::Body::Types*>(nullptr));
    }
    template<typename M, typename Pub> constexpr void published(uint8_t bus) {
      using Inner = typename Pub::Inner;
      str("  "); name(Pub::PubCode::name()); str(" -> "); dec(bus); put('/'); dec(M::slot);
      for (unsigned i = 0; i < Pub::Path::depth; ++i) { put('/'); dec(Pub::Path::path[i]); }
      str(IsEvent<Inner>::value ? " notify event" : NotifiesSync<Pub>::value ? " notify sync" : " silent"); fields<Inner>(); put('\n');
    }
    template<typename M, typename... Pub> constexpr void publishedAll(hapi::Chain<Pub...>*, uint8_t bus) { (published<M, Pub>(bus), ...); }
  };

}
