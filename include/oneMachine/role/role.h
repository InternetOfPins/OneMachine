#pragma once
// oneMachine/role/role.h -- what a machine's outputs are for, and the only names a consumer uses to reach them.
//
// A role is role::Role<Tag, Kind, Endpoint>:
//   Tag       ONEMACHINE_STATE_NAME(name, "x"): the role's name, unique in the machine. It names the role's layer in the command and the
//             report, so a consumer writes `x/target_um` and reads `x/pos_um`, never a channel, an address or a path.
//   Kind      what sort of output it is (role/kinds.h: Switch, Light, Axis): its command and report fields, their units, its parameters
//             and its safe command. Two roles of one kind differ by their Tag and their parameters: kind and name together are the meaning.
//   Endpoint  where it is, known only to the device: a fixed one (pins, role/sim.h), one bound when discovery finds its device
//             (role/found.h, the default for a discovered device), or one pinned by path (role/route.h). The device routes every request
//             itself; the consumer never sees this.
//
// role::Machine<Items...> collects roles and references (role/ref.h) and gives:
//   Command, Report       state compositions (state/state.h), one layer per role; the report adds `live` to every role: false when its
//                         device is not there (not found, gone, not yet pinned) and its command was not applied
//   wants<Impl>, bind(row, Impl*), unbind(), release(row), served<D>
//                         a discover:: binder (BinderSet's shape): DriverBase::found() -> W::bind<Impl>(row) -> here -> the Found endpoints
//                         whose driver is Impl. Only those are compiled into Impl's found().
//   pin()                 the Pinned endpoints look their path up again (after a discover() or a reprobe); nothing for the others
//   apply(cmd, rep)       at the cycle boundary: every live role writes its command, reads back its report
//   sense(rep)            refresh the report without writing (every cycle)
//   safe(cmd, rep)        replace the command with every kind's safe one (the supervisor went quiet); apply it after
//   describe(put)         the machine description (role/face.h)
//   Tuning, tuning, valid  the roles whose kind is role::Tuned<Kind>: their parameters as a state composition (one layer per tuned role,
//                         the kind's Tune fields), the machine's current values (one per image, starting at the template arguments),
//                         and the check that every value stays inside what the firmware allows. A machine without a tuned role has
//                         an empty Tuning and pays nothing. apply, sense and safe use the current values of a tuned role.
// Rules, compile errors: two roles on one endpoint; two roles with one name (state.h's own rule on the layers).
#include <oneMachine/state/state.h>

namespace role {
  struct RoleItem {};                                          // what Machine filters roles by (hapi::TagIs)
  struct TunedItem {};                                         // ... and the tuned ones

  // Tuned<Kind>: the same kind, with its parameters changeable at run time (over the link: role/link.h ops T, G, S), inside the template
  // arguments, which stay the firmware's limits. In RAM: a reset goes back to the template arguments.
  template<class K> struct Tuned : K { using base = K; };
  template<class K> struct IsTuned : std::false_type {};
  template<class K> struct IsTuned<Tuned<K>> : std::true_type {};
  template<class K> struct KindOf { using type = K; };
  template<class K> struct KindOf<Tuned<K>> { using type = K; };
  struct NotTuned {};

  template<class Tag, class Kind, class Endpoint>
  struct Role : RoleItem, std::conditional_t<IsTuned<Kind>::value, TunedItem, NotTuned> {
    using tag = Tag; using kind = typename KindOf<Kind>::type; using endpoint = Endpoint;
    static constexpr bool tuned = IsTuned<Kind>::value;
  };

  // the report slot of a role: its kind's fields, then `live`
  template<class K> struct Reported : K::Report {
    bool live;
    ONEMACHINE_STATE_NAME(n_live, "live");
    template<class Self, class F> static constexpr void each(Self& s, F& f) { K::Report::each(s, f); f(n_live(), s.live); }
  };

  template<class R> using CommandLayer = state::Layer<typename R::tag, typename R::kind::Command>;
  template<class R> using ReportLayer  = state::Layer<typename R::tag, Reported<typename R::kind>>;
  template<class R> using TuneLayer    = state::Layer<typename R::tag, typename R::kind::Tune>;
  template<class... LL> using StateOf  = typename hapi::APIOf<state::API, LL...>::Res;

  template<class I> struct IsRole : std::bool_constant<std::is_base_of<RoleItem, I>::value> {};
  template<class I> struct IsTunedRole : std::bool_constant<std::is_base_of<TunedItem, I>::value> {};

  // what an endpoint may declare, each optional: pin(); wants<Impl> with bind<Impl>(row); unbind(); release(row)
  template<class E, class = void> struct HasPin : std::false_type {};
  template<class E> struct HasPin<E, std::void_t<decltype(E::pin())>> : std::true_type {};
  template<class E, class Impl, class = void> struct Wants : std::false_type {};
  template<class E, class Impl> struct Wants<E, Impl, std::void_t<decltype(E::template wants<Impl>)>> : std::bool_constant<E::template wants<Impl>> {};
  template<class E, class = void> struct HasUnbind : std::false_type {};
  template<class E> struct HasUnbind<E, std::void_t<decltype(E::unbind())>> : std::true_type {};
  template<class I, class Impl, bool = IsRole<I>::value> struct RoleWants : std::false_type {};
  template<class I, class Impl> struct RoleWants<I, Impl, true> : Wants<typename I::endpoint, Impl> {};
  template<class Impl, class... II> struct AnyWants : std::false_type {};
  template<class Impl, class I, class... II> struct AnyWants<Impl, I, II...> : std::bool_constant<RoleWants<I, Impl>::value || AnyWants<Impl, II...>::value> {};

  // how many roles of Items... sit on endpoint E (recursive: MSVC and AVR's type_traits both stay out of it)
  template<class E, class... II> struct OnEndpoint { static constexpr unsigned n = 0; };
  template<class E, class I, class... II> struct OnEndpoint<E, I, II...> {
    template<class J, bool = IsRole<J>::value> struct Is { static constexpr unsigned v = 0; };
    template<class J> struct Is<J, true> { static constexpr unsigned v = std::is_same<typename J::endpoint, E>::value ? 1u : 0u; };
    static constexpr unsigned n = Is<I>::v + OnEndpoint<E, II...>::n;
  };
  template<class... II> struct EndpointsUnique { static constexpr bool value = true; };
  template<class I, class... II> struct EndpointsUnique<I, II...> {
    template<class J, bool = IsRole<J>::value> struct Ok { static constexpr bool v = true; };
    template<class J> struct Ok<J, true> { static constexpr bool v = OnEndpoint<typename J::endpoint, II...>::n == 0; };
    static constexpr bool value = Ok<I>::v && EndpointsUnique<II...>::value;
  };

  template<class... Items>
  struct Machine {
    static_assert(EndpointsUnique<Items...>::value, "role: two roles on one endpoint");
    using Roles   = hapi::Eval<hapi::Filter<hapi::TagIs<RoleItem>>, hapi::Chain<Items...>>;
    using Command = typename Roles::template Map<CommandLayer>::template Build<StateOf>;
    using Report  = typename Roles::template Map<ReportLayer>::template Build<StateOf>;
    using TunedRoles = hapi::Eval<hapi::Filter<hapi::TagIs<TunedItem>>, hapi::Chain<Items...>>;
    using Tuning  = typename TunedRoles::template Map<TuneLayer>::template Build<StateOf>;
    static constexpr bool tunable = (IsTunedRole<Items>::value || ... || false);
    static inline Tuning tuning{};
    static bool valid(const Tuning& t) { return (validOne<Items>(t) && ... && true); }

    template<class Impl> static constexpr bool wants = AnyWants<Impl, Items...>::value;
    template<class Impl> static void bind(uint8_t row, Impl*) { (bindOne<Items, Impl>(row), ...); }
    static void unbind() { (unbindOne<Items>(), ...); }
    static void release(uint8_t row) { (releaseOne<Items>(row), ...); }
    template<class Drivers> static constexpr bool served = true;
    static void pin() { (pinOne<Items>(), ...); }
    static void apply(const Command& c, Report& r) { (applyOne<Items>(c, r), ...); }
    static void sense(Report& r) { (senseOne<Items>(r), ...); }
    static void safe(Command& c, const Report& r) { (safeOne<Items>(c, r), ...); }
    template<class V> static void each(V& v) { (v.template item<Items>(), ...); }   // the items in order: faces walk this

  private:
    template<class I> static void pinOne() { if constexpr (IsRole<I>::value) { if constexpr (HasPin<typename I::endpoint>::value) I::endpoint::pin(); } }
    template<class I, class Impl> static void bindOne([[maybe_unused]] uint8_t row) { if constexpr (RoleWants<I, Impl>::value) I::endpoint::template bind<Impl>(row); }
    template<class I> static void unbindOne() { if constexpr (IsRole<I>::value) { if constexpr (HasUnbind<typename I::endpoint>::value) I::endpoint::unbind(); } }
    template<class I> static void releaseOne([[maybe_unused]] uint8_t row) { if constexpr (IsRole<I>::value) { if constexpr (HasUnbind<typename I::endpoint>::value) I::endpoint::release(row); } }
    template<class I> static void applyOne(const Command& c, Report& r) {
      if constexpr (IsRole<I>::value) {
        using T = typename I::tag; auto& rep = state::get<T>(r);
        rep.live = I::endpoint::live();
        if (!rep.live) return;
        if constexpr (IsTunedRole<I>::value) I::kind::template apply<typename I::endpoint>(state::get<T>(c), rep, state::get<T>(tuning));
        else I::kind::template apply<typename I::endpoint>(state::get<T>(c), rep);
      }
    }
    template<class I> static void senseOne(Report& r) {
      if constexpr (IsRole<I>::value) {
        using T = typename I::tag; auto& rep = state::get<T>(r);
        rep.live = I::endpoint::live();
        if (!rep.live) return;
        if constexpr (IsTunedRole<I>::value) I::kind::template sense<typename I::endpoint>(rep, state::get<T>(tuning));
        else I::kind::template sense<typename I::endpoint>(rep);
      }
    }
    template<class I> static void safeOne(Command& c, const Report& r) {
      if constexpr (IsRole<I>::value) {
        using T = typename I::tag;
        if constexpr (IsTunedRole<I>::value) I::kind::safe(state::get<T>(c), state::get<T>(r), state::get<T>(tuning));
        else I::kind::safe(state::get<T>(c), state::get<T>(r));
      }
    }
    template<class I> static bool validOne([[maybe_unused]] const Tuning& t) {
      if constexpr (IsTunedRole<I>::value) return I::kind::valid(state::get<typename I::tag>(t)); else return true;
    }
  };
}
