// Consumer bindings on a discovered row, beyond the sample fan-out.
//   direct connection    Shell<W,Iface> {row, Iface*}, bound where the concrete driver type is known (found<Impl>);
//                        usable only while the row is alive and still holds that driver.
//   capability-set class OutClass = a Chain of op layers over OutBase; the handle dispatches to the provider type by
//                        pointer compare over the providers, so it adds no virtual call.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "registry.h"
#include "state.h"

namespace discover {

  // ---- direct connection ------------------------------------------------------------------------
  template<typename W, typename Iface, typename Owner = void>
  struct Shell {
    RowId  row = noRow;
    Iface* p   = nullptr;

    // a new binding is a new session with the device: the consumer's ClientState starts over
    void bind(RowId r, Iface* d) { row = r; p = d; resetClient(); }
    void unbind() { row = noRow; p = nullptr; resetClient(); }
    void release(RowId r) { if (row == r) unbind(); }

    // this consumer's ClientState (Owner names the consumer)
    static auto& client() {
      static_assert(ClientStateOf<Iface>::has, "driver declares no ClientState");
      static_assert(!std::is_same<Owner, void>::value, "a shell on a driver with a ClientState needs its consumer as Owner (else consumers share it)");
      return ClientCell<Owner, Iface>::v;
    }
    void resetClient() { if constexpr (ClientStateOf<Iface>::has) zeroBytes(client()); }

    // the interface, or nullptr unless the row is Alive and still holds this driver (a rediscovery can reuse the index)
    [[nodiscard]] Iface* get() const {
      if (!p) return nullptr;
      if constexpr (LifecycleOf<W>::value) { if (W::reg.driverOf(row) != instOf<Iface>()) return nullptr; }   // total: nothing indexes past the table
      if (W::reg.status(row) != Status::Alive) return nullptr;
      if (W::reg.rows[row].drv != instOf<Iface>()) return nullptr;
      return p;
    }
  };

  // ---- binder fold: consumers with wants<Impl>, bind(row, Impl*), unbind(), served<Drivers> ----------
  template<typename L> struct BinderSet;
  template<typename... C>
  struct BinderSet<hapi::Chain<C...>> {
    template<typename Impl> static void bind(RowId row) { (bindOne<C, Impl>(row), ...); }
    static void unbind() { (C::unbind(), ...); }
    static void release(RowId row) { (C::release(row), ...); }
    template<typename Drivers> static constexpr bool served = (C::template served<Drivers> && ...);
  private:
    template<typename Cn, typename Impl> static void bindOne([[maybe_unused]] RowId row) {
      if constexpr (Cn::template wants<Impl>) Cn::bind(row, &Inst<Impl>::obj);
    }
  };

  // ---- capability-set output class ----------------------------------------------------------------
  struct Print {};        // operation capabilities: what a driver can be asked to do
  struct Clear {};
  struct SetCursor {};

  template<typename D, typename = void> struct OpsOf { using Type = hapi::Chain<>; };
  template<typename D> struct OpsOf<D, std::void_t<typename D::Ops>> { using Type = typename D::Ops; };

  // predicate: the driver's Ops contain all of Ops... (ListHas shape)
  template<typename... Ops>
  struct ProvidesAll {
    template<typename O> using Check = typename hapi::Traverse<ProvidesAll<Ops...>,O>::Beta;
    template<typename O> using Apply = std::bool_constant<(hapi::Exists<hapi::SameAs<Ops>, typename OpsOf<O>::Type>::value && ...)>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };
  template<typename L> struct ProvidesAllOf;
  template<typename... Ops> struct ProvidesAllOf<hapi::Chain<Ops...>> { using Type = ProvidesAll<Ops...>; };

  template<typename P> struct TypeTag { using Type = P; };

  template<typename L> struct ProviderSet;
  template<typename... P>
  struct ProviderSet<hapi::Chain<P...>> {
    static constexpr uint8_t size = sizeof...(P);
    // call f(TypeTag<P>) for the provider type behind `d`; false if `d` is none of them
    template<typename F> static bool visit(const IDriver* d, F&& f) {
      return ((d == instOf<P>() ? (f(TypeTag<P>{}), true) : false) || ...);
    }
  };

  // a class handle holds one ClientState slot sized to the largest its providers declare (nothing when none does)
  template<typename Providers, typename S = CliSlotOf<Providers>> struct OutHeld : Held<S> {};

  template<typename W, typename Providers>
  struct OutBase : OutHeld<Providers> {
    static_assert(ProviderSet<Providers>::size > 0, "no driver in the list provides this output class");
    RowId          row = noRow;
    const IDriver* drv = nullptr;

    // `replace`: the last provider row wins instead of the first; a new binding starts a new client session
    void bind(RowId r, const IDriver* d, bool replace) {
      if (replace || row == noRow) { row = r; drv = d; this->resetClient(); }
    }
    void unbind() { row = noRow; drv = nullptr; this->resetClient(); }
    void release(RowId r) { if (row == r) unbind(); }

    // the ClientState of provider P
    template<typename P> auto& client() {
      return this->cs.template as<typename ClientStateOf<P>::Type>();
    }

    // calls f(TypeTag<Provider>) only while the row is Alive and still holds the driver it was bound to
    template<typename F> bool visit(F&& f) const {
      if constexpr (LifecycleOf<W>::value) {
        // a released binding has no driver and never indexes; a live one is checked against the table with a total accessor
        if (!drv || W::reg.driverOf(row) != drv) return false;
      }
      if (row == noRow || W::reg.status(row) != Status::Alive) return false;
      if (W::reg.rows[row].drv != drv) return false;
      return ProviderSet<Providers>::visit(drv, static_cast<F&&>(f));
    }
  };

  // op layers: each adds one member function that calls the provider type's static function of the same name
  struct OpPrint {
    template<typename T> struct Part : T {
      bool print(const char* s) {
        return this->visit([&](auto t) {
          using P = typename decltype(t)::Type;
          if constexpr (ClientStateOf<P>::has) P::print(this->row, this->template client<P>(), s);
          else P::print(this->row, s);
        });
      }
    };
  };
  struct OpClear {
    template<typename T> struct Part : T {
      bool clear() {
        return this->visit([&](auto t) {
          using P = typename decltype(t)::Type;
          if constexpr (ClientStateOf<P>::has) P::clear(this->row, this->template client<P>());
          else P::clear(this->row);
        });
      }
    };
  };

  template<typename NeedsL, typename LayersL> struct OutClass { using Needs = NeedsL; using Layers = LayersL; };
  using TextOut = OutClass<hapi::Chain<Print, Clear>, hapi::Chain<OpPrint, OpClear>>;

  // the handle type for a class over a driver list: providers = drivers whose Ops cover the class
  template<typename W, typename Class, typename Drivers>
  using OutHandleT = typename Class::Layers::template Part<
    OutBase<W, hapi::Eval<hapi::Filter<typename ProvidesAllOf<typename Class::Needs>::Type>, Drivers>>>;

  template<typename Impl, typename NeedsL> struct Provides;
  template<typename Impl, typename... Ops>
  struct Provides<Impl, hapi::Chain<Ops...>> : std::bool_constant<(hapi::Exists<hapi::SameAs<Ops>, typename OpsOf<Impl>::Type>::value && ...)> {};

}
