// Glue between the failure stacks and discoverCompose's registry (read-only: nothing in ../discoverCompose changes).
#pragma once
#include <oneMachine/discover/registry.h>
#include "layers.h"
#include "inject.h"

namespace fail {

  // W: the world (reg); Ds: Chain<drivers>. A driver provides tickRow(row, now). Direct calls only.
  template<typename W, typename Ds> struct TickFold;
  template<typename W, typename... D> struct TickFold<W, hapi::Chain<D...>> {
    static void run([[maybe_unused]] uint32_t now) { (one<D>(now), ...); }
  private:
    template<typename Dr> static void one(uint32_t now) {
      for (RowId r = 0; r < W::reg.count; ++r)
        if (W::reg.rows[r].drv == discover::instOf<Dr>()) Dr::tickRow(r, now);
    }
  };

  // the drivers whose stack has tick(now); empty (and free) when no driver chose a ticking component
  template<typename W, typename Ds> using Ticks = TickFold<W, hapi::Eval<hapi::Filter<TicksStack>, Ds>>;

}
