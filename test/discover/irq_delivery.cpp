// The ESP8266 delivery components (examples/spi/src/irq_esp8266.h) against the host stub: the pins they accept compile, the strapping pins
// and GPIO16 for an ISR are rejected with their own message (-DNEG_*).
#include <Arduino.h>
#include "../../examples/spi/src/irq_esp8266.h"

#if defined(NEG_D3)
  using L = irq::Sampled<0>;
#elif defined(NEG_D4)
  using L = irq::IsrFlag<2>;
#elif defined(NEG_D8)
  using L = irq::Sampled<15>;
#elif defined(NEG_ISR16)
  using L = irq::IsrFlag<16>;
#elif defined(NEG_PAST)
  using L = irq::Sampled<17>;
#else
  using L = irq::Sampled<16>;       // D0: no interrupt, sampled
  using F = irq::IsrFlag<5>;        // D1: an ISR flag
#endif

int main() {
  L::begin(); L::arm();
#if !defined(NEG_D3) && !defined(NEG_D4) && !defined(NEG_D8) && !defined(NEG_ISR16) && !defined(NEG_PAST)
  F::begin(); F::arm(); F::isr();
  return F::ready() && !L::ready() ? 0 : 1;   // the stub reads the line high
#else
  return L::ready();
#endif
}
