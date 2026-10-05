// What the spi example publishes of its RFID reader: the codes and the published nodes, from the RC522 machine (rc522_machine.h). air_tree.h puts
// them in one description with the air sensor's.
//   card          an event: the UID of the card that arrived, 0 when it left
//   rfid/gain     the receiver gain: bits 6..4 of RFCfgReg, 0..7 (4 is the chip's reset value). What a step is in dB is the datasheet's table, which is not
//                 monotonic (0..3 repeat): it is the consumer's, not a scale here
//   rfid/antenna  the TX drivers: bits 1..0 of TxControlReg, 0..3 (0 both off, 3 both on, 1 or 2 one of them)
// The reader's timer registers are reconciled by the machine and not published: the driver's interrupt timeout assumes the 25 ms they give.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/manifest.h>
#include "rc522_machine.h"

namespace rfidTree {

  // the codes, numbered by their position in the description: after the air sensor's five
  struct CodeCard    { static constexpr uint8_t num = 5; ONEMACHINE_STATE_NAME(name, "card"); };
  struct CodeGain    { static constexpr uint8_t num = 6; ONEMACHINE_STATE_NAME(name, "rfid/gain"); };
  struct CodeAntenna { static constexpr uint8_t num = 7; ONEMACHINE_STATE_NAME(name, "rfid/antenna"); };

  constexpr uint8_t bus = 0;    // the reader's bus is the App's first machine: its path codes start with 0
  constexpr uint8_t slot = 0;   // its chip select's position in the SPI bus's slots (main.cpp: slot 0 is the RC522's, slot 1 is declared empty)

  // the reader: the machine at its slot (its Criteria), under the App W's failure handling Mode
  template<typename W, typename Mode = rc522::NoFail> using Machine = rc522m::Machine<W, rc522m::Slot<slot>, Mode>;

  // card is node #0 of the machine; gain and antenna are registers #6 and #7 of its group #2
  template<typename M> using Pubs = hapi::Chain<
    rc522m::PublishedAt<CodeCard,    rc522m::PathRef<M, 0>>,
    rc522m::PublishedAt<CodeGain,    rc522m::PathRef<M, 2, 6>>,
    rc522m::PublishedAt<CodeAntenna, rc522m::PathRef<M, 2, 7>>>;

}
