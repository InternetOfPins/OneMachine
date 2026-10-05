// What the spi example publishes of its air sensor: the codes, the published nodes and the card's line, in one place. The sketch (main.cpp), the
// simulated device (test/link/tree_device.cpp), the AVR image (test/link/avr_tree.cpp) and the description generator (examples/spi/describe.cpp)
// all build them from here, so the description the generator writes is the one the firmware's hash is computed from.
//   M      the machine (bmpm::Machine<W, Criteria, Mode>)
//   Note   Note<Code>::fn: a constexpr pointer to what a published value calls when it changes (the App's: print it, mark it for the link, ...)
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "bmp280_machine.h"

namespace airTree {

  // the codes, numbered by their position in the description (the link's change records use the numbers)
  struct CodeTemp     { static constexpr uint8_t num = 0; ONEMACHINE_STATE_NAME(name, "temp"); };
  struct CodePress    { static constexpr uint8_t num = 1; ONEMACHINE_STATE_NAME(name, "press"); };
  struct CodeAir      { static constexpr uint8_t num = 2; ONEMACHINE_STATE_NAME(name, "air"); };
  struct CodeConfig   { static constexpr uint8_t num = 3; ONEMACHINE_STATE_NAME(name, "air/config"); };
  struct CodeCtrlMeas { static constexpr uint8_t num = 4; ONEMACHINE_STATE_NAME(name, "air/ctrl_meas"); };
  struct CodeCard     { static constexpr uint8_t num = 5; ONEMACHINE_STATE_NAME(name, "card"); };

  // the sensor: the machine at 0x76 (its Criteria), under the App W's failure handling Mode
  template<typename W, typename Mode = bmpm::Plain> using Machine = bmpm::Machine<W, bmpm::Addr<0x76>, Mode>;
  constexpr uint8_t bus = 1;   // the air sensor's bus is the App's second machine: its path codes start with 1

  // temp and press notify on sync; air is the control group, config and ctrl_meas its registers (silent), each by its path in the machine
  template<typename M, template<typename> class Note> using Pubs = hapi::Chain<
    bmpm::PublishedAt<CodeTemp,     bmpm::PathRef<M, 0>,    oneData::OnSync<Note<CodeTemp>::fn>>,
    bmpm::PublishedAt<CodePress,    bmpm::PathRef<M, 1>,    oneData::OnSync<Note<CodePress>::fn>>,
    bmpm::PublishedAt<CodeAir,      bmpm::PathRef<M, 3>>,
    bmpm::PublishedAt<CodeConfig,   bmpm::PathRef<M, 3, 0>>,
    bmpm::PublishedAt<CodeCtrlMeas, bmpm::PathRef<M, 3, 1>>>;

  // the card: a code that only notifies (an event with a value, the UID, 0 when it leaves). The App's text line is cardText then the status of the
  // reader's row; the description's static line (its hash, the build output) is cardText without the trailing " status ".
  // (a macro, so an App's text path uses it as a literal of its own, as before: the text build is the same image to the byte)
  #define AIRTREE_CARD_TEXT "  card -> 0/1 notify event ro value u32 status "
  inline constexpr char cardText[] = AIRTREE_CARD_TEXT;
  template<typename P> constexpr void cardLine(P& put) { for (unsigned i = 0; i + 8 < sizeof cardText - 1; ++i) put(cardText[i]); put('\n'); }
}
