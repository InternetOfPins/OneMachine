#pragma once
// oneMachine/role/ref.h -- a pointer from the machine description to a fuller one (a document, a drawing, a robot description): a `ref`
// line. What it points to is the describer's concern, not the machine's; the machine only carries the pointer.
//   Ref<Text>      fixed in the firmware: Text declares ONEMACHINE_STATE_NAME(name, "https://..."), in flash on AVR
//   RefFrom<Src>   given at run time: Src::ref() returns a RAM string or nullptr (a user setting, read from EEPROM); no line when empty
// Both go in role::Machine<...> beside the roles and cost nothing unless the description is printed (role/face.h).
namespace role {
  template<class Text> struct Ref {};
  template<class Src>  struct RefFrom {};
}
