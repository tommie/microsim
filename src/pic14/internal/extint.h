#ifndef sim_pic14_internal_extint_h
#define sim_pic14_internal_extint_h

#include "core.h"
#include "interrupt.h"
#include "pin.h"

namespace sim::pic14::internal {

  /// External interrupt pin.
  class ExternalInterrupt {
  public:
    ExternalInterrupt(Core::OptionReg &&option_reg, InterruptMux::MaskableIntconEdgeSignal &&interrupt);

    InputPin *pin() { return &pin_; }

  private:
    Core::OptionReg option_reg_;
    InterruptMux::MaskableIntconEdgeSignal interrupt_;
    InputPin pin_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_internal_extint_h
