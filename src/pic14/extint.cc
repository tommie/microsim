#include "extint.h"

namespace sim::pic14::internal {

  ExternalInterrupt::ExternalInterrupt(Core::OptionReg &&option_reg, InterruptMux::MaskableIntconEdgeSignal &&interrupt)
    : option_reg_(std::move(option_reg)),
      interrupt_(std::move(interrupt)),
      pin_([this](bool rising) {
        if (option_reg_.intedg() == rising) {
          interrupt_.raise();
        }
      }) {}

}  // namespace sim::pic14::internal
