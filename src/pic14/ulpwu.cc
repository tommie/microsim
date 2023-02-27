#include "ulpwu.h"

namespace sim::pic14::internal {

  double ULPWUPin::resistance() const {
    return enabled_ ? 0.5 : 1;
  }

  UltraLowPowerWakeUp::UltraLowPowerWakeUp(sim::core::DeviceListener *listener, Core::PConReg &&pcon, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt)
    : listener_(listener),
      pcon_reg_(std::move(pcon)),
      interrupt_(std::move(interrupt)),
      pin_([this](bool value) {
        if (!value) interrupt_.raise();
      }) {}

  void UltraLowPowerWakeUp::pcon_updated() {
    if (pin_.enabled_ == pcon_reg_.ulpwue())
      return;

    pin_.enabled_ = pcon_reg_.ulpwue();
    listener_->pin_changed(&pin_, sim::core::DeviceListener::RESISTANCE);
  }

}  // namespace sim::pic14::internal
