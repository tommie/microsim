#ifndef sim_pic14_ulpwu_h
#define sim_pic14_ulpwu_h

#include "../core/device.h"
#include "core.h"
#include "interrupt.h"
#include "pin.h"

namespace sim::pic14::internal {

  class ULPWUPin : public InputPin {
    friend class UltraLowPowerWakeUp;

  public:
    using InputPin::InputPin;

    double resistance() const override;

  private:
    bool enabled_ = false;
  };

  class UltraLowPowerWakeUp {
  public:
    UltraLowPowerWakeUp(sim::core::DeviceListener *listener, Core::PConReg &&pcon, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt);

    InputPin *pin() { return &pin_; }

    void pcon_updated();

  private:
    sim::core::DeviceListener *listener_;
    Core::PConReg pcon_reg_;
    InterruptMux::MaskablePeripheralEdgeSignal interrupt_;
    ULPWUPin pin_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_ulpwu_h
