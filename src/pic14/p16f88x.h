#ifndef sim_pic14_p16f88x_h
#define sim_pic14_p16f88x_h

#include "../core/device.h"
#include "../core/scheduler.h"
#include "execution.h"
#include "port.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sim::pic14 {

  class P16F88X : public internal::Execution {
    static const uint16_t FILE_BUS_SIZE = 0x200;
    static const uint16_t PROG_SIZE = 8192;
    static const uint16_t EEDATA_SIZE = 256;
    static const uint16_t CONFIG_SIZE = 9;

  public:
    explicit P16F88X(core::DeviceListener *listener);

    sim::core::Advancement advance_to(const sim::core::SimulationLimit &limit) override;

    const std::vector<sim::core::PinDescriptor>& pins() const override { return pin_descrs_; }

    bool is_sleeping() const { return internal::Execution::is_sleeping(); }

  private:
    internal::DataBus build_data_bus();
    std::vector<sim::core::PinDescriptor> build_pin_descrs();

  private:
    sim::core::Clock fosc4_;
    sim::core::ClockScheduler clock_scheduler_;

    std::array<internal::Port, 4> ports_;
    internal::InterruptiblePort portb_;
    std::vector<sim::core::PinDescriptor> pin_descrs_;
    internal::Executor executor_;
    sim::core::Scheduler scheduler_;

    static const std::u16string_view address_map();
  };

}  // namespace sim::pic14

#endif  // sim_pic14_p16f88x_h
