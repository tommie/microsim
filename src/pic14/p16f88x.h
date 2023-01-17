#ifndef sim_pic14_p16f88x_h
#define sim_pic14_p16f88x_h

#include "../core/device.h"
#include "../core/scheduler.h"
#include "execution.h"
#include "port.h"
#include "register.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sim::pic14 {

  namespace internal {

    template<uint16_t ProgSize, uint16_t EEDataSize, int NumPorts>
    class P16F88X : public sim::core::Device {
      static const uint16_t FILE_BUS_SIZE = 0x200;
      static const uint16_t PROG_SIZE = ProgSize;
      static const uint16_t EEDATA_SIZE = EEDataSize;
      static const uint16_t CONFIG_SIZE = 9;

    public:
      explicit P16F88X(core::DeviceListener *listener);

      sim::core::Advancement advance_to(const sim::core::SimulationLimit &limit) override;

      ICSP enter_icsp() { return nv_.enter_icsp(); }
      bool is_sleeping() const { return executor_.is_sleeping(); }
      const std::vector<sim::core::PinDescriptor>& pins() const override { return pin_descrs_; }

    private:
      internal::DataBus build_data_bus();
      std::vector<sim::core::PinDescriptor> build_pin_descrs();

      const OptionReg option_reg() const { return OptionReg(const_cast<DataBus*>(&executor_.data_bus())); }
      OptionReg option_reg() { return OptionReg(&executor_.data_bus()); }

      void reset(uint8_t status);

    private:
      sim::core::Clock fosc4_;
      sim::core::ClockScheduler clock_scheduler_;
      sim::core::SignalQueue signal_queue_;

      internal::InterruptMux interrupt_mux_;
      internal::NonVolatile nv_;
      internal::Executor executor_;
      std::array<internal::Port, NumPorts - 1> ports_;
      internal::InterruptiblePort portb_;
      std::vector<sim::core::PinDescriptor> pin_descrs_;
      sim::core::Scheduler scheduler_;

      static const std::u16string_view address_map();
    };

  }  // namespace internal

  using P16F884 = internal::P16F88X<4096, 256, 5>;
  using P16F887 = internal::P16F88X<8192, 256, 5>;

}  // namespace sim::pic14

#endif  // sim_pic14_p16f88x_h
