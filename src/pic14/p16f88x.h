#ifndef sim_pic14_p16f88x_h
#define sim_pic14_p16f88x_h

#include "../core/device.h"
#include "../core/scheduler.h"
#include "adc.h"
#include "core.h"
#include "eprom.h"
#include "eusart.h"
#include "execution.h"
#include "extint.h"
#include "port.h"
#include "register.h"
#include "timer0.h"
#include "ulpwu.h"
#include "watchdog.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sim::pic14 {

  namespace internal {

    constexpr int pgmdat_buf_size(uint16_t prog_size) {
      if (prog_size < 8192) return 4;
      else return 8;
    }

    template<uint16_t ProgSize_, uint16_t EEDataSize_, int NumPorts_, int PgmDatBufSize_ = pgmdat_buf_size(ProgSize_)>
    struct P16F88XConfig {
      static constexpr uint16_t ProgSize = ProgSize_;
      static constexpr uint16_t EEDataSize = EEDataSize_;
      static constexpr int NumPorts = NumPorts_;
      static constexpr int PgmDatBufSize = PgmDatBufSize_;
    };

    template<typename Config>
    class P16F88X : public sim::core::Device {
      static const uint16_t FILE_BUS_SIZE = 0x200;
      static const uint16_t PROG_SIZE = Config::ProgSize;
      static const uint16_t EEDATA_SIZE = Config::EEDataSize;
      static const uint16_t CONFIG_SIZE = 9;

      using EPROMType = internal::EPROM<Config::PgmDatBufSize, self_write_cutoffs<Config::ProgSize>()>;

    public:
      explicit P16F88X(sim::core::DeviceListener *listener, sim::core::Clock *extosc);

      std::vector<sim::core::Clock*> clock_sources() override;
      sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

      ICSP enter_icsp() { return core_.enter_icsp(); }
      bool is_sleeping() const { return core_.is_sleeping(); }
      const std::vector<sim::core::PinDescriptor>& pins() const override { return pin_descrs_; }

    private:
      internal::DataBus build_data_bus();
      std::vector<sim::core::PinDescriptor> build_pin_descrs();

      void reset();

    private:
      internal::NonVolatile nv_;
      internal::Core core_;
      internal::InterruptMux interrupt_mux_;
      internal::Executor executor_;
      internal::WatchDogTimer wdt_;
      internal::Timer0 timer0_;
      std::array<internal::Port, Config::NumPorts - 1> ports_;
      internal::InterruptiblePort portb_;
      internal::ExternalInterrupt extint_;
      internal::ADConverter adc_;
      internal::UltraLowPowerWakeUp ulpwu_;
      EPROMType eprom_;
      internal::EUSART eusart_;
      std::vector<sim::core::PinDescriptor> pin_descrs_;
      sim::core::Scheduler scheduler_;

      static const std::u16string_view address_map();
    };

  }  // namespace internal

  using P16F884 = internal::P16F88X<internal::P16F88XConfig<4096, 256, 5, 4>>;
  using P16F887 = internal::P16F88X<internal::P16F88XConfig<8192, 256, 5, 8>>;

}  // namespace sim::pic14

#endif  // sim_pic14_p16f88x_h
