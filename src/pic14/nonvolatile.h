#ifndef sim_pic14_nonvolatile_h
#define sim_pic14_nonvolatile_h

#include <cstdint>
#include <functional>
#include <string_view>
#include <system_error>

#include "../util/status.h"

namespace sim::pic14 {

  class ICSP;

  namespace internal {

    /// The non-volatile memory of a PIC14 device.
    ///
    /// This memory can be programmed in-circuit.
    class NonVolatile {
      friend class sim::pic14::ICSP;

    public:
      using ResetSignal = std::function<void()>;

      struct Config {
        uint16_t prog_size;
        uint16_t config_size;
        uint16_t eedata_size;
      };

      NonVolatile(const Config &config, ResetSignal reset)
        : progmem_(config.prog_size, 0x3FFF),
          config_(config.config_size, 0x3FFF),
          eedata_(config.eedata_size, 0xFF),
          reset_(std::move(reset)) {}

    public:
      /// Returns whether the device is currently in ICSP mode. While
      /// in this mode, no normal operations should run.
      bool is_in_icsp() const { return in_icsp; }

      /// Enters ICSP mode, allowing programming the non-volatile
      /// memory. When the returned ICSP object is destroyed, the
      /// device leaves ICSP mode.
      ICSP enter_icsp();

      std::u16string& progmem() { return progmem_; }
      std::u16string& config() { return config_; }
      std::u8string& eedata() { return eedata_; }

    private:
      std::u16string progmem_;
      std::u16string config_;
      std::u8string eedata_;

      ResetSignal reset_;
      bool in_icsp = false;
    };

  } // namespace internal

  /// A handle of an on-going ICSP.
  ///
  /// When destroyed, it informs the device it should reset itself.
  ///
  /// Instances can be moved, but not copied.
  class ICSP {
  public:
    ICSP(internal::NonVolatile *device) : device(device) {
      device->in_icsp = true;
    }

    ~ICSP() {
      device->in_icsp = false;
      device->reset_();
    }

    /// Programs memory at the specified address. For 14-bit data
    /// (program memory, configuration words), little-endian is used
    /// when decoding. This can also program EEPROM data presented as
    /// 16-bit values where the top byte is discarded.
    sim::util::Status load_program(uint16_t addr, std::u8string_view data);

    /// Program data at the specified address. Data is represented as
    /// 8-bit values.
    sim::util::Status load_data(uint16_t addr, std::u8string_view data);

    ICSP(ICSP&&) = default;
    ICSP(const ICSP&) = delete;
    ICSP& operator=(const ICSP&) = delete;

  private:
    internal::NonVolatile *device;
  };

}  // namespace sim::pic14

#endif // sim_pic14_nonvolatile_h
