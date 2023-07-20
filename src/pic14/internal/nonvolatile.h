#ifndef sim_pic14_internal_nonvolatile_h
#define sim_pic14_internal_nonvolatile_h

#include <cstdint>
#include <vector>

namespace sim::pic14::internal {

  /// The non-volatile memory of a PIC14 device.
  ///
  /// This memory can be programmed in-circuit.
  class NonVolatile {
  public:
    struct Config {
      uint16_t prog_size;
      uint16_t config_size;
      uint16_t eedata_size;
    };

    explicit NonVolatile(const Config &config)
      : progmem_(config.prog_size, 0x3FFF),
        config_(config.config_size, 0x3FFF),
        eedata_(config.eedata_size, 0xFF) {}

  public:
    std::vector<uint16_t>& progmem() { return progmem_; }
    std::vector<uint16_t>& config() { return config_; }
    std::vector<uint8_t>& eedata() { return eedata_; }

  private:
    std::vector<uint16_t> progmem_;
    std::vector<uint16_t> config_;
    std::vector<uint8_t> eedata_;
  };

}  // namespace sim::pic14::internal

#endif // sim_pic14_internal_nonvolatile_h
