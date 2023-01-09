#ifndef sim_pic14_data_bus_h
#define sim_pic14_data_bus_h

#include <cstdint>
#include <string>
#include <vector>

namespace sim::pic14::internal {

  class RegisterBackend {
  public:
    virtual ~RegisterBackend() = default;

    virtual uint8_t read_register(uint16_t addr) = 0;
    virtual void write_register(uint16_t addr, uint8_t value) = 0;
  };

  class SRAM {
  public:
    SRAM(uint16_t size, uint8_t reset_value) : reset_value_(reset_value), cells_(size, reset_value) {}

    uint8_t read_register(uint16_t addr) const { return cells_[addr]; }
    void write_register(uint16_t addr, uint8_t value) { cells_[addr] = value; }

    void reset();

  private:
    uint8_t reset_value_;
    std::u8string cells_;
  };

  /// A data bus that can dispatch to backends, or SRAM. Only
  /// SRAM-backed addresses can use `const_read_register`.
  class ActiveDataBus {
  public:
    /// Constructs a new bus. The `backmap` contains indices into
    /// `backends`. An 0xFF entry means "backed by SRAM."
    ActiveDataBus(SRAM &&sram, std::vector<RegisterBackend*> &&backends, std::u8string &&backmap)
      : sram_(std::move(sram)),
        backends_(std::move(backends)),
        backmap_(std::move(backmap)) {}

    uint8_t const_read_register(uint16_t addr) const {
      auto *be = backend(addr);
      if (be) return 0;
      return sram_.read_register(addr);
    }

    uint8_t read_register(uint16_t addr) {
      auto *be = backend(addr);
      return (be ? be->read_register(addr) : sram_.read_register(addr));
    }

    void write_register(uint16_t addr, uint8_t value) {
      auto *be = backend(addr);
      if (be) be->write_register(addr, value);
      else sram_.write_register(addr, value);
    }

  private:
    const RegisterBackend* backend(uint16_t addr) const {
      auto bi = backmap_[addr];
      return (bi == 0xFF ? nullptr : backends_[bi]);
    }

    RegisterBackend* backend(uint16_t addr) {
      auto bi = backmap_[addr];
      return (bi == 0xFF ? nullptr : backends_[bi]);
    }

  private:
    SRAM sram_;
    std::vector<RegisterBackend*> backends_;
    std::u8string backmap_;
  };

  /// A data bus with a virtual address space.
  template<typename Base>
  class MappingDataBus : public Base {
  public:
    /// Constructs a new bus with the given address map. The `addrmap`
    /// view is borrowed, and must remain valid during the lifetime of
    /// the DataBus.
    MappingDataBus(Base &&bus, std::u16string_view addrmap) : Base(std::move(bus)), addrmap_(addrmap) {}

    uint8_t const_read_register(uint16_t addr) const { return Base::const_read_register(map_address(addr)); }
    uint8_t read_register(uint16_t addr) { return Base::read_register(map_address(addr)); }
    void write_register(uint16_t addr, uint8_t value) { Base::write_register(map_address(addr), value); }

  protected:
    uint16_t map_address(uint16_t addr) const {
      if (addr >= addrmap_.size()) {
        addr %= addrmap_.size();
      }
      return addrmap_[addr];
    }

  private:
    std::u16string_view addrmap_;
  };

  /// A data bus that implements indirect addressing.
  template<typename Base, uint16_t IndAddr, uint16_t StatusAddr, uint16_t FsrAddr, int IRPBit>
  class IndirectDataBus : public Base {
  public:
    IndirectDataBus(Base &&bus) : Base(std::move(bus)) {}

    uint8_t const_read_register(uint16_t addr) const { return Base::const_read_register(map_address(addr)); }
    uint8_t read_register(uint16_t addr) { return Base::read_register(map_address(addr)); }
    void write_register(uint16_t addr, uint8_t value) { Base::write_register(map_address(addr), value); }

  protected:
    uint16_t map_address(uint16_t addr) const {
      if (Base::map_address(addr) == IndAddr) {
        addr = irp_base() | Base::const_read_register(FsrAddr);
      }
      return addr;
    }

  private:
    uint16_t irp_base() const {
      return (uint16_t) Base::const_read_register(StatusAddr) << (8 - IRPBit) & 0x100;
    }
  };

  class DataBus : public IndirectDataBus<MappingDataBus<ActiveDataBus>, 0x00, 0x03, 0x04, 7> {
  public:
    /// The `addrmap` view is borrowed, and must remain valid during
    /// the lifetime of the DataBus.
    DataBus(uint16_t size, uint8_t reset_value, std::vector<RegisterBackend*> &&backs, std::u8string &&backmap, std::u16string_view addrmap)
      : IndirectDataBus(MappingDataBus(ActiveDataBus(SRAM(size, reset_value), std::move(backs), std::move(backmap)), addrmap)) {}
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_data_bus_h
