#ifndef sim_pic14_internal_data_bus_h
#define sim_pic14_internal_data_bus_h

#include <cstdint>
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
    std::vector<uint8_t> cells_;
  };

  /// A data bus that can dispatch to backends, or SRAM. Only
  /// SRAM-backed addresses can use `const_read_register`.
  class ActiveDataBus {
  public:
    /// Constructs a new bus. The `backmap` contains indices into
    /// `backends`. An 0xFF entry means "backed by SRAM." The
    /// `indexmap` is a mapping from address space to per-backend
    /// index space. An 0xFFFF entry means "identity mapping."
    ActiveDataBus(SRAM &&sram, std::vector<RegisterBackend*> &&backends, std::vector<uint8_t> &&backmap, std::vector<uint16_t> &&indexmap)
      : sram_(std::move(sram)),
        backends_(std::move(backends)),
        backmap_(std::move(backmap)),
        indexmap_(std::move(indexmap)) {}

    uint8_t const_read_register(uint16_t addr) const {
      auto *be = backend(addr);
      if (be) return 0;
      return sram_.read_register(index_for(addr));
    }

    uint8_t read_register(uint16_t addr) {
      auto *be = backend(addr);
      return (be ? be->read_register(index_for(addr)) : sram_.read_register(index_for(addr)));
    }

    void write_register(uint16_t addr, uint8_t value) {
      auto *be = backend(addr);
      if (be) be->write_register(index_for(addr), value);
      else sram_.write_register(index_for(addr), value);
    }

  protected:
    uint16_t index_for(uint16_t addr) const {
      if (addr >= indexmap_.size()) {
        addr %= indexmap_.size();
      }

      uint16_t index = indexmap_[addr];
      return (index == 0xFFFF ? addr : index);
    }

  private:
    const RegisterBackend* backend(uint16_t addr) const {
      if (addr >= backmap_.size()) {
        addr %= backmap_.size();
      }

      auto bi = backmap_[addr];
      return (bi == 0xFF ? nullptr : backends_[bi]);
    }

    RegisterBackend* backend(uint16_t addr) {
      if (addr >= backmap_.size()) {
        addr %= backmap_.size();
      }

      auto bi = backmap_[addr];
      return (bi == 0xFF ? nullptr : backends_[bi]);
    }

  private:
    SRAM sram_;
    std::vector<RegisterBackend*> backends_;
    std::vector<uint8_t> backmap_;
    std::vector<uint16_t> indexmap_;
  };

  /// A data bus that implements indirect addressing.
  template<typename Base, uint16_t StatusAddr, uint16_t FsrAddr, int IRPBit>
  class IndirectDataBus : public Base {
  public:
    IndirectDataBus(Base &&bus) : Base(std::move(bus)) {}

    enum class Register : uint16_t {
      INDF = 0xFFFE,  // We don't compare backends for this, so this value must be unique across the index map.
    };

    uint8_t const_read_register(uint16_t addr) const { return Base::const_read_register(map_address(addr)); }
    uint8_t read_register(uint16_t addr) { return Base::read_register(map_address(addr)); }
    void write_register(uint16_t addr, uint8_t value) { Base::write_register(map_address(addr), value); }

  protected:
    uint16_t map_address(uint16_t addr) const {
      if (Base::index_for(addr) == static_cast<uint16_t>(Register::INDF)) {
        addr = irp_base() | Base::const_read_register(FsrAddr);
      }
      return addr;
    }

  private:
    uint16_t irp_base() const {
      return (uint16_t) Base::const_read_register(StatusAddr) << (8 - IRPBit) & 0x100;
    }
  };

  class DataBus : public IndirectDataBus<ActiveDataBus, 0x03, 0x04, 7> {
  public:
    /// The `addrmap` view is borrowed, and must remain valid during
    /// the lifetime of the DataBus.
    DataBus(uint16_t size, uint8_t reset_value, std::vector<RegisterBackend*> &&backs, std::vector<uint8_t> &&backmap, std::vector<uint16_t> &&indexmap)
      : IndirectDataBus(ActiveDataBus(SRAM(size, reset_value), std::move(backs), std::move(backmap), std::move(indexmap))) {}
  };

  template<size_t BusSize>
  class DataBusBuilder {
  public:
    template<typename I>
    struct IndexMapping {
      I index;
      std::initializer_list<uint16_t> addrs;
    };

    template<typename IC>
    DataBusBuilder(const IC &indexmap)
      : backmap_(BusSize, 0xFF),
        indexmap_(BusSize, 0xFFFF) {
      std::copy(std::begin(indexmap), std::end(indexmap), std::begin(indexmap_));
    }

    DataBus build(uint8_t reset_value) {
      return DataBus(BusSize, reset_value, std::move(backs_), std::move(backmap_), std::move(indexmap_));
    }

    template<typename B>
    void backend(B *back, std::initializer_list<IndexMapping<typename B::Register>> mapping) {
      static_assert(std::is_enum_v<typename B::Register>, "B::Registers must be an enum");
      for (const auto& v : mapping) {
        for (const auto& addr : v.addrs) {
          backmap_[addr] = backs_.size();
          indexmap_[addr] = static_cast<uint16_t>(v.index);
        }
      }

      backs_.push_back(back);
    }

    void indirect(std::initializer_list<uint16_t> indf_addrs, std::initializer_list<uint16_t> fsr_addrs) {
      for (const auto& addr : indf_addrs) {
        indexmap_[addr] = static_cast<uint16_t>(DataBus::Register::INDF);
      }
      alias(fsr_addrs);
    }

  private:
    void alias(std::initializer_list<uint16_t> addrs) {
      for (const auto& addr : addrs) {
        indexmap_[addr] = *addrs.begin();
      }
    }

  private:
    std::vector<internal::RegisterBackend*> backs_;
    std::vector<uint8_t> backmap_;
    std::vector<uint16_t> indexmap_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_internal_data_bus_h
