#ifndef sim_pic14_eusart_h
#define sim_pic14_eusart_h

#include <cstdint>
#include <variant>

#include "../core/clock.h"
#include "../core/device.h"
#include "../core/scheduler.h"
#include "data_bus.h"
#include "execution.h"
#include "interrupt.h"
#include "pin.h"
#include "register.h"

namespace sim::pic14::internal {

  template<typename Backend>
  class BaudCtlRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0x1B;

  public:
    enum Bits {
      ABDEN, WUE, BRG16 = 3, SCKP, RCIDL = 6, ABDOVF,
    };

    explicit BaudCtlRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool abden() const { return Base::template bit<ABDEN>(); }
    bool wue() const { return Base::template bit<WUE>(); }
    bool brg16() const { return Base::template bit<BRG16>(); }
    bool sckp() const { return Base::template bit<SCKP>(); }
    bool rcidl() const { return Base::template bit<RCIDL>(); }
    bool abdovf() const { return Base::template bit<ABDOVF>(); }

    void reset() { Base::write(0x40); }
  };

  template<typename Backend>
  class RCStaRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0xF8;

  public:
    enum Bits {
      RX9D, OERR, FERR, ADDEN, CREN, SREN, RX9, SPEN,
    };

    explicit RCStaRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool rx9d() const { return Base::template bit<RX9D>(); }
    void set_rx9d(bool v) { Base::template set_bit<RX9D>(v); }
    bool oerr() const { return Base::template bit<OERR>(); }
    void set_oerr(bool v) { Base::template set_bit<OERR>(v); }
    bool ferr() const { return Base::template bit<FERR>(); }
    void set_ferr(bool v) { Base::template set_bit<FERR>(v); }
    bool adden() const { return Base::template bit<ADDEN>(); }
    bool cren() const { return Base::template bit<CREN>(); }
    bool sren() const { return Base::template bit<SREN>(); }
    bool rx9() const { return Base::template bit<RX9>(); }
    bool spen() const { return Base::template bit<SPEN>(); }

    void reset() { Base::write(0); }
  };

  template<typename Backend>
  class TXStaRegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0xFD;

  public:
    enum Bits {
      TX9D, TRMT, BRGH, SENDB, SYNC, TXEN, TX9, CSRC,
    };

    explicit TXStaRegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    uint8_t tx9d() const { return Base::template bit_field<TX9D, 1>(); }
    bool trmt() const { return Base::template bit<TRMT>(); }
    void set_trmt(bool v) { Base::template set_bit<TRMT>(v); }
    bool brgh() const { return Base::template bit<BRGH>(); }
    bool sendb() const { return Base::template bit<SENDB>(); }
    bool sync() const { return Base::template bit<SYNC>(); }
    bool txen() const { return Base::template bit<TXEN>(); }
    bool tx9() const { return Base::template bit<TX9>(); }
    bool csrc() const { return Base::template bit<CSRC>(); }

    void update_tx_done() { Base::template update_masks<1u << TRMT, 1u << SENDB, 0>(); }

    void reset() { Base::write(0x02); }
  };

  class EUSART : public RegisterBackend, public sim::core::Schedulable {
    class ResetImpl {
    public:
      void rc_pin_changed(bool v);
      void write_register(uint16_t addr, uint8_t value);
      sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit);
    };

    class AsyncImpl {
    public:
      explicit AsyncImpl(EUSART *eusart);

      void rc_pin_changed(bool v);
      void write_register(uint16_t addr, uint8_t value);
      sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit);

    private:
      sim::core::TimePoint advance_rc();
      sim::core::TimePoint advance_tx();

      void load_tsr();

    private:
      EUSART *eusart_;

      uint16_t tsr_;
      int tsr_bits_ = 0;
      bool tx_reg_valid_ = false;

      uint16_t rsr_;
      int rsr_bits_ = 0;
    };

    class SyncImpl {
    public:
      explicit SyncImpl(EUSART *eusart);

      void rc_pin_changed(bool v);
      void write_register(uint16_t addr, uint8_t value);
      sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit);

    private:
      EUSART *eusart_;
    };

  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using RCStaReg = RCStaRegBase<MultiRegisterBackend<EUSART, 0x18>>;
    using TXReg = MultiRegisterBackend<EUSART, 0x19>;
    using RCReg = MultiRegisterBackend<EUSART, 0x1A>;
    using TXStaReg = TXStaRegBase<MultiRegisterBackend<EUSART, 0x98>>;
    using SPBRGReg = MultiRegisterBackend<EUSART, 0x99>;
    using SPBRGHReg = MultiRegisterBackend<EUSART, 0x9A>;
    using BaudCtlReg = BaudCtlRegBase<MultiRegisterBackend<EUSART, 0x187>>;

    EUSART(sim::core::DeviceListener *listener, sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralLevelSignal &&rc_interrupt, InterruptMux::MaskablePeripheralLevelSignal &&tx_interrupt);

    void reset();

    RCStaReg rcsta_reg() { return RCStaReg(MultiRegisterBackend<EUSART, 0x18>(this)); }
    TXStaReg txsta_reg() { return TXStaReg(MultiRegisterBackend<EUSART, 0x98>(this)); }
    TXReg tx_reg() { return TXReg(this); }
    RCReg rc_reg() { return RCReg(this); }
    SPBRGReg spbrg_reg() { return SPBRGReg(this); }
    SPBRGHReg spbrgh_reg() { return SPBRGHReg(this); }
    BaudCtlReg baudctl_reg() { return BaudCtlReg(MultiRegisterBackend<EUSART, 0x187>(this)); }

    InputPin& rc_pin() { return rc_pin_; }
    OutputPin& tx_pin() { return tx_pin_; }

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

    void fosc_changed() { schedule_immediately(); }

  private:
    void update_impl();
    void update_bit_duration();

    void push_rcreg(uint16_t v);
    uint8_t pop_rcreg();
    void set_tx_pin(bool v);

  private:
    sim::core::DeviceListener *listener_;
    sim::core::ClockModifierView rc_fosc_;
    sim::core::ClockModifierView tx_fosc_;
    InterruptMux::MaskablePeripheralLevelSignal rc_interrupt_;
    InterruptMux::MaskablePeripheralLevelSignal tx_interrupt_;

    InputPin rc_pin_;
    OutputPin tx_pin_;

    std::variant<ResetImpl, AsyncImpl, SyncImpl> impl_;

    RCStaRegBase<SingleRegisterBackend<uint8_t>> rcsta_reg_;
    TXStaRegBase<SingleRegisterBackend<uint8_t>> txsta_reg_;
    SingleRegisterBackend<uint8_t> tx_reg_;
    SingleRegisterBackend<uint8_t> spbrg_reg_;
    SingleRegisterBackend<uint8_t> spbrgh_reg_;
    BaudCtlRegBase<SingleRegisterBackend<uint8_t>> baudctl_reg_;

    std::array<uint16_t, 2> rcreg_fifo_;
    std::size_t rcreg_fifo_head_ = 0;
    std::size_t rcreg_fifo_tail_ = 0;

    sim::core::Clock::duration bit_duration_;
  };

}  // namespace sim::pic14::internal

#endif  // sim_pic14_eusart_h
