#ifndef sim_pic14_adc_h
#define sim_pic14_adc_h

#include <cstdint>
#include <vector>

#include "../core/clock.h"
#include "../core/device.h"
#include "../core/scheduler.h"
#include "../core/trace.h"
#include "data_bus.h"
#include "execution.h"
#include "interrupt.h"
#include "register.h"

namespace sim::pic14::internal {

  template<typename Backend>
  class ADCon0RegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

  public:
    enum Bits {
      ADON, GO, CHS0, CHS1, CHS2, CHS3, ADCS0, ADCS1,
    };

    explicit ADCon0RegBase(Backend backend) : BitRegister<Backend>(backend) {}

    bool adon() const { return Base::template bit<ADON>(); }
    bool go() const { return Base::template bit<GO>(); }
    void set_go(bool v) { Base::template set_bit<GO>(v); }
    uint8_t chs() const { return Base::template bit_field<CHS0, 4>(); }
    uint8_t adcs() const { return Base::template bit_field<ADCS0, 2>(); }

    void reset() { Base::write(0); }
  };

  template<typename Backend>
  class ADCon1RegBase : public BitRegister<Backend> {
    using Base = BitRegister<Backend>;

    static constexpr uint8_t WRITE_MASK = 0xB0;

  public:
    enum Bits {
      VCFG0 = 4, VCFG1, ADFM = 7,
    };

    explicit ADCon1RegBase(Backend backend) : BitRegister<Backend>(backend) {}

    void write_masked(uint8_t v) { Base::template set_masked<WRITE_MASK>(v); }

    bool vcfg0() const { return Base::template bit<VCFG0>(); }
    bool vcfg1() const { return Base::template bit<VCFG1>(); }
    bool adfm() const { return Base::template bit<ADFM>(); }

    void reset() { Base::write(0); }
  };

  /// A simple analog input pin.
  class AnalogInputPin : public sim::core::Pin {
  public:
    explicit AnalogInputPin(double v = 0.5) : value_(v) {}

    double value() const override { return 0; }
    double resistance() const override { return 1; }

    void set_external(double v) override {
      value_ = v;
    }

    double external() const { return value_; }

  private:
    double value_;
  };

  class ADConverter : public RegisterBackend, public sim::core::Schedulable {
  public:
    using RegisterType = uint8_t;
    using RegisterAddressType = uint16_t;
    using ADResHReg = MultiRegisterBackend<ADConverter, 0x1E>;
    using ADCon0Reg = ADCon0RegBase<MultiRegisterBackend<ADConverter, 0x1F>>;
    using ADResLReg = MultiRegisterBackend<ADConverter, 0x9E>;
    using ADCon1Reg = ADCon1RegBase<MultiRegisterBackend<ADConverter, 0x9F>>;

    ADConverter(sim::core::ClockModifier *fosc, InterruptMux::MaskablePeripheralEdgeSignal &&interrupt);

    void reset();

    ADCon0Reg adcon0_reg() { return ADCon0Reg(MultiRegisterBackend<ADConverter, 0x1F>(this)); }
    ADCon1Reg adcon1_reg() { return ADCon1Reg(MultiRegisterBackend<ADConverter, 0x9F>(this)); }
    ADResLReg adresl_reg() { return ADResLReg(this); }
    ADResHReg adresh_reg() { return ADResHReg(this); }

    const std::vector<AnalogInputPin>& input_pins() const { return input_pins_; }
    std::vector<AnalogInputPin>& input_pins() { return input_pins_; }
    AnalogInputPin& vref_neg_pin() { return input_pins_[2]; }
    AnalogInputPin& vref_pos_pin() { return input_pins_[3]; }
    AnalogInputPin& avdd_pin() { return avdd_pin_; }

    std::vector<sim::core::Clock*> clock_sources() { return { &frc_ }; }

    uint8_t read_register(uint16_t addr) override;
    void write_register(uint16_t addr, uint8_t value) override;

    sim::core::Advancement advance_to(const sim::core::AdvancementLimit &limit) override;

    void fosc_changed() { schedule_immediately(); }

  private:
    double sample() const;

  private:
    sim::core::ClockModifierView fosc_;
    InterruptMux::MaskablePeripheralEdgeSignal interrupt_;

    std::vector<AnalogInputPin> input_pins_;
    AnalogInputPin avdd_pin_;

    sim::core::Clock frc_;
    sim::core::ClockView frc_view_;

    ADCon0RegBase<SingleRegisterBackend<uint8_t>> adcon0_reg_;
    ADCon1RegBase<SingleRegisterBackend<uint8_t>> adcon1_reg_;
    SingleRegisterBackend<uint8_t> adresl_reg_;
    SingleRegisterBackend<uint8_t> adresh_reg_;
  };

}  // namespace sim::pic14::internal

namespace sim::pic14 {

  class ADConversionDoneTraceEntry : public sim::util::TraceEntryBase {
  public:
    static const sim::util::TraceEntryType<ADConversionDoneTraceEntry> TYPE;

    explicit ADConversionDoneTraceEntry() {}
  };

}  // namespace sim::pic14

#endif  // sim_pic14_adc_h
