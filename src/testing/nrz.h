#ifndef sim_testing_nrz_h
#define sim_testing_nrz_h

namespace sim::testing {

  /// A receiver for Non-Return-to-Zero data, such as that from UARTs.
  ///
  /// It assumes one start bit and one stop bit. The unit of ticks is
  /// unimportant, but must be consistent.
  class NRZReceiver {
  public:
    NRZReceiver(int num_data_bits, unsigned long bit_ticks)
      : num_data_bits_(num_data_bits),
        bit_ticks_(bit_ticks) {}

    bool is_receiving() const { return start_tick_ > 0; }
    std::u16string_view received() const { return data_; }

    void signal_changed(unsigned long tick, bool value) {
      if (!is_receiving() || tick > start_tick_ + (2 + num_data_bits_) * bit_ticks_) {
        if (!value) {
          start_tick_ = tick;
          bit_ = 0;
          buf_ = 0;
        }
      } else {
        int bits = (tick - prev_tick_ + bit_ticks_ / 2) / bit_ticks_;

        if (bit_ + bits > num_data_bits_) {
          if (value) start_tick_ = 0;
          bits = num_data_bits_ - bit_;
        }

        bit_ += bits;
        buf_ <<= bits;
        if (prev_value_) buf_ |= (1u << bits) - 1;

        if (bit_ == num_data_bits_)
          data_.push_back(buf_);
      }

      prev_tick_ = tick;
      prev_value_ = value;
    }

  private:
    const int num_data_bits_;
    const unsigned long bit_ticks_;

    unsigned long start_tick_ = 0;
    unsigned long prev_tick_ = 0;
    bool prev_value_;
    int bit_ = 0;
    uint16_t buf_ = 0;
    std::u16string data_;
  };

}  // namespace sim::testing

#endif  // sim_testing_nrz_h
