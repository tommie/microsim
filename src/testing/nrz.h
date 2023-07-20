#ifndef sim_testing_nrz_h
#define sim_testing_nrz_h

#include <vector>

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
    const std::vector<uint16_t>& received() const { return data_; }

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
    std::vector<uint16_t> data_;
  };

  /// A transmitter for Non-Return-to-Zero data, such as that to UARTs.
  ///
  /// It assumes one start bit and one stop bit. The unit of ticks is
  /// unimportant, but must be consistent.
  class NRZTransmitter {
  public:
    NRZTransmitter(int num_data_bits, unsigned long bit_ticks, const std::vector<uint16_t> &data)
      : num_data_bits_(num_data_bits),
        bit_ticks_(bit_ticks),
        data_(data) {}

    NRZTransmitter(NRZTransmitter&&) = default;
    NRZTransmitter& operator =(NRZTransmitter&&) = default;

    bool empty() const { return data_.empty() && bit_ == 0; }

    /// Returns (next_tick, value), where the signal should be `value`
    /// until `next_tick`. At `next_tick`, this function should be
    /// called to figure out what to do. If `next_tick` is zero, the
    /// transmitter is done.
    std::pair<unsigned long, bool> next_signal_change() {
      if (bit_ == 0) {
        buf_ = data_[0];
        data_.erase(std::begin(data_));
        buf_ <<= 1;
        buf_ |= 1u << (1 + num_data_bits_);
        tick_ = 0;
      } else {
        buf_ >>= 1;
        tick_ += bit_ticks_;
      }

      ++bit_;

      if (bit_ == 2 + num_data_bits_) {
        bit_ = 0;
        return {0, true};
      }

      return {
        tick_ + bit_ticks_,
        (buf_ & 1) != 0,
      };
    }

  private:
    const int num_data_bits_;
    const unsigned long bit_ticks_;
    std::vector<uint16_t> data_;

    int bit_ = 0;
    uint16_t buf_;
    unsigned long tick_;
  };

}  // namespace sim::testing

#endif  // sim_testing_nrz_h
