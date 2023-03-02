#ifndef sim_testing_spi_h
#define sim_testing_spi_h

#include <string>
#include <string_view>

namespace sim::testing {

  /// An SPI master transmitter and receiver.
  class SPIMaster {
  public:
    SPIMaster(int num_data_bits, unsigned long bit_ticks)
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

  /// An SPI slave transmitter and receiver.
  class SPISlave {
  public:
    SPISlave(int num_data_bits, std::u16string_view data = {})
      : num_data_bits_(num_data_bits),
        data_(data) {}

    SPISlave(SPISlave&&) = default;
    SPISlave& operator =(SPISlave&&) = default;

    bool empty() const { return data_.empty() && bit_ == 0; }
    std::u16string_view data() const { return data_; }

    /// Called when either the clock or data signal changed. If an
    /// input is in tri-state, the value should be 0.5. Returns the
    /// value to set the data pin to, or 0.5 if none.
    double signal_changed(double ck, double dt) {
      double v = 0.5;

      if (dt >= 0.4 && dt < 0.6) {
        // The master wants to receive.
        if (prev_ck_ < 0.5 && ck > 0.5) {
          if (bit_ == 0) {
            if (data_.empty()) {
              prev_ck_ = ck;
              return 0.5;
            }

            buf_ = data_[0];
            data_.erase(0, 1);
            ++bit_;
          } else {
            buf_ >>= 1;
            if (++bit_ == num_data_bits_)
              bit_ = 0;
          }
        }

        v = buf_ & 1;
      } else {
        // The master is sending.
        if (prev_ck_ > 0.5 && ck < 0.5) {
          buf_ >>= 1;
          buf_ |= (dt >= 0.5 ? 0x100 : 0);
          ++bit_;

          if (bit_ == num_data_bits_) {
            buf_ >>= (1 + 9 - bit_);
            data_.append(1, buf_);
            bit_ = 0;
            buf_ = 0;
          }
        }
      }

      prev_ck_ = ck;

      return v;
    }

  private:
    const int num_data_bits_;
    std::u16string data_;

    int bit_ = 0;
    uint16_t buf_ = 0;

    double prev_ck_ = 0;
  };

}  // namespace sim::testing

#endif  // sim_testing_spi_h
