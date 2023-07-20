#ifndef sim_testing_spi_h
#define sim_testing_spi_h

#include <string>
#include <string_view>
#include <tuple>

namespace sim::testing {

  /// An SPI master transmitter and receiver. It can also handle LIN Bus,
  /// where there is only a single data line.
  class SPIMaster {
  public:
    SPIMaster(SPIMaster&&) = default;
    SPIMaster& operator =(SPIMaster&&) = default;

    /// Constructs a new master. If `data[i]` has the highest bit set,
    /// the master will receive data from the slave.
    SPIMaster(int num_data_bits, unsigned long bit_ticks, std::u16string_view data = {})
      : num_data_bits_(num_data_bits),
        bit_ticks_(bit_ticks),
        data_(data) {}

    bool empty() const { return bit_count_ == 8 * data_.size() && bit_ == 0; }
    std::u16string_view data() const { return data_; }

    std::tuple<unsigned long, double, double> next_signal_change(unsigned long tick, double dt) {
      if (bit_count_ >= 8 * data_.size()) {
        return {};
      }

      if (tick >= bit_count_ * bit_ticks_ + bit_ticks_ / 2) {
        if ((data_[bit_count_ / 8] & 0x8000) != 0) {
          // The master receives.
          buf_ >>= 1;
          buf_ |= (dt >= 0.5 ? 0x100 : 0);
          ++bit_;

          if (bit_ == num_data_bits_) {
            buf_ >>= 9 - bit_;
            data_[bit_count_ / 8] = buf_;
          }

          dt_ = 0.5;
        }

        if (bit_ == num_data_bits_) {
          bit_ = 0;
          buf_ = 0;
        }

        ++bit_count_;
        ck_ = 0;
        return {bit_count_ * bit_ticks_, ck_, dt_};
      } else if (tick >= bit_count_ * bit_ticks_) {
        if ((data_[bit_count_ / 8] & 0x8000) == 0) {
          // The master transmits.
          if (bit_ == 0) {
            buf_ = data_[bit_count_ / 8];
          } else {
            buf_ >>= 1;
          }

          ++bit_;
          dt_ = buf_ & 1;
        }

        ck_ = 1;
        return {bit_count_ * bit_ticks_ + bit_ticks_ / 2, ck_, dt_};
      }

      return {bit_count_ * bit_ticks_, ck_, dt_};
    }

  private:
    int num_data_bits_;
    unsigned long bit_ticks_;

    int bit_ = 0;
    unsigned int bit_count_ = 0;
    uint16_t buf_ = 0;
    std::u16string data_;

    double dt_ = 0;
    double ck_ = 0;
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
