#ifndef sim_core_status_h
#define sim_core_status_h

#include <cstddef>
#include <ostream>
#include <string>
#include <string_view>
#include <system_error>
#include <variant>

namespace sim::core {

  class Status {
  public:
    Status() = default;
    Status(std::error_code err, std::string_view context = "")
      : err_(err), context_(context) {}

    const std::error_code& err() const { return err_; }
    std::string_view context() const { return context_; }

    bool ok() const { return !static_cast<bool>(err_); }

  private:
    std::error_code err_;
    std::string context_;
  };

  inline std::ostream& operator <<(std::ostream &os, const Status &status) {
    os << status.err().message();
    if (!status.context().empty()) {
      os << ": " << status.context();
    }
    return os;
  }

  template<typename T>
  class StatusOr {
  public:
    StatusOr() = delete;
    StatusOr(T &&v) : v_(std::in_place_index<1>, std::forward(v)) {}
    StatusOr(const std::error_code &err)
      : v_(std::in_place_index<1>, err) {
      if (err) std::abort();
    }

    const T& value() const { return std::get<0>(v_); }
    T& value() { return std::get<0>(v_); }

    template<typename TT>
    const T& value_or(TT def) const { return v_.index() == 0 ? std::get<0>(v_) : def; }

    const Status& status() const { return std::get<1>(v_); }
    bool ok() const { return v_.index() == 0; }

  private:
    std::variant<T, Status> v_;
  };

}  // namespace sim::core

#endif  // sim_core_status_h
