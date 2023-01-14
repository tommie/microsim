// See http://blog.think-async.com/2010/04/system-error-support-in-c0x-part-1.html
// for a five part series on how to make a file like this.

#ifndef sim_pic14_errors_h
#define sim_pic14_errors_h

#include <string>
#include <system_error>

namespace sim::pic14 {

  enum class Error {
    stack_overflow = 1,
    stack_underflow,
  };

  class ErrorCategoryImpl : public std::error_category {
  public:
    const char* name() const noexcept override;
    std::string message(int err_value) const override;
  };

  const std::error_category& pic14_category();

  inline std::error_code make_error_code(Error err_value) {
    return std::error_code(static_cast<int>(err_value), pic14_category());
  }

  inline std::error_condition make_error_condition(Error err_value) {
    return std::error_condition(static_cast<int>(err_value), pic14_category());
  }

}  // namespace sim::pic14

namespace std {

  template <>
  struct is_error_code_enum<sim::pic14::Error> : public true_type {};

}  // namespace std

#endif  // sim_pic14_errors_h
