#include "errors.h"

namespace sim::pic14 {

  const char* ErrorCategoryImpl::name() const noexcept {
    return "pic14";
  }

  std::string ErrorCategoryImpl::message(int err_value) const {
    switch (static_cast<Error>(err_value)) {
    case Error::stack_overflow: return "Stack overflow";
    case Error::stack_underflow: return "Stack underflow";
    default: return "Unknown PIC14 error";
    }
  }

  const std::error_category& pic14_category() {
    static ErrorCategoryImpl impl;

    return impl;
  }

}  // namespace sim::pic14
