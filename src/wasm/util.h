#ifndef sim_wasm_util_h
#define sim_wasm_util_h

#include <string>

#include "../util/status.h"

namespace sim::wasm {

  inline std::string status_message(const sim::util::Status &status) {
    const auto &err = status.err();
    if (!err) return "";

    auto msg = err.message();
    auto context = status.context();

    if (context.empty()) return msg;

    return std::string(context) + ": " + msg;
  }

  struct StatusWrapper {
    StatusWrapper() = default;
    StatusWrapper(const sim::util::Status &status)
      : context(status.context()),
        ok(status.ok())
    {
      const auto &err = status.err();
      if (err) {
        auto msg = err.message();

        if (err) error = msg;
        else error = err.category().name();
      }
    }

    std::string error;
    std::string context;
    bool ok;
  };

}  // namespace sim::wasm

#endif  // sim_wasm_util_h
