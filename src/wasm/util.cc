#include "util.h"

#include <string>

#include <emscripten/bind.h>

using namespace emscripten;

namespace sim::wasm {

  EMSCRIPTEN_BINDINGS(util) {

    value_object<StatusWrapper>("Status")
      .field("error", &StatusWrapper::error)
      .field("context", &StatusWrapper::context)
      .field("ok", &StatusWrapper::ok);

    register_vector<std::string>("Strings");

  }

}  // namespace sim::wasm
