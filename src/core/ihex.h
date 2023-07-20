#ifndef sim_core_ihex_h
#define sim_core_ihex_h

#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>

#include "../util/status.h"

namespace sim::core {

  // Parses a stream of Intel HEX and passes chunks of data to the
  // load function.
  sim::util::Status load_ihex(std::istream &in, std::function<sim::util::Status(uint32_t, const std::vector<uint8_t>&)> load);

}  // namespace sim::core

#endif  // sim_core_ihex_h
