#include "ihex.h"

#include <array>

namespace sim::core {

  namespace {

    static int parse_hex(char c) {
      if (c < '0') return -1;
      if (c <= '9') return c - '0';
      if (c < 'A') return -1;
      if (c <= 'F') return c - 'A' + 10;
      if (c < 'a') return -1;
      if (c <= 'f') return c - 'a' + 10;
      return -1;
    }

    static int parse_hex(char c1, char c2) {
      int i1 = parse_hex(c1);
      if (i1 < 0) return -1;
      return (i1 << 4) | parse_hex(c2);
    }

  }

  sim::util::Status load_ihex(std::istream &in, std::function<sim::util::Status(uint32_t, const std::vector<uint8_t>&)> load) {
    for (std::array<char, 256> buf; in.getline(&buf[0], buf.size());) {
      std::string line(&buf[0]);

      if (line.size() == 0) {
        continue;
      }

      if (line.size() < 1 + 2 + 4 + 2 + 2) {
        return std::make_error_code(std::errc::invalid_argument);
      }

      if (line[0] != ':') {
        return std::make_error_code(std::errc::invalid_argument);
      }

      size_t count = parse_hex(line[1], line[2]);
      if (count < 0 || line.size() < 1 + 2 + 4 + 2 + 2 * count + 2)
        return std::make_error_code(std::errc::invalid_argument);

      int addr = parse_hex(line[3], line[4]);
      if (addr < 0) {
        return std::make_error_code(std::errc::invalid_argument);
      }

      addr = (addr << 8) | parse_hex(line[5], line[6]);
      int rtype = parse_hex(line[7], line[8]);
      unsigned int csum = parse_hex(line[1 + 2 + 4 + 2 + 2 * count], line[1 + 2 + 4 + 2 + 2 * count + 1]);
      if (addr < 0 || rtype < 0 || csum < 0) {
        return std::make_error_code(std::errc::invalid_argument);
      }

      std::vector<uint8_t> data(count, 0);
      for (size_t i = 0, j = 1 + 2 + 4 + 2; i < count; ++i, j += 2) {
        data[i] = parse_hex(line[j], line[j + 1]);
      }

      unsigned int csum_real = 0;
      for (size_t i = 1; i < line.size() - 2; i += 2) {
        csum_real += parse_hex(line[i], line[i + 1]);
      }

      if (static_cast<uint8_t>(csum + csum_real) != 0) {
        return std::make_error_code(std::errc::invalid_argument);
      }

      switch (rtype) {
      case 0:
        if (auto status = load(static_cast<uint16_t>(addr / 2), data); !status.ok()) {
          return status;
        }
        break;

      case 1:
        return sim::util::Status();

      case 4:
        // Ignored.
        break;

      default:
        return std::make_error_code(std::errc::invalid_argument);
      }
    }

    return sim::util::Status(std::make_error_code(std::errc::invalid_argument), "missing EOF record");
  }

}  // namespace sim::core
