#include <memory>
#include <optional>
#include <vector>

#include <emscripten/bind.h>

#include "../core/device.h"
#include "../pic14/p16f88x.h"
#include "../pic14/trace.h"
#include "util.h"

using namespace emscripten;
using namespace sim::wasm;

namespace {

  // This wrapper works around that RAII doesn't work from JS. We need
  // to release ICSP explicitly, so adding a release function.
  //
  // Another approach is to provide a withICSP(callback), but this is
  // more versatile.
  class Pic14ICSPWrapper {
  public:
    explicit Pic14ICSPWrapper(sim::pic14::ICSP &&icsp)
      : icsp_(std::move(icsp)) {}

    void release() {
      icsp_.reset();
    }

    StatusWrapper loadProgram(uint16_t addr, val data) {
      return StatusWrapper(icsp_->load_program(addr, convertJSArrayToNumberVector<uint8_t>(data)));
    }

    StatusWrapper loadData(uint16_t addr, val data) {
      return StatusWrapper(icsp_->load_data(addr, convertJSArrayToNumberVector<uint8_t>(data)));
    }

  private:
    std::optional<sim::pic14::ICSP> icsp_;
  };

  template<typename Proc>
  std::unique_ptr<Pic14ICSPWrapper> enterICSP(Proc &self) {
    return std::make_unique<Pic14ICSPWrapper>(self.enter_icsp());
  }

  EMSCRIPTEN_BINDINGS(pic14) {

    enum_<sim::pic14::EUSARTDataTraceEntry::Mode>("EUSARTDataTraceEntryMode")
      .value("ASYNC_TRANSMIT", sim::pic14::EUSARTDataTraceEntry::Mode::ASYNC_TRANSMIT)
      .value("ASYNC_RECEIVED", sim::pic14::EUSARTDataTraceEntry::Mode::ASYNC_RECEIVED)
      .value("SYNC_MASTER_TRANSMIT", sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_MASTER_TRANSMIT)
      .value("SYNC_MASTER_RECEIVED", sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_MASTER_RECEIVED)
      .value("SYNC_SLAVE_TRANSMIT", sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_SLAVE_TRANSMIT)
      .value("SYNC_SLAVE_RECEIVED", sim::pic14::EUSARTDataTraceEntry::Mode::SYNC_SLAVE_RECEIVED);

    class_<Pic14ICSPWrapper>("Pic14ICSP")
      .function("release", &Pic14ICSPWrapper::release)
      .function("loadProgram", &Pic14ICSPWrapper::loadProgram)
      .function("loadData", &Pic14ICSPWrapper::loadData);

    class_<sim::pic14::P16F887, base<sim::core::Device>>("P16F887")
      .constructor<sim::core::DeviceListener*, sim::core::Clock*>()
      .function("enterICSP", &enterICSP<sim::pic14::P16F887>)
      .property("sleeping", &sim::pic14::P16F887::is_sleeping);

  }

}  // namespace
