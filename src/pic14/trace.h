#ifndef sim_pic14_trace_h
#define sim_pic14_trace_h

#include "internal/adc.h"
#include "internal/eprom.h"
#include "internal/eusart.h"
#include "internal/execution.h"
#include "internal/watchdog.h"

namespace sim::pic14 {

  using ADConversionDoneTraceEntry = internal::ADConversionDoneTraceEntry;
  using WroteEEDATATraceEntry = internal::WroteEEDATATraceEntry;
  using WroteProgramFlashTraceEntry = internal::WroteProgramFlashTraceEntry;
  using EUSARTDataTraceEntry = internal::EUSARTDataTraceEntry;
  using ExecutedTraceEntry = internal::ExecutedTraceEntry;
  using WatchDogClearedTraceEntry = internal::WatchDogClearedTraceEntry;
  using WatchDogTimedOutTraceEntry = internal::WatchDogTimedOutTraceEntry;

}  // namespace sim::pic14

#endif  // sim_pic14_trace_h
