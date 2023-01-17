AR = emar
CXX = em++
LD = em++
CXXFLAGS = -Wall -std=c++20 -g -fsanitize=address
LDFLAGS = -sLLD_REPORT_UNDEFINED -sSTRICT_JS -sINCOMING_MODULE_JS_API=noExitRuntime,noInitialRun,onAbort,onRuntimeInitialized,print,printErr -fsanitize=address

.PHONY: all
all: libcore.a

.PHONY: check
check: clock_test.js scheduler_test.js signal_test.js
	nodejs ./clock_test.js
	nodejs ./scheduler_test.js
	nodejs ./signal_test.js

.PHONY: clean
clean:
	rm -f *.o libcore.a

%_test.js: LDADD += libcore.a ../testing/libtesting.a

%.js %.wasm: %.o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDADD)

libcore.a: device.o ihex.o scheduler.o signal.o simulation.o
	$(AR) cr $@ $^

device.o: device.cc device.h scheduler.h simulation.h ../util/status.h

ihex.o: ihex.cc ihex.h ../util/status.h

scheduler.o: scheduler.cc scheduler.h

signal.o: signal.cc signal.h

simulation.o: simulation.cc simulation.h scheduler.h ../util/status.h

clock_test.js: clock_test.o $(LDADD) libcore.a

clock_test.o: clock_test.cc clock.h

scheduler_test.js: scheduler_test.o $(LDADD) libcore.a

scheduler_test.o: scheduler_test.cc scheduler.h

signal_test.js: signal_test.o $(LDADD) libcore.a

signal_test.o: signal_test.cc signal.h
