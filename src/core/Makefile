AR = emar
CXX = em++
LD = em++
CXXFLAGS = -Wall -std=c++20 -g -fsanitize=address
LDFLAGS = -sLLD_REPORT_UNDEFINED -sSTRICT_JS -sINCOMING_MODULE_JS_API=noExitRuntime,noInitialRun,onAbort,onRuntimeInitialized,print,printErr -fsanitize=address

.PHONY: all
all: libcore.a

.PHONY: check
check: scheduler_test.js
	nodejs ./scheduler_test.js

.PHONY: clean
clean:
	rm -f *.o libcore.a

%_test.js: LDADD += libcore.a ../testing/libtesting.a

%.js %.wasm: %.o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDADD)

libcore.a: device.o ihex.o scheduler.o simulation.o
	$(AR) cr $@ $^

device.o: device.cc device.h ../util/status.h

ihex.o: ihex.cc ihex.h ../util/status.h

scheduler.o: scheduler.cc scheduler.h

scheduler_test.js: scheduler_test.o $(LDADD) libcore.a

scheduler_test.o: scheduler_test.cc scheduler.h
