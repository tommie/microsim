AR = emar
CXX = em++
LD = em++
CXXFLAGS = -Wall -std=c++20 -g -fsanitize=address
LDFLAGS = -sLLD_REPORT_UNDEFINED -sSTRICT_JS -sINCOMING_MODULE_JS_API=noExitRuntime,noInitialRun,onAbort,onRuntimeInitialized,print,printErr -fsanitize=address
LDADD = ../core/libcore.a

.PHONY: all all-r
all: libpic14.a all-r

all-r:
	$(MAKE) -C testdata all

.PHONY: check
check: tests.js all-r
	node $<

.PHONY: clean
clean:
	rm -f *.o libpic14.a tests.js tests.wasm

libpic14.a: errors.o execution.o data_bus.o interrupt.o nonvolatile.o port.o p16f88x.o
	$(AR) cr $@ $^

tests.js tests.wasm: tests.o libpic14.a $(LDADD)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -o $@ $^ $(LDADD)

errors.o: errors.cc errors.h

execution.o: execution.cc execution.h errors.h data_bus.h interrupt.h nonvolatile.h ../core/clock.h ../core/device.h ../core/scheduler.h ../util/status.h

data_bus.o: data_bus.cc data_bus.h

interrupt.o: interrupt.cc interrupt.h data_bus.h register.h

nonvolatile.o: nonvolatile.cc nonvolatile.h

port.o: port.cc port.h execution.h data_bus.h interrupt.h ../core/device.h

p16f88x.o: p16f88x.cc p16f88x.h execution.h data_bus.h interrupt.h nonvolatile.h port.h ../core/clock.h ../core/device.h ../core/scheduler.h ../util/status.h

tests.o: tests.cc p16f88x.h execution.h data_bus.h nonvolatile.h port.h ../core/device.h ../util/status.h ../testing/testing.h
tests.js: LDFLAGS += -sNODERAWFS
tests.js: LDADD += ../testing/libtesting.a
