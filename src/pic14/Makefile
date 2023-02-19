include ../../Makefile.common

LDADD = ../core/libcore.a

.PHONY: all all-r
all: libpic14.a all-r

all-r:
	$(MAKE) -C testdata all

.PHONY: check
check: tests$(EXEEXT) all-r
	$(TESTRUNNER) ./$<

.PHONY: clean
clean:
	rm -f *.o libpic14.a tests$(EXEEXT)
ifeq ($(host),wasm32-unknown-unknown)
	rm -f tests.wasm
endif

libpic14.a: core.o errors.o execution.o data_bus.o interrupt.o port.o p16f88x.o
	$(AR) cr $@ $^

tests$(EXEEXT): tests.o libpic14.a $(LDADD)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -o $@ $^ $(LDADD)

core.o: core.cc core.h data_bus.h nonvolatile.h pin.h register.h ../core/signal.h ../util/status.h

errors.o: errors.cc errors.h

execution.o: execution.cc execution.h core.h errors.h data_bus.h interrupt.h nonvolatile.h pin.h ../core/clock.h ../core/device.h ../core/scheduler.h ../core/signal.h ../util/status.h

data_bus.o: data_bus.cc data_bus.h

interrupt.o: interrupt.cc interrupt.h core.h data_bus.h pin.h register.h ../core/signal.h

port.o: port.cc port.h execution.h core.h data_bus.h interrupt.h pin.h ../core/device.h ../core/signal.h

p16f88x.o: p16f88x.cc p16f88x.h core.h execution.h data_bus.h interrupt.h nonvolatile.h pin.h port.h register.h ../core/clock.h ../core/device.h ../core/scheduler.h ../core/signal.h ../util/status.h


tests.o: tests.cc p16f88x.h core.h execution.h data_bus.h interrupt.h nonvolatile.h pin.h port.h ../core/device.h ../core/signal.h ../util/status.h ../testing/testing.h

tests$(EXEEXT): LDADD += ../testing/libtesting.a
ifeq ($(host),wasm32-unknown-unknown)
tests$(EXEEXT): LDFLAGS += -sNODERAWFS
endif
