include ../../Makefile.common

.PHONY: all
all: libcore.a

.PHONY: check
check: clock_test$(EXEEXT) scheduler_test$(EXEEXT) signal_test$(EXEEXT)
	$(TESTRUNNER) ./clock_test$(EXEEXT)
	$(TESTRUNNER) ./scheduler_test$(EXEEXT)
	$(TESTRUNNER) ./signal_test$(EXEEXT)

.PHONY: clean
clean:
	rm -f *.o libcore.a clock_test$(EXEEXT) scheduler_test$(EXEEXT) signal_test$(EXEEXT)


libcore.a: device.o ihex.o scheduler.o signal.o simulation.o
	$(AR) cr $@ $^

device.o: device.cc device.h scheduler.h simulation.h ../util/status.h

ihex.o: ihex.cc ihex.h ../util/status.h

scheduler.o: scheduler.cc scheduler.h

signal.o: signal.cc signal.h

simulation.o: simulation.cc simulation.h scheduler.h ../util/status.h

clock_test$(EXEEXT): clock_test.o $(LDADD) libcore.a

clock_test.o: clock_test.cc clock.h

scheduler_test$(EXEEXT): scheduler_test.o $(LDADD) libcore.a

scheduler_test.o: scheduler_test.cc scheduler.h

signal_test$(EXEEXT): signal_test.o $(LDADD) libcore.a

signal_test.o: signal_test.cc signal.h
