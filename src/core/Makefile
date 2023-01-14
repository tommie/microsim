AR = emar
CXX = em++
CXXFLAGS = -Wall -std=c++20 -g -fsanitize=address

.PHONY: all
all: libcore.a

.PHONY: clean
clean:
	rm -f *.o libcore.a

libcore.a: device.o ihex.o scheduler.o simulation.o
	$(AR) cr $@ $^

device.o: device.cc device.h ../util/status.h

ihex.o: ihex.cc ihex.h ../util/status.h
