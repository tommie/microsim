AR = emar
CXX = em++
CXXFLAGS = -Wall -std=c++20 -g -fsanitize=address

.PHONY: all
all: libtesting.a

.PHONY: clean
clean:
	rm -f *.o libtesting.a

libtesting.a: testing.o
	$(AR) cr $@ $^

testing.o: testing.cc testing.h ../core/status.h
