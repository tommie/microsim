include ../../Makefile.common

.PHONY: all
all: libtesting.a

.PHONY: clean
clean:
	rm -f *.o libtesting.a

libtesting.a: testing.o
	$(AR) cr $@ $^

testing.o: testing.cc testing.h ../util/status.h
