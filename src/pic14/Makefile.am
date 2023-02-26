SUBDIRS = testdata

lib_LTLIBRARIES = libpic14.la

libpic14_la_SOURCES = \
	core.cc \
	data_bus.cc \
	errors.cc \
	execution.cc \
	interrupt.cc \
	p16f88x.cc \
	port.cc

include_HEADERS = \
	core.h \
	data_bus.h \
	errors.h \
	execution.h \
	interrupt.h \
	nonvolatile.h \
	p16f88x.h \
	pin.h \
	port.h \
	register.h

check_PROGRAMS = p16f88x_test

p16f88x_test_SOURCES = p16f88x_test.cc

test_libs = libpic14.la ../core/libcore.la ../util/libutil.la ../testing/libtesting.la
p16f88x_test_LDADD = $(test_libs)

TESTS = $(check_PROGRAMS)