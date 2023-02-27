SUBDIRS = testdata

lib_LTLIBRARIES = libpic14.la

libpic14_la_SOURCES = \
	adc.cc \
	core.cc \
	data_bus.cc \
	eprom.cc \
	errors.cc \
	execution.cc \
	extint.cc \
	interrupt.cc \
	p16f88x.cc \
	port.cc \
	timer0.cc \
	ulpwu.cc \
	watchdog.cc

include_HEADERS = \
	adc.h \
	core.h \
	data_bus.h \
	eprom.h \
	errors.h \
	execution.h \
	extint.h \
	interrupt.h \
	nonvolatile.h \
	p16f88x.h \
	pin.h \
	port.h \
	register.h \
	timer0.h \
	ulpwu.h \
	watchdog.h

check_PROGRAMS = p16f88x_test

p16f88x_test_SOURCES = p16f88x_test.cc

test_libs = libpic14.la ../core/libcore.la ../util/libutil.la ../testing/libtesting.la
p16f88x_test_LDADD = $(test_libs)

TESTS = $(check_PROGRAMS)
