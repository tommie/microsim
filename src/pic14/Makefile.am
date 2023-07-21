include ../common.mk

SUBDIRS = testdata

lib_LTLIBRARIES = libpic14.la

internal_sources = \
	internal/adc.cc \
	internal/core.cc \
	internal/data_bus.cc \
	internal/eprom.cc \
	internal/eusart.cc \
	internal/execution.cc \
	internal/extint.cc \
	internal/interrupt.cc \
	internal/port.cc \
	internal/timer0.cc \
	internal/ulpwu.cc \
	internal/watchdog.cc

internal_headers = \
	internal/adc.h \
	internal/core.h \
	internal/data_bus.h \
	internal/eprom.h \
	internal/eusart.cc \
	internal/execution.h \
	internal/extint.h \
	internal/interrupt.h \
	internal/nonvolatile.h \
	internal/pin.h \
	internal/port.h \
	internal/register.h \
	internal/timer0.h \
	internal/ulpwu.h \
	internal/watchdog.h \
	trace.h

device_sources = \
	p16f88x.cc

device_headers = \
	p16f88x.h

libpic14_la_SOURCES = \
	$(internal_sources) \
	$(device_sources) \
	errors.cc

include_HEADERS = \
	$(internal_headers) \
	$(device_headers) \
	errors.h

check_PROGRAMS = p16f88x_test

p16f88x_test_SOURCES = p16f88x_test.cc

test_libs = libpic14.la ../core/libcore.la ../util/libutil.la ../testing/libtesting.la
p16f88x_test_LDADD = $(test_libs)

TESTS = $(check_PROGRAMS)
