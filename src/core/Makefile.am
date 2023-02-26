lib_LTLIBRARIES = libcore.la

libcore_la_SOURCES = \
	clock.cc \
	device.cc \
	ihex.cc \
	scheduler.cc \
	signal.cc \
	simtime.cc \
	simulation.cc

include_HEADERS = \
	clock.h \
	device.h \
	ihex.h \
	scheduler.h \
	signal.h \
	simtime.h \
	simulation.h

check_PROGRAMS = \
	clock_test \
	scheduler_test \
	signal_test

clock_test_SOURCES = clock_test.cc
scheduler_test_SOURCES = scheduler_test.cc
signal_test_SOURCES = signal_test.cc

test_libs = libcore.la ../util/libutil.la ../testing/libtesting.la
clock_test_LDADD = $(test_libs)
scheduler_test_LDADD = $(test_libs)
signal_test_LDADD = $(test_libs)

TESTS = $(check_PROGRAMS)
