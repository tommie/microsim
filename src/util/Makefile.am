lib_LTLIBRARIES = libutil.la

libutil_la_SOURCES = \
	trace.cc

include_HEADERS = \
	registry.h \
	status.h \
	trace.h

check_PROGRAMS = \
	trace_test

trace_test_SOURCES = trace_test.cc

test_libs = libutil.la ../testing/libtesting.la
trace_test_LDADD = $(test_libs)

TESTS = $(check_PROGRAMS)
