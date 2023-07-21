include ../common.mk

www_PROGRAMS = microsim
www_SCRIPTS = microsim.wasm
www_DATA = microsim.d.ts microsim.wasm.map package.json
CLEANFILES = $(www_SCRIPTS) $(www_DATA)

noinst_HEADERS = core.h util.h

microsim_SOURCES = \
	core.cc \
	pic14.cc \
	trace.cc \
	util.cc
microsim_LDADD = \
	../pic14/libpic14.la \
	../core/libcore.la \
	../util/libutil.la \
	-lembind

# emcc warns about "running limited binaryen optimizations because DWARF info requested"
# if -gsource-map is used, and -O0 doesn't stop it.
#
# Gotchas:
#
#   -sMINIMAL_RUNTIME=2 -- uses require() for Node even with MODULARIZE
microsim_LDFLAGS = \
	$(AM_LDFLAGS) \
	-sLLD_REPORT_UNDEFINED \
	-sIGNORE_MISSING_MAIN=1 \
	-sINVOKE_RUN=0 \
	-sEXIT_RUNTIME=0 \
	-sINCOMING_MODULE_JS_API=onAbort,onRuntimeInitialized,print,printErr \
	-sALLOW_TABLE_GROWTH=1 \
	-sFILESYSTEM=0 \
	-sNODERAWFS=0

# https://www.gnu.org/software//automake/manual/html_node/Multiple-Outputs.html
microsim.wasm: microsim$(EXEEXT)
microsim.wasm.map: microsim$(EXEEXT)

package.json: package.json.in $(top_builddir)/config.status
	wwwdir=$(wwwdir) ;\
	sed -e "s|@PACKAGE_TARNAME[@]|$(PACKAGE_TARNAME)|g" \
		-e "s|@PACKAGE_VERSION[@]|$(PACKAGE_VERSION)|g" \
		-e "s|@PACKAGE_BUGREPORT[@]|$(PACKAGE_BUGREPORT)|g" \
		-e "s|@wwwdir[@]|$${wwwdir//\/\//\/}|g" $< >$@.tmp ||\
		{ status=$? ; rm -f $@.tmp ; exit $status ; }
	mv $@.tmp $@

check-local:
	[ "x$(builddir)" = "x$(srcdir)" ] || cp $(srcdir)/microsim_test.mjs $(builddir)/microsim_test.mjs
	node --test ./microsim_test.mjs
