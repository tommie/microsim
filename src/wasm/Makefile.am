include ../common.mk

bin_PROGRAMS = microsim
bin_SCRIPTS = microsim.wasm microsim.wasm.map
CLEANFILES = $(bin_SCRIPTS)

microsim_SOURCES = microsim.cc
microsim_LDADD = \
	../pic14/libpic14.la \
	../core/libcore.la \
	../util/libutil.la \
	-lembind

# emcc warns about "running limited binaryen optimizations because DWARF info requested"
# if -gsource-map is used, and -O0 doesn't stop it.
microsim_LDFLAGS = \
	$(AM_LDFLAGS) \
	-sLLD_REPORT_UNDEFINED \
	-sIGNORE_MISSING_MAIN=1 \
	-sMINIMAL_RUNTIME=2 \
	-sINVOKE_RUN=0 \
	-sEXIT_RUNTIME=0 \
	-sINCOMING_MODULE_JS_API=onAbort,onRuntimeInitialized,print,printErr \
	-sALLOW_MEMORY_GROWTH=1 \
	-sALLOW_TABLE_GROWTH=1 \
	-sFILESYSTEM=0 \
	-sNODERAWFS=0

# https://www.gnu.org/software//automake/manual/html_node/Multiple-Outputs.html
microsim.wasm: microsim$(EXEEXT)
microsim.wasm.map: microsim$(EXEEXT)
