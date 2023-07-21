AM_CXXFLAGS = $(MORE_CXXFLAGS)
AM_LDFLAGS = $(MORE_LDFLAGS)

if HAVE_WASM
LOG_COMPILER = node -e "import(process.argv[1]).then(Module => Module.default({ preRun: [] }))"

# Replace with this if srcdir is needed in tests:
#LOG_COMPILER = node -e "import(process.argv[1]).then(Module => Module.default({ preRun: [(Module) => { Module.ENV['srcdir'] = '$$srcdir'; }] }))"

# And add this to tests:
test_ldflags = --js-transform "$(top_srcdir)/tools/emfixenv"
endif
