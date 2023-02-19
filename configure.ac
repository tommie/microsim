AC_INIT([microsim], [0.1], [tommie@users.noreply.github.com])
AC_CONFIG_MACRO_DIRS([autom4])
AC_CONFIG_AUX_DIR([autoaux])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AM_PROG_AR
AC_PROG_CXX
LT_INIT
AC_PATH_PROG([GPASM], [gpasm])

# Emscripten flags:
#   -sLLD_REPORT_UNDEFINED -sSTRICT_JS -sINCOMING_MODULE_JS_API=noExitRuntime,noInitialRun,onAbort,onRuntimeInitialized,print,printErr

AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_FILES([
 Makefile
 src/Makefile
 src/core/Makefile
 src/pic14/Makefile
 src/testing/Makefile
 src/util/Makefile
])
AC_OUTPUT
