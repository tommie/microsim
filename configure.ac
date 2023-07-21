AC_INIT([microsim], [0.1], [tommie@users.noreply.github.com])
AC_CONFIG_MACRO_DIRS([autom4])
AC_CONFIG_AUX_DIR([autoaux])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])

AC_CANONICAL_HOST

AM_PROG_AR
AC_PROG_CXX
LT_INIT
AC_PATH_PROG([GPASM], [gpasm])
AC_ARG_VAR([GPASM], [GnuPIC assembler])
AC_PATH_PROG([NODE], [node])
AC_ARG_VAR([NODE], [NodeJS interpreter])

AC_MSG_CHECKING([whether the target is WASM])
have_wasm=
case "$host" in
wasm32-*|wasm64-*) have_wasm=yes ;;
esac
if test -n "$have_wasm"; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
fi

MORE_LDFLAGS=
MORE_TESTS_ENVIRONMENT=
if test -n "$have_wasm"; then
  AC_DEFINE([HAVE_WASM], 1, [Whether the target is WebAssembly.])

  EXEEXT=.mjs
  MORE_LDFLAGS='-Wno-limited-postlink-optimizations -sIGNORE_MISSING_MAIN=0 -sALLOW_MEMORY_GROWTH=1 -sINVOKE_RUN=1 -sEXIT_RUNTIME=1 -sNODERAWFS=1'
  MORE_TESTS_ENVIRONMENT="$NODE"
fi
AM_CONDITIONAL([HAVE_WASM], [test -n "$have_wasm"])

MORE_CXXFLAGS='-std=c++20 -Wall -Werror'

AC_SUBST([MORE_CXXFLAGS])
AC_SUBST([MORE_LDFLAGS])
AC_SUBST([MORE_TESTS_ENVIRONMENT])

wwwdir='${exec_prefix}'
AC_SUBST([wwwdir])

AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_FILES([
 Makefile
 src/Makefile
 src/core/Makefile
 src/pic14/Makefile
 src/pic14/testdata/Makefile
 src/testing/Makefile
 src/util/Makefile
 src/wasm/Makefile
])
AC_OUTPUT
