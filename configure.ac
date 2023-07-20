AC_INIT([microsim], [0.1], [tommie@users.noreply.github.com])
AC_CONFIG_MACRO_DIRS([autom4])
AC_CONFIG_AUX_DIR([autoaux])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])

AC_CANONICAL_HOST

AM_PROG_AR
AC_PROG_CXX
LT_INIT
AC_PATH_PROG([GPASM], [gpasm])

AC_MSG_CHECKING([whether the target is WASM])
have_wasm=
case "$host" in
wasm32-*|wasm64-*) have_wasm=yes ;;
esac
if test -n "$have_wasm"; then
  AC_MSG_RESULT([yes])
  AC_DEFINE([HAVE_WASM], 1, [Whether the target is WebAssembly.])
else
  AC_MSG_RESULT([no])
fi
AM_CONDITIONAL([HAVE_WASM], [test -n "$have_wasm"])

AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_FILES([
 Makefile
 src/Makefile
 src/core/Makefile
 src/pic14/Makefile
 src/pic14/testdata/Makefile
 src/testing/Makefile
 src/util/Makefile
])
AC_OUTPUT
