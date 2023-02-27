# Microsim

Microsim is a PIC microcontroller simulator, inspired by the venerable [GNU PIC Simulator](https://sourceforge.net/projects/gpsim/).
The aim of this project is to provide a modular framework to creating microcontroller simulators in C++.
It is using C++17 and can be compiled to WebAssembly.

## Status

This is in an early stage of development and is more of a meditation project than something serious at this point.

## Building

The project uses [GNU Automake](https://www.gnu.org/software/automake/manual/automake.html) for building:

```
autoreconf -i
./configure 'CXXFLAGS=-Wall -std=c++20 -g -fsanitize=address'
make -j2
```

## Testing

```
make check
```

Running a test under GDB:

```
cd src/pic14
make p16f887_test
../../libtool --mode=execute gdb p16f887_test
```

Useful environment variables:

* `ASAN_OPTIONS=abort_on_error=1` to make ASAN abort right away on issues it finds.
  Useful when using a debugger.
* `MICROSIM_TRACE_BUFFER_SIZE=1024` enables the cyclic trace buffer that provides information about the internal state changes of the simulator.

## License

Released under the [MIT License](https://opensource.org/license/mit/).
