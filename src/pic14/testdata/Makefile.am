include ../../common.mk

GPASM_FLAGS = -p p16f887 -r dec

noinst_DATA = \
	adc.cod \
	eedata.cod \
	eeprog.cod \
	eusart_abd.cod \
	eusart_async_rc.cod \
	eusart_async_tx.cod \
	eusart_master_rc.cod \
	eusart_master_tx.cod \
	eusart_slave_rc.cod \
	eusart_slave_tx.cod \
	eusart_wakeup.cod \
	extint.cod \
	gie.cod \
	port.cod \
	t0if.cod \
	timer0.cod \
	ulpwu.cod \
	watchdog.cod
CLEANFILES = $(noinst_DATA)

clean-local:
	for f in $(noinst_DATA); do rm $${f%.cod}.lst $${f%.cod}.hex || exit $? ; done

SUFFIXES = .asm .cod .hex .lst

.cod.hex:
.cod.lst:
.asm.cod:
	$(GPASM) $(GPASM_FLAGS) -o $@ $^
