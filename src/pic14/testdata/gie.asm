    include "p16f887.inc"

    goto    start

    org     4
irq:
    bcf     INTCON, INTF
    banksel PORTB
    movf    PORTB, W
    banksel PORTA
    movwf   PORTA
    retfie

start:
    movlw   ~0x01
    banksel TRISA
    movwf   TRISA

    bcf     INTCON, INTF
    bsf     INTCON, INTE
    bsf     INTCON, GIE
    sleep

    sleep

    end
