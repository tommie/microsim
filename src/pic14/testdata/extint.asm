    include "p16f887.inc"

    movlw   ~0x01
    banksel TRISA
    movwf   TRISA

    bcf     INTCON, INTF
    bsf     INTCON, INTE
    sleep

    bcf     INTCON, INTF
    banksel PORTB
    movf    PORTB, W
    banksel PORTA
    movwf   PORTA
    sleep

    end
