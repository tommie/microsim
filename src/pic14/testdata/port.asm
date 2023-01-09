    include "p16f887.inc"

    movlw   ~0x01
    banksel TRISA
    movwf   TRISA

    banksel PORTB
    movf    PORTB, W
    banksel PORTA
    movwf   PORTA
    sleep

    end
