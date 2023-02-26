    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

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
