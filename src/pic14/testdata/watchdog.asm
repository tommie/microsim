    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_ON & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    btfss   STATUS, NOT_TO
    goto    timed_out

    clrwdt

    banksel WDTCON
    clrf    WDTCON

loop1:
    goto    loop1

timed_out:
    movlw   0x01
    banksel PORTA
    movwf   PORTA
    xorlw   0xFF
    banksel TRISA
    movwf   TRISA
    sleep

    end
