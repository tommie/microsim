    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    clrf    PORTA
    movlw   ~0x01
    banksel TRISA
    movwf   TRISA

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PCON
    bsf     PCON, ULPWUE
    banksel PIR2
    bcf     PIR2, ULPWUIF
    banksel PIE2
    bsf     PIE2, ULPWUIE
    sleep

    banksel PIR2
    bcf     PIR2, ULPWUIF
    banksel PORTA
    movlw   0x01
    movwf   PORTA
    sleep

    end
