    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    banksel RCSTA
    bsf     RCSTA, SPEN
    bsf     RCSTA, CREN

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PIE1
    bsf     PIE1, RCIE

    banksel PIR1
wait_rcif:
    btfss   PIR1, RCIF
    goto    wait_rcif

    banksel RCSTA
    btfsc   RCSTA, OERR
    goto    not_right

    btfsc   RCSTA, FERR
    goto    not_right

    bcf     RCSTA, CREN

    banksel RCREG
    movf    RCREG, W
    sublw   42
    btfss   STATUS, Z
    goto    not_right

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
