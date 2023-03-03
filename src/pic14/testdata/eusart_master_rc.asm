    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    movlw   (1 << SPEN) | (1 << SREN)
    banksel RCSTA
    movwf   RCSTA
    movlw   (1 << SYNC) | (1 << CSRC)
    banksel TXSTA
    movwf   TXSTA

    banksel PIR1
wait_rcif:
    btfss   PIR1, RCIF
    goto    wait_rcif

    banksel RCSTA
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
