    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    banksel RCSTA
    bsf     RCSTA, SPEN
    banksel TXSTA
    movlw   (1 << SYNC) | (1 << TXEN)
    movwf   TXSTA

    banksel TXREG
    movlw   42
    movwf   TXREG

    banksel TXSTA
    btfsc   TXSTA, TRMT
    goto    not_right

    ;; TXIF won't be clear unless both TSR and TXREG are in use.
    banksel TXREG
    movlw   43
    movwf   TXREG

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PIE1
    bsf     PIE1, TXIE
    sleep

    banksel PIE1
    bcf     PIE1, TXIE

    banksel PIR1
    btfss   PIR1, TXIF
    goto    not_right

    banksel TXSTA
check_empty:
    btfss   TXSTA, TRMT
    goto    check_empty

    bcf     TXSTA, TXEN

    banksel PIR1
    btfsc   PIR1, TXIF
    goto    not_right

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
