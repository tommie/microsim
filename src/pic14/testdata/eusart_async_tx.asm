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
    bsf     TXSTA, TXEN

    banksel TXREG
    movlw   42
    movwf   TXREG

    ;; TXIF won't be clear unless both TSR and TXREG are in use.
    banksel TXSTA
wait_tx:
    btfsc   TXSTA, TRMT
    goto    wait_tx

    banksel TXREG
    movlw   0xAA
    movwf   TXREG

    banksel PIR1
    btfsc   PIR1, TXIF
    goto    not_right

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PIE1
    bsf     PIE1, TXIE

    banksel PIR1
wait_txif:
    btfss   PIR1, TXIF
    goto    wait_txif

    banksel TXSTA
    btfsc   TXSTA, TRMT
    goto    not_right

    banksel TXSTA
    bcf     TXSTA, TXEN

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
