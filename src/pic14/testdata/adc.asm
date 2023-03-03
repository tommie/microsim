    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA
    movlw   (1 << ADCS1) | (1 << ADCS0) | (1 << CHS0) | (1 << ADON)
    banksel ADCON0
    movwf   ADCON0

    ;; Sampling delay not simulated.

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PIR1
    bcf     PIR1, ADIF
    banksel PIE1
    bsf     PIR1, ADIE
    banksel ADCON0
    bsf     ADCON0, GO
    sleep

    banksel PIR1
    bcf     PIR1, ADIF
    banksel ADCON0
    btfsc   ADCON0, GO
    goto    not_right

    banksel ADRESH
    movlw   0x81
    subwf   ADRESH, W
    btfsc   STATUS, C
    goto    not_right

    banksel ADRESH
    movlw   0x7F
    subwf   ADRESH, W
    btfss   STATUS, C
    goto    not_right

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
