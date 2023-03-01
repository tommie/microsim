    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    banksel OPTION_REG
    bcf     OPTION_REG, T0CS
    banksel TMR0
    clrf    TMR0
    bcf     INTCON, T0IF
    bsf     INTCON, T0IE

loop:
    btfss   INTCON, T0IF
    goto    loop

    bcf     INTCON, T0IF
    banksel PORTA
    movlw   1
    movwf   PORTA

    sleep

    end
