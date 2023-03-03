    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    ;; Write 42 to EEDATA[0x01].

    movlw   1
    banksel EEADR
    movwf   EEADR
    movlw   42
    movwf   EEDAT

    banksel INTCON
    bsf     INTCON, PEIE
    banksel PIR2
    bcf     PIR2, EEIF
    banksel PIE2
    bsf     PIE2, EEIE

    banksel EECON1
    bcf     EECON1, EEPGD
    bsf     EECON1, WREN

    movlw   0x55
    movwf   EECON2
    movlw   0xAA
    movwf   EECON2
    bsf     EECON1, WR

    sleep

    ;; Verify write.

    bcf     EECON1, WREN
    banksel PIR2
    bcf     PIR2, EEIF

    movlw   1
    banksel EEADR
    movwf   EEADR
    banksel EECON1
    bsf     EECON1, RD

    banksel EEDAT
    movf    EEDAT, W
    sublw   42

    btfss   STATUS, Z
    goto    not_right

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
