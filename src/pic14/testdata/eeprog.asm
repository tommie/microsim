    include "p16f887.inc"

    __config _CONFIG1, _INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _LVP_OFF & _DEBUG_OFF
    __config _CONFIG2, _BOR40V & _WRT_OFF

    movlw   ~0x01
    movwf   PORTA
    banksel TRISA
    movwf   TRISA

    ;; Write 42 to PROGMEM[0x0100-].

    movlw   HIGH 0x100
    banksel EEADRH
    movwf   EEADRH
    movlw   LOW 0x100
    movwf   EEADR

write:
    movlw   LOW 4242
    movwf   EEDAT
    movlw   HIGH 4242
    movwf   EEDATH

    banksel EECON1
    bsf     EECON1, EEPGD
    bsf     EECON1, WREN

    movlw   0x55
    movwf   EECON2
    movlw   0xAA
    movwf   EECON2
    bsf     EECON1, WR
    nop
    nop

    ;; The executor will halt for T_PEW here.

    banksel EEADR
    movf    EEADR, W
    incf    EEADR, F
    andlw   0x0F
    sublw   0x0F
    btfss   STATUS, Z
    goto    write

    banksel INTCON
    bcf     INTCON, INTF
    bsf     INTCON, INTE

    sleep

    bcf     INTCON, INTF

    ;; Verify write.

    bcf     EECON1, WREN

    movlw   HIGH 0x100
    banksel EEADRH
    movwf   EEADRH
    movlw   LOW 0x100
    movwf   EEADR

    banksel EECON1
    bsf     EECON1, RD
    nop
    nop

    banksel EEDAT
    movf    EEDAT, W
    sublw   LOW 4242

    btfss   STATUS, Z
    goto    not_right

    movf    EEDATH, W
    sublw   HIGH 4242

    btfss   STATUS, Z
    goto    not_right

    movlw   1
    banksel PORTA
    movwf   PORTA

not_right:
    sleep

    end
