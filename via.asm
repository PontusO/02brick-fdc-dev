;*******************************************************************************
;
; Description:
;   Equates for the 65C22 on the 02Brick computer.
;
; History:
;   2011-03-20
;     First edition
;
;*******************************************************************************
;
VIA_BASE    equ   $F800
VIA_ORB     equ   ($00+VIA_BASE)
VIA_IRB     equ   ($00+VIA_BASE)
VIA_ORA     equ   ($01+VIA_BASE)
VIA_IRA     equ   ($01+VIA_BASE)
VIA_DDRB    equ   ($02+VIA_BASE)
VIA_DDRA    equ   ($03+VIA_BASE)
VIA_T1CL    equ   ($04+VIA_BASE)
VIA_T1CH    equ   ($05+VIA_BASE)
VIA_T1LL    equ   ($06+VIA_BASE)
VIA_T1LH    equ   ($07+VIA_BASE)
VIA_T2CL    equ   ($08+VIA_BASE)
VIA_T2CH    equ   ($09+VIA_BASE)
VIA_SR      equ   ($0A+VIA_BASE)
VIA_ACR     equ   ($0B+VIA_BASE)
VIA_PCR     equ   ($0C+VIA_BASE)
VIA_IFR     equ   ($0D+VIA_BASE)
VIA_IER     equ   ($0E+VIA_BASE)
VIA_ORAN    equ   ($0F+VIA_BASE)
VIA_IRAN    equ   ($0F+VIA_BASE)
;
; Interrupt sources
;
VIA_INT_ANY equ   %10000000
VIA_INT_T1  equ   %01000000
VIA_INT_T2  equ   %00100000
VIA_INT_CB1 equ   %00010000
VIA_INT_CB2 equ   %00001000
VIA_INT_SFT equ   %00000100
VIA_INT_CA1 equ   %00000010
VIA_INT_CA2 equ   %00000001

    seg.u zero_page

    seg   code    
;===============================================================================
;
; De select the current drive
;
;===============================================================================
via_setup_one_shot
    lda   #$00
    sta   VIA_ACR
    
    lda   #%10100000          ; Enable Timer 2 interrupts
    sta   VIA_IER
    
    lda   #$ff
    sta   VIA_T2CL
    lda   #$ff
    sta   VIA_T2CH
    
    rts
;
; EOF
;