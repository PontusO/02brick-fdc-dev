; Unsigned 16-bit division
; Computes (dividend,partial) / divisor
; (i.e., 32 bits divided by 16 bits)
    seg.u     zero_page
    
dividend      ds  2 ; dividend
partial       ds  2 ; partial
divisor       ds  2 ; divisor
    
    seg       code
;===============================================================================
;
; Description:
;   Unsigned 32/16 division
;
;===============================================================================

usdiv:
    pha
    tya
    pha
    txa
    pha
    lda #%00000001
    sta VIA_ORAN
    ldy #$10                  ;set up for 16 bits
usdiv2:
    asl dividend
    rol dividend+$1
    rol partial
    rol partial+$1
    sec                       ;leave dividend mod divisor
    lda partial               ;in partial
    sbc divisor
    tax
    lda partial+$1
    sbc divisor+$1
    bcc usdiv3
    stx partial
    sta partial+$1
    inc dividend
usdiv3:
    dey
    bne usdiv2

    lda #0
    sta VIA_ORAN
    pla
    tax
    pla
    tay
    pla
    
    rts 