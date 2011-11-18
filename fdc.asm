;*******************************************************************************
;
; Description:
;   Equates for the floppy disk controller and low level driver
;
; History:
;   2011-03-20
;     First edition
;
;*******************************************************************************
;
; Macro definitions
;
; Set disk commands termination vector
    mac   dvec
    lda   #<{1}
    sta   disk_vector
    lda   #>{1}
    sta   disk_vector+1
    endm
; Pop the interrupt status byte and return vector from the stack
    mac   popint
    pla                         ; Pop the status register
    pla                         ; Pop low irq return address
    pla                         ; Pop high irq return address
    endm

FDC_BASE      equ   $F820
FDC_STAT      equ   ($00+FDC_BASE)
FDC_CTRL      equ   ($00+FDC_BASE)
FDC_TRCK      equ   ($01+FDC_BASE)
FDC_SECT      equ   ($02+FDC_BASE)
FDC_DATA      equ   ($03+FDC_BASE)
;
; Control bit definitons
;
FDC_SIZE      equ   $80         ; Floppy type select. 0=5 1/4" or 3.5", 1=8"
FDC_DDEN      equ   $40         ; Double Density Enable, 0=enabled, 1=disabled
FDC_DENSITY   equ   $20         ; Disk density set
FDC_SIDE      equ   $10         ; Side select, 0 = side 1, 1 = side 2
SIDE_MASK     equ   $EF
FDC_MOTOR_B   equ   $08
FDC_DRVSEL_A  equ   $04
FDC_DRVSEL_B  equ   $02
FDC_MOTOR_A   equ   $01
;
; FDC Commands
; Command definition is in bits 4-7
; Type I commands
FDCC_RESTORE  equ   $00
FDCC_SEEK     equ   $10
FDCC_STEP     equ   $20
FDCC_STEPIN   equ   $40
FDCC_STEPOUT  equ   $60
; Type II commands
FDCC_READ_SCT equ   $80
FDCC_WRIT_SCT equ   $A0
; Type III commands
FDCC_READ_ADR equ   $C0
FDCC_READ_TRK equ   $E0
FDCC_WRIT_TRK equ   $F0
FDCC_FOR_INT  equ   $D0
; Type I bit definitions
TRACK_UPDATE  equ   $10       ; T flag (Update track, 1=update, 0=no update)
HEAD_LOAD     equ   $08       ; h flag (Head load, 1=load head in beginning)
VERIFY_FLAG   equ   $04       ; V flag (Verify track, 1=Verify, 0=no verify)
RATE_0        equ   $00
RATE_1        equ   $01
RATE_2        equ   $02
RATE_3        equ   $03
; Type II and III bit definitions
SIDE_CMP_FLG  equ   $02       ; C flag
DELAY_15MS    equ   $04       ; E flag
SIDE_NUM_FLG  equ   $08       ; S flag
MULTIPLE_REC  equ   $10       ; m flag
;
; Type I Status register bit definition
FDC_NOT_READY equ   $80
FDC_WR_PROT   equ   $40
FDC_HEAD_LOADED equ $20
FDC_SEEK_ERR  equ   $10
FDC_CRC_ERR   equ   $08
FDC_TRACK_00  equ   $04
FDC_INDEX     equ   $02
FDC_BUSY      equ   $01
; Type II and III Status register bit definition
FDC_REC_TYP   equ   $20
FDC_REC_NF    equ   $10
FDC_DAT_LOST  equ   $04
FDC_DATA_REQ  equ   $02

NUM_OF_DRIVES equ   02  ; This is how many drives the system supports
NUM_OF_TRACKS equ   80  ; Number of physical tracks
NUM_OF_SIDES  equ   2   ; Number of sides per disk
;
; Here we define localy used variables    
    seg.u     zero_page
fdctemp       ds  2 ; Temporary storage used by the fdc module
fdcysave      ds  1 ; Just a location to temporarily store Y
fdcintsave    ds  1 ; Somewhere to temproarily store the VIA int status
fdcstat       ds  1
seccnt        ds  1
secpos        ds  1
secbytes      ds  2
skipbytes     ds  1
sectrack      ds  1
nxtdrive      ds  1   ; Requested drive
nxtsector     ds  2   ; Logical requested sector
phytrack      ds  1   ; Physical track number
physector     ds  1   ; Physical sector
trcnt         ds  1   ; track counter
dskside       ds  1   ; Disk side
dsktimer      ds  2   ; Disk on timer
dskon         ds  1   ; Flag indicating motor status


    seg.u     data_storage
timertab_lo   ds  256
timertab_hi   ds  256
sector_tab0   ds  32
sector_tab1   ds  32

    seg       code
;===============================================================================
;
; Initialize resources used by the floppy disk system
; As of HW rev. B FDC control pins (5*/8, ENMF* and DDEN*) are connected to the 
; SCN2681 UART pins OP6 and OP7. Prototype rev. A has been modified for this.
;
;===============================================================================
fdc_init:
    lda   #$FF
    sta   VIA_ORB           ; Prestore all disks off in the output register
    lda   #%00111111
    sta   VIA_DDRB          ; PB0-PB5 are outputs, PB6 and PB7 are inputs
    
    lda   #%11000000
    sta   SERIAL_OPBS       ; Clear FDC control pins
;    lda   #$FF & ~(FDC_SIZE | FDC_DDEN)
;    sta   VIA_ORB
    
;    lda   VIA_PCR
;    ora   #%00010000          ; Request interrupts on CB1 low-to-high
;    sta   VIA_PCR
;    lda   #%10010000
;    sta   VIA_IER             ; Enable disk controller interrupts

    lda   #$00
    sta   trcnt
    sta   dskon
    sta   phytrack
    sta   physector
    sta   nxtdrive          ; Select the first drive per default
    sta   dsktimer
    sta   dsktimer+1
    
    dvec  fdc_default_vec
    
    rts
;===============================================================================
;
; This is just a default vector for FDC interrupts
;
;===============================================================================
fdc_default_vec:
    rti
;===============================================================================
;
; The FDC controller have some timing requirements that need to be met in
; order for status register reads to be valid.
; The following matrix is shown in the application note:
;
;  Operation   |  Next operation  |  Delay FM  |  Delay MFM
;  ------------+------------------+------------+-----------
;  Write to    | Read busy bit    |   12 uS    |    6 uS
;  cmd reg.    | (status bit 0)   |            |
;  ------------+------------------+------------+-----------
;  Write to    |   Read status    |   28 uS    |   14 uS
;  cmd reg.    |   bits 1-7       |            |
;  ------------+------------------+------------+-----------
;  Write other | Read from diff.  |     0      |     0
;  registers   | register         |            |
;  ------------+------------------+------------+-----------
;
; Delay calculations are based on a 65C02 processor running at 2 MHz
; Which is 500 nS per cycle
;
;===============================================================================
fdc_delay:                    ; The JSR to here adds 6 cycles to get here
                              ; Number of cycles to execute
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    nop                       ; 2
    
    rts                       ; 6
;===============================================================================
;
; Start or stop the motor and activate the drive according to the accumulator
;
;===============================================================================
fdcf_set_motor:
    cmp   #$00
    beq   fdc_stop_motor
    lda   VIA_ORB             ; Start motor and select drive
    and   #(~(FDC_MOTOR_B | FDC_DRVSEL_B) & $FF)
    sta   VIA_ORB
    rts
fdc_stop_motor:
    lda   VIA_ORB             ; Stop motor and deselect drive
    ora   #(FDC_MOTOR_B | FDC_DRVSEL_B)
    sta   VIA_ORB
    rts
;===============================================================================
;
; Wait for the disk to be ready
;
;===============================================================================
fdc_not_ready_wait
    pha
fdc_not_ready_001
    lda   FDC_STAT          ; Make sure the drive is ready
    and   #FDC_NOT_READY
    bne   fdc_not_ready_001
    pla
    
    rts
;===============================================================================
;
; Check if the drive is busy, if so wait
;
;===============================================================================
fdc_drive_busy_wait:
    jsr   fdc_delay           ; Necessary to meet data sheet requirements.
    lda   #FDC_BUSY           ; Make sure no command is in progress
fdc_drive_busy_wait_001:    
    bit   FDC_STAT
    bne   fdc_drive_busy_wait_001

    rts
;===============================================================================
;
; Set side of disk
;
;===============================================================================
fdc_set_side:
    lda   dskside
    bne   set_side_001
    lda   VIA_ORB
    and   #SIDE_MASK          ; Clear side bit
    sta   VIA_ORB             ; and write it back to the io port
    rts
set_side_001:
    lda   VIA_ORB
    ora   #FDC_SIDE           ; Set side bit
    sta   VIA_ORB             ; and write it back to the io port
    rts
;===============================================================================
;
; Check if the drive is busy, if so wait
;
;===============================================================================
fdc_motor_on:
    jsr   fdc_set_side        ; First set what side to write on
    sei                       ; Disable interrupts temporarily
    lda   dskon               ; Check if motor is running already
    bne   motor_001
    
    jsr   via_setup_one_shot  ; Setup timer
    lda   #1
    sta   dskon
    jsr   fdcf_set_motor      ; Start motor and enable disk
    lda   #$00
    sta   dsktimer
    sta   dsktimer+1
    cli
motor_000:
    lda   dsktimer            ; Wait for disk to spin up (500 mS)
    cmp   #50
    bne   motor_000
    rts
    
motor_001:
    lda   #$00                ; Disk is already running so clear timer
    sta   dsktimer            ; and return
    sta   dsktimer+1
    cli                       ; Enable ints again

    rts    
;===============================================================================
;
; Select the current drive
; Remarks:
;   Currently only selects drive B, needs to be refined to select the requested
;   drive. Translates the requested logical sector and track to physical
;   information for the fdc.
;   The logical sector representation is a simple sector pointer that spans
;   the entire disk (0-2879). The logical sectors are translated to physical
;   track, side and sector data in this function.
;
; Returns:
;   Carry set indicates that there was an error during the operation.
;
;
;===============================================================================
fdc_select_drive:
    lda   nxtdrive
    cmp   #NUM_OF_DRIVES
    bcc   select_001          ; Check that it is a valid drive
   
    lda   #1                  ; Invalid drive
    sta   error
    sec
    rts
select_001:
    lda   nxtsector           ; Work out the physical parameters
    sta   dividend
    lda   nxtsector+1
    sta   dividend+1
    lsr   dividend+1          ; Divide by two for dual sided disk
    ror   dividend

    bcs   select_002          ; Jump if side 1
    lda   #$00
    sta   dskside
    beq   select_003
select_002:
    lda   #$01
    sta   dskside
select_003:
    lda   #$00
    sta   partial
    sta   partial+1
    sta   divisor+1
    lda   sectrack            ; Divide by number of sectors per track
    sta   divisor
    jsr   usdiv               ; Divide the number
    lda   dividend
    sta   phytrack            ; Set physical track
    lda   partial
    sta   physector           ; Set physical sector
    inc   physector           ; Sectors are numbered from 1
    
    jsr   fdc_motor_on
    clc
    rts
;===============================================================================
;
; Search for track 0 command
;
;===============================================================================
fdcf_restore:
    jsr   fdc_drive_busy_wait
    jsr   fdc_motor_on
    
    lda   #FDCC_RESTORE
    sta   FDC_CTRL
restore_001:
    bit   VIA_IRB
    bvc   restore_001
;
; Interrupt function for type 1 commands
fdcf_type1_end:
    lda   FDC_STAT            ; get fdc status
    and   #(FDC_SEEK_ERR + FDC_CRC_ERR)
    bne   type1_error
    clc
    rts
type1_error:
    sec
    rts
;===============================================================================
;
; Seek track command
; Input parameters:
;   A - Track to seek
;   X - Disk side
;
;===============================================================================
fdcf_seek:
    sta   phytrack
    jsr   fdc_motor_on
    
    lda   phytrack
    cmp   FDC_TRCK            ; Check that we're actually moving
    beq   seek_exit           ; Nops, same as before so skip it.

    jsr   fdc_drive_busy_wait
    jsr   fdc_motor_on    
    
    lda   phytrack
    sta   FDC_DATA
    lda   #FDCC_SEEK
    sta   FDC_CTRL
fdcf_seek_001:
    bit   VIA_IRB
    bvs   fdcf_type1_end
    bvc   fdcf_seek_001
seek_exit:
    clc
    rts
;===============================================================================
;
; Read Sector Command
; Parameters:
;   disk_buf_ptr - Pointer to where the data should be placed
;   nxtsector    - The sector that should be read
;
;===============================================================================
fdcf_read_sect:
    pha                       ; save accumulator
    jsr   fdc_select_drive    ; Select drive, calc phy and start motor
    
    lda   phytrack            ; Physical track
    jsr   fdcf_seek
    
    sei                       ; Disable all interrupts while writing track

    lda   physector
    sta   FDC_SECT            ; Store the sector number we are looking for.
    ldy   #$00
    
    lda   #%00100000
    sta   VIA_IER             ; While doing a sector read we disable timer 2      

    lda   #FDCC_READ_SCT
    sta   FDC_CTRL            ; Start the reading processs
rsect_001:
    bit   VIA_IRB             ; 4
    bvs   fdc_rsect_exit      ; 3
    bpl   rsect_001           ; 3
    lda   FDC_DATA            ; 4
    sta   (disk_buf_ptr),Y    ; 6
    iny                       ; 2
    bne   rsect_001           ; 3
    inc   disk_buf_ptr+1      ; 3
    bne   rsect_001           ; 3
    lda   #$80                ; If we end up here we have a internal error
    sec
    rts    
fdc_rsect_exit:
    jmp   fdc_type2_handler
;===============================================================================
;
; Write Sector Command
; Parameters:
;   disk_buf_ptr - Pointer to where the data to write is.
;   nxtsector    - The sector that should be written.
;
;===============================================================================
fdcf_write_sect:
    pha                       ; save accumulator
    lda   FDC_STAT
    and   #FDC_WR_PROT
    beq   wsect_000
    sec
    tay
    pla
    tya
    rts                       ; Write protected, so return with error cond.
wsect_000:
    jsr   fdc_select_drive    ; Select drive, calc phy and start motor
    
    lda   phytrack            ; Physical track
    jsr   fdcf_seek
    
    sei                       ; Disable all interrupts while writing track

    lda   physector
    sta   FDC_SECT            ; Store the sector number we are looking for.
    ldy   #$00

    lda   #%00100000
    sta   VIA_IER             ; While doing a sector write we disable timer 2

    lda   #FDCC_WRIT_SCT
    sta   FDC_CTRL            ; Start the reading processs
wsect_001:
    bit   VIA_IRB             ; 4
    bvs   fdc_wsect_exit      ; 3
    bpl   wsect_001           ; 3
    lda   (disk_buf_ptr),Y    ; 6
    sta   FDC_DATA            ; 4
    iny                       ; 2
    bne   wsect_001           ; 3
    inc   disk_buf_ptr+1
    bne   wsect_001
    lda   #$80                ; If we end up here we have a internal error
    sec
    rts
fdc_wsect_exit
    jmp   fdc_type2_handler
;===============================================================================
;
; Read Track address
;
;===============================================================================
fdcf_read_address:
    pha                       ; Save Acc, restored in interrupt
    ldy   #$00
    jsr   fdc_motor_on
    
    sei                       ; Disable all interrupts while writing track
    
    lda   #FDCC_READ_ADR + DELAY_15MS
    sta   FDC_CTRL
ra_001:
    bit   VIA_IRB             ; 4
    bvs   fdc_ra_exit
    bpl   ra_001              ; 3
    lda   FDC_DATA
    sta   (disk_buf_ptr),Y
    iny
    bne   ra_001
    lda   #$80                ; Indicate internal error
    sec
    rts
fdc_ra_exit:
    jmp   fdc_type2_handler
;===============================================================================
;
; Read Entire Track
;
;===============================================================================
fdcf_read_track:
    pha                       ; Save accumlator (restored in interrupt)
    ldy   #$00
    jsr   fdcf_seek
    
    sei                       ; Disable all interrupts while writing track

    lda   #FDCC_READ_TRK + DELAY_15MS
    sta   FDC_CTRL
    jsr   fdc_delay           ; delay before starting to read the status
rt_001:
    bit   VIA_IRB             ; 4
    bvs   fdc_rt_exit         ; 3
    bpl   rt_001              ; 3
    lda   FDC_DATA
    sta   (disk_buf_ptr),Y
    iny
    bne   rt_001
    inc   disk_buf_ptr+1
    bpl   rt_001
    lda   #$80                ; Error code
    sec
    rts
fdc_rt_exit:
    jmp   fdc_type2_handler
;===============================================================================
;
; Write X numbers of A into the formatting ram buffer
;
;===============================================================================
write_buffer:
    sty   fdcysave
    ldy   #0
wp_001:
    sta   (fdctemp),Y
    inc   fdctemp
    bne   wp_002
    inc   fdctemp+1
wp_002:
    dex
    bne   wp_001
    ldy   fdcysave
    rts
;===============================================================================
;
; This function sets the disk preamble in the ram buffer
;
;===============================================================================
set_preamble:
    ; This section is for double density disks.
    lda   #$4E
    ldx   #80
    jsr   write_buffer
    lda   #$00
    ldx   #12
    jsr   write_buffer
    lda   #$f6
    ldx   #3
    jsr   write_buffer
    lda   #$fc
    ldx   #1
    jsr   write_buffer
    lda   #$4e
    ldx   #50
    jsr   write_buffer
    rts
;===============================================================================
;
; Caclulate the sector interleaving and store it in the interleaving table.
;
;===============================================================================
calc_interleave:
    ldx   #31                 ; first clear the sector table
    lda   #$00
ci_001:
    sta   sector_tab0,X       ; Side 0 interleave table
    sta   sector_tab1,X       ; Side 1
    dex
    bpl   ci_001

    lda   #$01                ; Always start with sector 1
    sta   seccnt
    lda   #$00                ; Interleave table position
    sta   secpos
    ldy   sectrack
ci_002:
    ldx   secpos              ; Get the sector table position
ci_003:
    lda   sector_tab0,X       ; Check that is it a free entry
    beq   ci_005              ; Skip if all is ok
    inx                       ; Look for a free entry on the next address
    cpx   sectrack            ; Check that are not exceeding the table limit
    bne   ci_003              ; Go back and check again
    sec                       ; Could not find a free entry for the sector
    rts                       ; So return with an error indication
ci_005:
    lda   seccnt              ; Get the current phy sector number
    sta   sector_tab0,X       ; Store the phy sector number in the table
    txa
    clc
    adc   skipbytes           ; calculate the next sector table position
    cmp   sectrack            ; Do we need to wrap
    bcc   ci_010              ; No, skip to next
    sec                       ; Prepare to subtract
    sbc   sectrack            ; Wrap around
ci_010:
    sta   secpos
    tax
ci_020:
    lda   sector_tab1,X       ; Check that is it a free entry
    beq   ci_030              ; Skip if all is ok
    inx                       ; Look for a free entry on the next address
    cpx   sectrack            ; Check that are not exceeding the table limit
    bne   ci_030              ; Go back and check again
    sec                       ; Could not find a free entry for the sector
    rts                       ; So return with an error indication
ci_030:
    lda   seccnt              ; get phy sector
    inc   seccnt              ; Increment for next operation
    sta   sector_tab1,X       ; Store in interleave table
    txa
    clc
    adc   skipbytes           ; calculate the next sector table position
    cmp   sectrack            ; Do we need to wrap
    bcc   ci_040
    sec
    sbc   sectrack
ci_040:
    sta   secpos
    dey
    bne   ci_002
    
    clc
    rts
    
;===============================================================================
;
; This function creates a disk sector in the disk write ram buffer
;
;===============================================================================
set_sector:
    pha                       ; Save Acc
    tya
    pha                       ; Save Y
    
    lda   #$00
    ldx   #12
    jsr   write_buffer
    lda   #$F5
    ldx   #3
    jsr   write_buffer
    lda   #$FE
    ldx   #1
    jsr   write_buffer
    lda   phytrack            ; Track number
    ldx   #1
    jsr   write_buffer
    lda   dskside             ; Disk side
    bne   set_sector_side_1
    ldx   #1
    jsr   write_buffer
    ldx   physector           ; Sector number
    lda   sector_tab0,X       ; Get sector interleaving from table
    ldx   #1
    jsr   write_buffer
    jmp   set_sector_skip
set_sector_side_1:
    ldx   #1
    jsr   write_buffer
    ldx   physector           ; Sector number
    lda   sector_tab1,X       ; Get sector interleaving from table
    ldx   #1
    jsr   write_buffer
set_sector_skip:
    lda   secbytes            ; Number of bytes per sector (01 = 256 bytes)
    ldx   #1
    jsr   write_buffer
    lda   #$F7                ; Generate CRC
    ldx   #1
    jsr   write_buffer
    lda   #$4e
    ldx   #22
    jsr   write_buffer
    lda   #$00
    ldx   #12
    jsr   write_buffer
    lda   #$F5
    ldx   #3
    jsr   write_buffer
    lda   #$FB
    ldx   #1
    jsr   write_buffer
;
; Loop to fill the data area up to 1024 bytes
    lda   #$E5                ; Data area
    ldx   secbytes
    ldy   set_sector_len_tab,X ; Number of multiples of 128
set_sector_loop:
    ldx   #128
    jsr   write_buffer
    dey
    bne   set_sector_loop
    
    lda   #$F7                ; generate CRC
    ldx   #1
    jsr   write_buffer
    lda   #$4E
    ldx   #54
    jsr   write_buffer
    
    pla                       ; restore Acc and Y
    tay
    pla
    
    rts
set_sector_len_tab:
    dc.b  1, 2, 4, 8
;===============================================================================
;
; Format one track.
;   This is achieved by building the track information in RAM before blasting
;   the built RAM buffer onto the diskette.
;
; Parameters:
;   A - Holds the desired track to do a format on
;   X - Holds the side of the disk.
;
;===============================================================================
fdcf_format_track:
    stx   dskside
    sta   phytrack            ; Temporarily store the track to write
    lda   #$0
    sta   physector           ; Starting sector number

    tya
    pha                       ; Save Y
    jsr   calc_interleave     ; Calculate interleave tables
    lda   #$20                ; possible error cause
    bcs   fdc_format_error
    jsr   fdc_set_side        ; Set the right side to write on
    
    lda   #FDCC_FOR_INT       ; Ensure that we are in type 1 mode
    sta   FDC_CTRL
    jsr   fdc_delay
    jsr   fdc_delay

    lda   #%00100000
    sta   VIA_IER             ; While doing a format we need to disable timer 2    
    
    lda   FDC_STAT            ; Make sure the disk is not write protected
    and   #FDC_WR_PROT
    bne   fdc_format_error

    lda   phytrack            ; Seek the track to format 
    jsr   fdcf_seek           ; Also calls fdc_motor_on
    
    lda   #<FORMAT_BUFFER     ; Set up buffer pointer
    sta   fdctemp
    lda   #>FORMAT_BUFFER
    sta   fdctemp+1

    jsr   set_preamble        ; Write the preamble to the buffer
    ldy   sectrack            ; We're going to create n sectors in memory
format_001:
    jsr   set_sector
    inc   physector
    dey
    bne   format_001
    ldy   #0
    lda   #$4E                ; Fill the rest of the buffer with $4E
format_002:
    sta   (fdctemp),Y
    inc   fdctemp
    bne   format_003
    inc   fdctemp+1
format_003:
    ldx   fdctemp+1
    cpx   #$80                ; High pointer reaches $80(00) which is EOT
    bne   format_002
    
    lda   #<FORMAT_BUFFER     ; Set up buffer pointer
    sta   fdctemp
    lda   #>FORMAT_BUFFER
    sta   fdctemp+1

    sei                       ; Disable all interrupts while writing track

    lda   phytrack            ; Set track number to write
    sta   FDC_TRCK
    lda   #FDCC_WRIT_TRK  + DELAY_15MS
    ldy   #0
    sta   FDC_CTRL
format_004:
    bit   VIA_IRB             ; 4
    bvs   fdc_type2_handler   ; 3
    bpl   format_004          ; 3
    lda   (fdctemp),Y         ; 5
    sta   FDC_DATA            ; 4
    iny                       ; 2
    bne   format_004          ; 3
    inc   fdctemp+1           ; 5
    bpl   format_004          ; 3
    lda   #$80                ; If we end up here, something is wrong
fdc_format_error:
    tay                       ; We want to return with error cause in Acc
    pla                       ; pop Y from stack, not relevant since error
    tya
    sec
    rts

; This is the type II/III interrupt service handler.
fdc_type2_handler:
    lda   FDC_STAT            ; Clear interrupt (NMI)
    sta   fdcstat             ; Store it
    sty   error_offs

    pla
    tay                       ; Restore Y

    cli                       ; Enable system interrupts again

    jsr   via_setup_one_shot  ; Go start the motor timer again
    lda   fdcstat             ; Clear interrupt (NMI)
    and   #(FDC_REC_NF + FDC_DAT_LOST + FDC_CRC_ERR)
    bne   fdc_type2_error
    clc                       ; Indicate no error
    rts
fdc_type2_error:
    sec
    rts                       ; return with error code in acc
;
; EOF
;    