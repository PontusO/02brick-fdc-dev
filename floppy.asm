;*******************************************************************************
;
; Description:
;   This is a test program for controlling and evaluating the 6502-floppy
;   controller board.
;
; History:
;   2011-03-19
;     Starting to write the program.
;
;*******************************************************************************
    processor 6502
;
; Macro definitions
;
; print string
    mac   pstr
    pha
    lda   #{1} & $FF
    sta   string_ptr
    lda   #{1} / 256
    sta   string_ptr+1
    jsr   print_string
    pla
    endm
    
; print unsigned int16 number
    mac   pint16
    lda   #"$"
    jsr   putchar
    ldx   {1}
    ldy   {1}+1
    jsr   print_hex_int
    endm

; print unsigned int8 number
    mac   pint8
    lda   #"$"
    jsr   putchar
    lda   {1}
    jsr   print_hex_char
    endm
;  Base address of UART
UART_BASE     EQU     $F810
; Definition of the SCN2681 UART
SERIAL_MR1A   EQU   UART_BASE         ;Mode register 1 A
SERIAL_MR2A   EQU   UART_BASE         ;Mode register 2 A
SERIAL_CSRA   EQU   UART_BASE+1       ;Clock select register
SERIAL_CRA    EQU   UART_BASE+2       ;Control register A
SERIAL_ACR    EQU   UART_BASE+4       ;Auxillary control register
SERIAL_TXDATA EQU   UART_BASE+3       ;ACIA WRITE DATA
SERIAL_RXDATA EQU   UART_BASE+3       ;ACIA READ DATA
SERIAL_STATUS EQU   UART_BASE+1       ;ACIA STATUS
SERIAL_WCMD   EQU   UART_BASE+2       ;ACIA COMMAND REGISTER
SERIAL_WCTL   EQU   UART_BASE+3       ;ACIA CONTROL REGISTER
SERIAL_OPCR   EQU   UART_BASE+13      ;Uart output port control register
SERIAL_OPBS   EQU   UART_BASE+14      ;Uart set port bit register
SERIAL_OPBC   EQU   UART_BASE+15      ;Uart clear port bit register
RXRDY         EQU   $01               ;RECEIVE READY
TXRDY         EQU   $04               ;TRANSMIT READY
;
; Printable characters
CR            equ   $0D
LF            equ   $0A
SPACE         equ   $20
TAB           equ   $09
COMMA         equ   ","
NULL          equ   $00
;
; Non printable characters
DEL           equ   $08
;
; Other stuff
FORMAT_BUFFER equ   $6000 ; Temporary buffer for the formatting routines
;
; Zero page definitions
;
    seg.u     zero_page
    org       $0000
error         ds  1   ; system error number
error_offs    ds  1   ; Byte offset in sector
string_ptr    ds  2
parse_tab_ptr ds  2
parse_jmp_vec ds  2
disk_buf_ptr  ds  2
nmi_status    ds  1
numsctrs      ds  2
disk_vector   ds  2   ; jump vector used to terminate disk commands
;
; Noice vectors
;
    seg.u     noice_vectors
    org       $0200
nmi_vector    ds  4
irq_vector    ds  2
;
; High Data Storage
;
    seg.u     data_storage
    org       $0300
disk_buffer   ds  256
inbuffer      ds  256

    seg       code
    org       $1000
; The LOAD_ADDRESS variable tells the dasm noice generator to create a LOAD
; command in the generated .noi file.
LOAD_ADDRESS = *

    include   "math.asm"
    include   "via.asm"
    include   "fdc.asm"
    include   "atoi.asm"
    include   "xmodemrcv.asm"
    
;===============================================================================
;
; Print a nibble
;
; Parameters: A = Hex digit to print
;
;===============================================================================
print_hex_nibble:
    cmp   #$A
    bcc   phn_001
    clc
    adc   #a_hexnum
phn_001:
    adc   #$30
    jmp   putchar
;===============================================================================
;
; Print a hex 
;
; Parameters: A = Hex number to print
;
;===============================================================================
print_hex_char:
    pha
    lsr
    lsr
    lsr
    lsr
    jsr   print_hex_nibble
    pla
    and   #$f
    jmp   print_hex_nibble
;===============================================================================
;
; Print an integer to the serial port in hex format
;
; Parameters: x = low portion of data
;             y = high portion of data
;===============================================================================
print_hex_int:
    tya
    jsr   print_hex_char
    txa
    jmp   print_hex_char
;===============================================================================
;
; Get a character from the serial port.
;
;===============================================================================
getchar:
    lda   SERIAL_STATUS
    AND   #RXRDY
    BEQ   getchar         ; Not ready yet
    
    lda   SERIAL_RXDATA   ; Load received data
    rts
;===============================================================================
;
; Get a character from the serial port.
;
;===============================================================================
scanchar:
    clc
    lda   SERIAL_STATUS
    AND   #RXRDY
    BEQ   scanchar_001    ; Not ready yet
    lda   SERIAL_RXDATA   ; Load received data
    sec
scanchar_001:
    rts
;===============================================================================
;
; Get a character from the serial port.
;
;===============================================================================
putchar:
    pha
pc10:
    lda   SERIAL_STATUS   ;CHECK TX STATUS
    and   #TXRDY          ;RX READY ?
    beq   pc10
    pla
    sta   SERIAL_TXDATA   ;TRANSMIT CHAR.
    
    rts
crlf:
    lda   #CR
    jsr   putchar
    lda   #LF
    jmp   putchar
;===============================================================================
;
; Print a NULL terminated string pointer to by the string pointer $20,$21
;
;===============================================================================
print_string:
    tya
    pha
    ldy   #$00
ps001:
    lda   (string_ptr),Y
    beq   ps002
    jsr   putchar
    cmp   #CR              ; If we are printing a CR we also add a LF.
    bne   ps003
    lda   #LF
    jsr   putchar
ps003:
    inc   string_ptr
    bne   ps004
    inc   string_ptr+1
ps004:
    jmp   ps001
ps002:
    pla
    tay
    
    rts
;===============================================================================
;
; Get a string from the terminal, terminated by CR
; I could have added múch more error checking but decided against it.
;
;===============================================================================
get_string:
    pha                     ; Save Y and A
    tya
    pha
    
    ldy   #$00
gs001:
    jsr   getchar
    cmp   #DEL
    bne   gs001a
    cpy   #0                ; Check for beginning of buffer
    beq   gs001             ; If so, start again
    jsr   putchar
    lda   #SPACE
    jsr   putchar
    lda   #DEL
    jsr   putchar
    dey                     ; One step back in the buffer
    jmp   gs001
gs001a:
    jsr   putchar
    cmp   #CR
    beq   gs002
    sta   inbuffer,Y
    iny
    bne   gs001
gs002
    lda   #$00
    sta   inbuffer,Y        ; Terminate the string with 0
    
    pla                     ; Restore Y and A
    tay
    pla

    rts
;===============================================================================
;
; Initialize the hardware
;
;===============================================================================
init_hw

    rts
;===============================================================================
;
; Start of the program
;
;===============================================================================
START:
    jsr   init_hw
    jsr   fdc_init

;    lda   #<sys_nmi
;    sta   irq_vector
;    lda   #>sys_nmi
;    sta   irq_vector+1
    
    lda   #<sys_nmi
    sta   nmi_vector
    lda   #>sys_nmi
    sta   nmi_vector+1

    lda   #$00
    sta   error
    
    lda   #79                 ; Set to a known state
    jsr   fdcf_seek
    jsr   fdcf_restore
bigloop:
    pstr  start_string
loop:
    pstr  str_command
    jsr   get_string
    lda   inbuffer
    cmp   #"?"
    beq   bigloop
    
    jsr   parse_command
    
    jmp   loop

;===============================================================================
;
; System IRQ function
;
;===============================================================================
sys_irq:
    inc   error
    rti
    
;===============================================================================
;
; System IRQ function
;   The 65C22 device is connected to the NMI signal in this system. So we only
;   need to investigate the 65C22 IFR to see what has happened.
;
;===============================================================================
sys_nmi:
    pha                       ; Save acc during interrupt
    lda   VIA_IFR             ; We need to store this in our interrupt routine
    sta   nmi_status
    and   #VIA_INT_CB1
    beq   nmi_001
;
; CB1 interrupts are handled a bit unorthodoxly in that they are used to
; terminate a disk command, through a termination vector that is set up
; specially for each command. It is up to each handler to take care of
; clearing the interrupt status from the stack and returning properly.
nmi_cb1:
    lda   VIA_ORB             ; Clears the interrupt flag in the VIA
    lda   VIA_T2CL            ; Clear timer 2 interrupt flag
    pla
    jmp   (disk_vector)
;
; Check for more NMI events
nmi_001:
    lda   #VIA_INT_T2
    bit   nmi_status
    beq   nmi_done
; Timer 2 interrupt detected.
nmi_t2:
    inc   dsktimer            ; increment the disk on timer
    bne   nmi_t3
    inc   dsktimer+1
nmi_t3:
    lda   dsktimer            ; Check if timeout has been reached
    cmp   #$31
    bne   nmi_t4
    lda   dsktimer+1
    cmp   #$2
    bne   nmi_t4
    
    lda   VIA_T2CL            ; Clear interrupt flag
    lda   VIA_ORB             ; Stop motor and deselect drive
    ora   #(FDC_MOTOR_B | FDC_DRVSEL_B)
    sta   VIA_ORB
    lda   #$00
    sta   dskon               ; indicate motor is off
nmi_t4:
    lda   #$ff                ; Restart the timer for another round
    sta   VIA_T2CL
    sta   VIA_T2CH
;
; All nmi events handled
nmi_done:
    pla
    rti
;===============================================================================
;
; Skip any white spaces in the input buffer
;
;===============================================================================
skip_white:
    dex
sw_001
    inx
    lda   inbuffer,X
    cmp   #SPACE
    beq   sw_001
    cmp   #TAB
    beq   sw_001
    
    rts
;===============================================================================
;
; Parse the command in the terminal buffer
;
;===============================================================================
parse_command:
    lda   #command_table & $FF
    sta   parse_tab_ptr
    lda   #command_table / $100
    sta   parse_tab_ptr+1

    ldy   #0
    ldx   #0
pc001:
    lda   inbuffer,X          ; First skip any white space characters
    cmp   #SPACE
    beq   pc_skip
    cmp   #TAB
    beq   pc_skip
    bne   pc002
pc_skip:
    inx
    bne   pc001
pc002:
    lda   (parse_tab_ptr),Y
    cmp   inbuffer,X
    bne   pc_next_entry
    iny
    lda   (parse_tab_ptr),Y
    dey
    cmp   inbuffer+1,X
    bne   pc_next_entry
    lda   inbuffer+2,X
    cmp   #SPACE
    beq   pc_found_command
    cmp   #NULL
    beq   pc_found_command
pc_next_entry:
    iny
    iny
    iny
    iny                       ; Point to the next command entry
    lda   (parse_tab_ptr),Y   ; Check if there are more entries in that table
    beq   pc_command_not_found
    bne   pc002
    
pc_found_command:
    iny
    iny
    lda   (parse_tab_ptr),Y   ; Get function pointer to command
    sta   parse_jmp_vec
    iny
    lda   (parse_tab_ptr),Y
    sta   parse_jmp_vec+1
    
    inx
    inx
    inx                       ; Adjust ptr to point to next character
    jmp   (parse_jmp_vec)     ; Jump to the function
    
pc_command_not_found
    pstr  str_command_not_found
    rts    
;===============================================================================
;
; Select Drive Command
;
;===============================================================================
command_sd:
    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer
    lda   pfac
    sta   nxtdrive       ; Store the current drive
    rts
;===============================================================================
;
; Search for track 0 command
;
;===============================================================================
command_s0:
    jmp   fdcf_restore
;===============================================================================
;
; Start/Stop Motor Command
;   1 = Start Motor spinning
;   0 = Stop Motor
;   The drive will be selected at the same time the motor is started
;
;===============================================================================
command_sm:
    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer
    lda   pfac
    jmp   fdcf_set_motor
;===============================================================================
;
; Set Track Command
;
;===============================================================================
command_st:
    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer
    lda   pfac                ; Track to seek
    clc                       ; No head loading
    jsr   fdcf_seek
    rts
;===============================================================================
;
; Read Sector Command
;   This command will read the specified sector
;
;===============================================================================
command_rs:
    jsr   skip_white
    cmp   #NULL
    beq   crs_exit            ; Make sure there are some parameters

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer

    lda   #<disk_buffer       ; Set up disk buffer pointer
    sta   disk_buf_ptr
    lda   #>disk_buffer
    sta   disk_buf_ptr+1
    
    lda   pfac                ; sector to read
    sta   nxtsector
    lda   pfac+1
    sta   nxtsector+1
    
    jsr   fdcf_read_sect      ; Go read sector
    bcs   rs_001
    
crs_exit:
    rts
rs_001:
    jmp   print_status_error    
;===============================================================================
;
; Write Sector Command
;   This command will write the specified sector
;
;===============================================================================
command_ws:
    jsr   skip_white
    cmp   #NULL
    beq   cws_exit            ; Make sure there are some parameters

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer

    lda   #<disk_buffer       ; Set up disk buffer pointer
    sta   disk_buf_ptr
    lda   #>disk_buffer
    sta   disk_buf_ptr+1
    
    lda   pfac                ; sector to read
    sta   nxtsector
    lda   pfac+1
    sta   nxtsector+1
    
    jsr   fdcf_write_sect      ; Go read sector
    bcs   cws_001
    
cws_exit:
    rts
cws_001:
    jmp   print_status_error    
;===============================================================================
;
; Read the address information from a track
;
;===============================================================================
command_ra:
    lda   #<disk_buffer       ; Set up disk buffer pointer
    sta   disk_buf_ptr
    lda   #>disk_buffer
    sta   disk_buf_ptr+1

    lda   #%00100000
    sta   VIA_IER             ; While doing a track read we disable timer 2  
    
    jsr   fdcf_read_address
    bcs   cra_error
    
    jsr   via_setup_one_shot  ; Start timer 2 again    
    pstr  str_cra_001
    lda   #"$"
    jsr   putchar
    lda   disk_buffer
    jsr   print_hex_char

    pstr  str_cra_side
    lda   #"$"
    jsr   putchar
    lda   disk_buffer+1
    jsr   print_hex_char
    
    pstr  str_cra_length
    lda   #"$"
    jsr   putchar
    lda   disk_buffer+3
    jsr   print_hex_char
    
    pstr  str_cra_sector
    lda   #"$"
    jsr   putchar
    lda   disk_buffer+2
    jsr   print_hex_char
    jsr   crlf

    rts
cra_error:
    pha
    jsr   via_setup_one_shot  ; Start timer 2 again    
    pla
    jsr   print_status_error
    rts
str_cra_001:
    dc.b  CR, "Read address - Track no: ", 0
str_cra_side:
    dc.b  " Side: ", 0
str_cra_length:
    dc.b  " Sector Length: ", 0
str_cra_sector:
    dc.b  " Sector no: ", 0
;===============================================================================
;
; Read the entire track into memory, starting at $4000 and upwards
;
;===============================================================================
command_rt:
    jsr   skip_white
    cmp   #NULL
    beq   invalid_argument    ; Make sure there are some parameters

    lda   #<FORMAT_BUFFER
    sta   disk_buf_ptr
    lda   #>FORMAT_BUFFER
    sta   disk_buf_ptr+1

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer

    lda   pfac+1
    bne   invalid_argument    ; Make sure the track is within range
    lda   pfac                ; sector to read
    cmp   #80
    bcs   invalid_argument
    
    ldx   #%00100000
    stx   VIA_IER             ; While doing a track read we disable timer 2  

    jsr   fdcf_read_track     ; Go and read track
    bcs   crt_error
    
    jsr   via_setup_one_shot  ; Start timer 2 again    
    rts
crt_error:
    pha
    jsr   via_setup_one_shot  ; Start timer 2 again
    pla
    jmp   print_status_error
    
invalid_argument:
    pstr  str_invalid_argument
    rts
;===============================================================================
;
; Set Disk Side Command
;
;===============================================================================
command_ss:
    jsr   skip_white
    cmp   #NULL
    beq   invalid_argument   ; Make sure there are some parameters

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the coming string to an integer

    lda   pfac                ; sector to read
    sta   dskside

    rts
;===============================================================================
;
; Break execution and return to monitor
;
;===============================================================================
command_br:
    brk
;===============================================================================
;
; Format the disk
;
;===============================================================================
command_fd:
    pstr  str_format_skip     ; Get the skip factor (Interleave factor)
    jsr   get_string    
    ldx   #<inbuffer
    ldy   #>inbuffer
    jsr   strbin
    lda   pfac
    sta   skipbytes
    
    pstr  str_format_bytes    ; Get the number of bytes per sector
    jsr   get_string    
    ldx   #<inbuffer
    ldy   #>inbuffer
    jsr   strbin
    lda   pfac
    sta   secbytes

    pstr  str_format_sectors  ; Get the number of sectors per track
    jsr   get_string    
    ldx   #<inbuffer
    ldy   #>inbuffer
    jsr   strbin
    lda   pfac
    sta   sectrack
    
    jsr   crlf

    ldy   #0                  ; Start at track 0
    sty   trcnt
cmd_format_001:
    ; Even tracks
    lda   trcnt
    jsr   print_hex_char
    lda   #CR
    jsr   putchar
    tya
    ldx   #0
    jsr   fdcf_format_track
    inc   trcnt
    bcs   format_error
    
    ; Odd tracks
    lda   trcnt
    jsr   print_hex_char
    lda   #CR
    jsr   putchar
    tya
    ldx   #1
    jsr   fdcf_format_track
    inc   trcnt
    bcs   format_error

    iny
    cpy   #80
    bne   cmd_format_001
    
    jsr   via_setup_one_shot  ; Start timer 2 again
    jsr   crlf
    pstr  str_format_done
    rts
format_error:
    pha
    jsr   via_setup_one_shot  ; Start timer 2 again
    pla
    jmp   print_status_error
str_format_skip:
    dc.b  CR, "Sector Skip factor (1 - #sectors-1): ", 0
str_format_bytes:
    dc.b  CR, "# Bytes Per Sector (0=128, 1=256, 2=512, 3=1024): ", 0
str_format_sectors:
    dc.b  CR, "# Sectors per track: ", 0
str_format_error:
    dc.b  CR, "Error while formating the disk.", CR, 0
str_format_done:
    dc.b  CR, "Disk formatting done !", CR, 0
;===============================================================================
;
; Read 256 sectors from the disk, starting from 0
;
;===============================================================================

command_sc:
    pstr  str_scan_sectors
    jsr   get_string 
    ldx   #<inbuffer
    ldy   #>inbuffer
    jsr   strbin
    lda   pfac
    sta   numsctrs
    lda   pfac+1
    sta   numsctrs+1

    pstr  str_scan_interleave
    jsr   get_string
    ldx   #<inbuffer
    ldy   #>inbuffer
    jsr   strbin
    
    lda   #$00
    sta   nxtsector
    sta   nxtsector+1
    pstr  str_scan_disk
sc_001:
    lda   nxtsector
;    jsr   print_hex_char
;    lda   #CR
;    jsr   putchar
    jsr   fdcf_read_sect

    clc
    lda   nxtsector
    adc   pfac
    sta   nxtsector
    lda   nxtsector+1
    adc   #$00
    sta   nxtsector+1
    sec
    lda   numsctrs
    sbc   #1
    sta   numsctrs
    lda   numsctrs+1
    sbc   #0
    sta   numsctrs+1
    ora   numsctrs
    bne   sc_001

    jsr   crlf
    
    rts
str_scan_disk:
    dc.b  CR, "Scanning first 256 sectors of the disk", CR,0
str_scan_interleave:
    dc.b  CR, "Number of sectors in each step ? ", 0
str_scan_sectors:
    dc.b  CR, "Number of sectors to scan ? ", 0
;===============================================================================
;
; Display the memory buffer
;
;===============================================================================
command_sb:
    pstr  str_mem_buf_001
    
    lda   #<disk_buffer       ; Set up disk buffer pointer
    sta   disk_buf_ptr
    lda   #>disk_buffer
    sta   disk_buf_ptr+1
    
    ldy   #$00
csb_001:
    tya
    and   #$0f
    bne   csb_002
    jsr   crlf
    tya
    jsr   print_hex_char
    lda   #SPACE
    jsr   putchar
    jsr   putchar
csb_002:
    lda   (disk_buf_ptr),Y
    jsr   print_hex_char
    lda   #SPACE
    jsr   putchar
    iny
    bne   csb_001
    jsr   crlf    
    rts
str_mem_buf_001:
    dc.b  CR, "Memory Buffer Dump", CR
    dc.b  "===================================================", CR
    dc.b  "    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F", 0
;===============================================================================
;
; Fill memory buffer with value
;
;===============================================================================
command_fb:
    jsr   skip_white
    cmp   #NULL
    beq   cfb_invalid_arg     ; Make sure there are some parameters

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the arg to an integer
    bcs   cfb_invalid_arg

    lda   #<disk_buffer       ; Set up disk buffer pointer
    sta   disk_buf_ptr
    lda   #>disk_buffer
    sta   disk_buf_ptr+1

    lda   pfac                ; sector to read
    ldy   #$00
cfb_001
    sta   (disk_buf_ptr),Y
    iny
    bne   cfb_001
    rts
cfb_invalid_arg:
    pstr  str_invalid_argument
    rts
;===============================================================================
;
; Download file to memory
;
;===============================================================================
command_df:
    jsr   skip_white
    cmp   #NULL
    beq   cfb_invalid_arg     ; Make sure there are some parameters

    txa
    clc
    adc   #<inbuffer
    tax
    lda   #0
    adc   #>inbuffer
    tay
    jsr   strbin              ; Convert the arg to an integer
    bcs   cfb_invalid_arg
    
    lda   pfac
    ldx   pfac+1
    jsr   XModem
    
    rts

;===============================================================================
;
; Just for testing
;
;===============================================================================
command_tc:
    jsr   calc_interleave
    rts

;===============================================================================
;
; Print an error message depending on the bit settings in Acc
;
;===============================================================================
print_status_error:
    sta   error
    lda   #FDC_REC_NF
    bit   error
    bne   pse_record_not_found
    lda   #FDC_DAT_LOST
    bit   error
    bne   pse_data_lost
    lda   #FDC_CRC_ERR
    bit   error
    bne   pse_crc_error
    lda   #FDC_WR_PROT
    bit   error
    bne   pse_write_protected
    lda   #$80
    bit   error
    bne   pse_internal_error
    lda   #$20
    bit   error
    bne   pse_format_not_supported
    rts
pse_record_not_found:
    pstr  str_record_not_found
    rts
pse_data_lost:
    pstr  str_data_lost
    lda   #"$"
    jsr   putchar
    lda   error_offs
    jsr   print_hex_char
    jmp   crlf
pse_crc_error:
    pstr  str_crc_error
    rts
pse_write_protected:
    pstr  str_write_protected
    rts
pse_internal_error:
    pstr  str_internal_error
    rts
pse_format_not_supported:
    pstr  str_format_not_supported
    rts

;===============================================================================
;
; String section
;
;===============================================================================    
start_string:
    dc.b  CR, "Floppy Disk Monitor firmware v. 0.01", CR, CR
    dc.b  "Available Commands:", CR
    dc.b  "SD <n> - Select drive       SM <n> - Start motor", CR
    dc.b  "ST <n> - Seek track         SS <n> - Set disk side", CR
    dc.b  "S0 - Seek track 00          RT <n> - Read Track", CR
    dc.b  "RA - Read Track Address     BR - Break Execution", CR
    dc.b  "RS <n> - Read Sector        WS <n> - Write Sector", CR
    dc.b  "FD - Format Disk            ", CR
    dc.b  "------------------ Memory Commands ------------------", CR
    dc.b  "SB - Show disk buffer       FB <n> - Fill disk buffer", CR
    dc.b  "SC - Scan 256 sectors       DF <addr> - Download file", CR, 0
str_command:
    dc.b  CR, "Command: ", $00
str_command_not_found:
    dc.b  CR, "Not a valid command !", CR, 0
str_invalid_argument:
    dc.b  CR, "Invalid argument !", CR, 0
str_disk_info:
    dc.b  CR, "Evaluating drive parameter algorithm !",CR,0
str_test_via
    dc.b  CR, "Testing via timer 2, timeout at: ",0
str_sect_error:
    dc.b  CR, "Error trying to read sector ", 0
str_record_not_found:
    dc.b  CR, "The requested record (track/side/sector) was not found.", CR, 0
str_data_lost:
    dc.b  CR, "The computer is not fast enough to read/write data. Offset: ", 0
str_crc_error:
    dc.b  CR, "A CRC error was detected on the diskette.", CR, 0
str_write_protected:
    dc.b  CR, "The disk is write protected.", CR, 0
str_internal_error:
    dc.b  CR, "An internal error occured.", CR, 0
str_format_not_supported:
    dc.b  CR, "Format not supported.", CR, 0
    
command_table:
    dc.b  "SD"              ; Select Drive
    dc.w  command_sd
    dc.b  "SM"              ; Start motor
    dc.w  command_sm
    dc.b  "ST"              ; Set Track
    dc.w  command_st
    dc.b  "SS"              ; Set Disk side
    dc.w  command_ss
    dc.b  "S0"              ; Search for track 0
    dc.w  command_s0
    dc.b  "RS"              ; Read sector <n> into memory (0300)
    dc.w  command_rs
    dc.b  "WS"              ; Write sector <n> from memory (0300)
    dc.w  command_ws
    dc.b  "BR"              ; Read sector <n> into memory
    dc.w  command_br
    dc.b  "RA"              ; Read Track Address into memory
    dc.w  command_ra
    dc.b  "RT"              ; Read Entire track into memory
    dc.w  command_rt
    dc.b  "FD"              ; Format entire diskette
    dc.w  command_fd
    dc.b  "SC"              ; Scan sectors
    dc.w  command_sc
    dc.b  "SB"              ; Show disk buffer
    dc.w  command_sb
    dc.b  "FB"              ; Fill disk buffer
    dc.w  command_fb
    dc.b  "DF"              ; Download file from host computer
    dc.w  command_df
    dc.b  "TC"              ; Test command
    dc.w  command_tc
    dc.b  0                 ; Terminate table    
        
    END
        