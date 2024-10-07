; main program to be run 
$NOLIST
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;

;----------------clock values---------------------
CLK               EQU 16600000 ; Microcontroller system frequency in Hz
BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RATE   		EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)

TIMER0_RELOAD 		EQU ((65536-(CLK/TIMER0_RATE)))
TIMER2_RATE         EQU 1000    ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD   	EQU ((65536-(CLK/TIMER2_RATE)))

PAGE_ERASE_AP   EQU 00100010b 	;for flash memory
BYTE_PROGRAM_AP EQU 00100001b

ORG 0x0000
	ljmp MainProgram

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
	
; Timer/Counter 2 overflow interrupt vector
org 0x002B
	ljmp Timer2_ISR


;               1234567890123456    <- This helps determine the location of the counter
param1:     db 'SOAK TEMP:      ', 0
param2:     db 'SOAK TIME:      ', 0
param3:     db 'REFLOW TEMP:    ', 0
param4:     db 'REFLOW TIME:     ', 0
startmsg:     db '->START 2 START  ', 0
editmsg:     db '->EDIT 2 EDIT    ', 0
title06:     db 'CONFIRM?           ', 0
title07:     db 'REFLOW TIME?       ', 0
title08:     db 'CONFIRM?           ', 0
title1:     db '1 RAMP   Ts:       ', 0
title2:     db '2 SOAK             ', 0
title3:     db '3 RAMP   Tr:       ', 0
title4:     db '4 REFLOW           ', 0
title5:     db '5 COOLING          ', 0
blank:     db '                  ', 0

cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3

PWM_OUT     equ P1.0 ; logic=1 oven on
SOUND_OUT	equ P1.6

;-------------------------------------------------------
DSEG at 0x30

;---------------temp validation----------------------

x:   ds 4
y:   ds 4
bcd: ds 5
VLED_ADC: ds 2


;----------------- clock and speaker ------------------------------
Count1ms:               ds 2 ;to determine when one second has passed
secs_ctr:               ds 1
mins_ctr:               ds 1


state_secs_ctr:			ds 2

buzzer:					ds 1

;-----------------FSM-s-------------------
FSM1_state:             ds 1 ; determine state
edit_state:				ds 1

;----------------fsm values--------------------
stemp: 					ds 2
stime: 					ds 2
rtemp: 					ds 2
rtime:					ds 2
stime_bcd:				ds 2
rtime_bcd:				ds 2

;-----------------timer values-----------------
soak_timer:             ds 2
reflow_timer:           ds 1

;----------------PWM-----------------
pwm_counter:            ds 1
pwm:                    ds 1

;------------------oven temp-------------
oven_temp:				ds 2


;-------------------flags-----------------------------
BSEG 
mf: dbit 1
one_second_flag: dbit 1

;----------------push buttons------------------
incr:			dbit 1
decr:  			dbit 1
edit:			dbit 1
reset:			dbit 1
start_stop:		dbit 1

;---------------soak and reflow time flag-------------
soak_timer_flag:    dbit 1
reflow_timer_flag:  dbit 1
soak_temp_flag:    dbit 1
reflow_temp_flag:  dbit 1
;--------------PWM flag---------------------
pwm_flag:           dbit 1

$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$include(math32.inc)
$LIST
;---------flash memory functions-------------
putchar:
    JNB TI, putchar
    CLR TI
    MOV SBUF, a
    RET

SendString:
    CLR A
    MOVC A, @A+DPTR
    JZ SSDone
    LCALL putchar
    INC DPTR
    SJMP SendString
SSDone:
    ret

; Sends the byte in the accumulator to the serial port in decimal 
Send_byte:
	mov b, #100
	div ab
	orl a, #'0'
	lcall putchar
	mov a, b
	mov b, #10
	div ab
	orl a, #'0'
	lcall putchar
	mov a, b
	orl a, #'0'
	lcall putchar
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	ret

;---------------------------------;
; Routine to initialize the pins  ;
; for input out - ADC inc         ;
;---------------------------------;
Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	mov SCON, #52H
	
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	
	; Since the reset button bounces, we need to wait a bit before
    ; sending messages, otherwise we risk displaying gibberish!
	mov R1, #200
    mov R0, #104
    djnz R0, $   ; 4 cycles->4*60.285ns*104=25us
    djnz R1, $-4 ; 25us*200=5.0ms
	
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10000001 ; Activate AIN0 and AIN7 analog inputs
	orl ADCCON1, #0x01 ; Enable ADC
	
	ret

;-----------functions for temp reading ADC-----------
Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret

Send_BCD mac
	push ar0
	mov r0, %0
	lcall ?Send_BCD
	pop ar0
	endmac
	
	?Send_BCD:
	push acc
	; Write most significant digit
	mov a, r0
	swap a
	anl a, #0fh
	orl a, #30h
	lcall putchar
	; write least significant digit
	mov a, r0
	anl a, #0fh
	orl a, #30h
	lcall putchar
	pop acc
	ret

	SendChar mac
	push acc
	mov a,%0
	lcall putchar
	pop acc
	endmac

Display_formated_BCD:
	Set_Cursor(2, 1)
	Display_BCD(bcd+3)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)

	ret
;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 0                     ;
;---------------------------------;
Timer0_Init:
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz wave at pin SOUND_OUT   ;
;---------------------------------;
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	; Timer 0 doesn't have 16-bit auto-reload, so
	clr TR0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	setb TR0
	cpl SOUND_OUT 	
	reti

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;
Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.z
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	orl T2MOD, #0x80 ; Enable timer 2 autoreload
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
	; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Enable the timer and interrupts
	orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    ;setb TR2  ; Enable timer 2
	ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1
	;mov a, buzzer
	;cjne a, #0, buzzer_out

pwm_counts:
    
    inc pwm_counter
    clr c
    mov a, pwm
    subb a, pwm_counter ;If pwm_counter <= pwm then c=1
	cpl c
	mov PWM_OUT, c
	
    mov a, pwm_counter
	cjne a, #100, Inc_Done
	mov pwm_counter, #0


Inc_Done:
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), jumper ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_done
	
	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb one_second_flag; Let the main program know half second had passed
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Increment the BCD counter
	mov a, secs_ctr
	;jnb UPDOWN, Timer2_ISR_decrement
	add a, #0x01
	sjmp clkup
	
jumper:
	ljmp Timer2_ISR_done

; clkup:
; 	mov a, state_secs_ctr						;variables for state timers
; 	add a, #1
;	da a
; 	mov state_secs_ctr, a
; 	mov a, secs_ctr
; 	add a, #1
; 	da a ; Decimal adjust instruction.  Check datasheet for more details!
; 	mov secs_ctr, a
; 	cjne a, #0x60, Timer2_ISR_done
; 	mov secs_ctr, #0x00
; 	mov a, mins_ctr
; 	add a, #1
; 	da a
; 	mov mins_ctr, a

clkup:
	mov a, state_secs_ctr	
	inc a	;variables for state timers
	da a
	mov state_secs_ctr, a
	mov a, secs_ctr
	add a, #0x01
	da a
	mov secs_ctr, a
	cjne a, #0x60, Timer2_ISR_done
	mov secs_ctr, #0x00
	mov a, mins_ctr
	inc a					 ; Decimal adjust instruction.  Check datasheet for more details!
	da a
 	mov mins_ctr, a


Timer2_ISR_done:
	pop psw
	pop acc
	reti


;-------------------Generate Display--------------------------
GenerateDisplay:

MtimerDisplay:

    Set_Cursor(2, 15)
    Display_BCD(secs_ctr)
    Set_Cursor(2, 14)
    Display_char(#':')
    Set_Cursor(2, 12)
    Display_BCD(mins_ctr)

TempDisplay:
    Set_Cursor(2, 1)
	; Read the 2.08V LED voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VLED_ADC+0, R0
	mov VLED_ADC+1, R1

	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(20740) ; The MEASURED LED voltage: 2.074V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LED value
	mov y+0, VLED_ADC+0
	mov y+1, VLED_ADC+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32
	

	Load_y (74)
	lcall mul32
	Load_y (207000)  
	lcall add32
    ; x is vCH 
    ; Load_y(27300) ;load 2.73V into y
    ; lcall sub32
    ; Load_y(100)
    ;lcall mul32
    ;x is now T
	; Convert to BCD and display
	lcall hex2bcd
	lcall Display_formated_BCD
	
    Send_BCD(bcd+3)
    Send_BCD(bcd+2)
    SendChar(#'.')
    Send_BCD(bcd+1)
    Send_BCD(bcd+0)
    SendChar(#'\n')

	mov oven_temp+0, bcd+2			;saving the oven temp
	mov oven_temp+1, bcd+3

	ret

;-----------------lcd pushbuttons--------------------------
LCD_PB:
	; Set variables to 1: 'no push button pressed'
	setb incr
	setb decr
	setb edit
	setb start_stop
	setb reset
	; The input pin used to check set to '1'
	setb P1.5
	
	; Check if any push button is pressed
	clr P0.0
	clr P0.1
	clr P0.2
	clr P0.3
	clr P1.3
	jb P1.5, LCD_PB_Done

	; Debounce
	mov R2, #50
	lcall waitms
	jb P1.5, LCD_PB_Done

	; Set the LCD data pins to logic 1
	setb P0.0
	setb P0.1
	setb P0.2
	setb P0.3
	setb P1.3
	
	; Check the push buttons one by one
	clr P1.3
	mov c, P1.5
	mov reset, c
	setb P1.3

	clr P0.0
	mov c, P1.5
	mov start_stop, c
	setb P0.0
	
	clr P0.1
	mov c, P1.5
	mov edit, c
	setb P0.1
	
	clr P0.2
	mov c, P1.5
	mov decr, c
	setb P0.2
	
	clr P0.3
	mov c, P1.5
	mov incr, c
	setb P0.3

LCD_PB_Done:		
	ret

wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD)
	mov	TL0,#low(TIMER0_RELOAD)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret

;--------------------------------- Main Program --------------------------------

MainProgram:                            ;main program
    
    mov SP, #0x7f
    lcall Init_All
    lcall LCD_4BIT                                        ; initialize settings

    clr one_second_flag     
    clr soak_timer_flag                               ; flags
    clr reflow_timer_flag
    clr pwm_flag

    mov FSM1_state, #0
	mov edit_state, #0
	                                    ; fsm
    mov secs_ctr, #0                                    ; clock
    mov mins_ctr, #0
    mov soak_timer, #0
    mov reflow_timer, #0
	mov buzzer, #0
	mov pwm_counter, #0
                                        ; edit settings
    ;lcall Timer1_Init                                    ; timers
    lcall Timer2_Init
	
	setb TR2
    setb EA



    mov stemp+0, #0x50
	mov stemp+1, #0x01
	mov stime, #0x60
	mov a, stime
	da a 
	mov stime_bcd, a
	mov rtemp+0, #0x17
    mov rtemp+1, #0x02
	mov rtime, #0x45
	mov a, rtime
	da a 
	mov rtime_bcd, a
    
                                        ; 

forever:  
    jnb one_second_flag, SkipDisplay            ;every second, the screen should update
    clr one_second_flag
    lcall GenerateDisplay 
	

SkipDisplay:                                    ;if no display needed
	 					 
	ljmp FSM1
;-----------------------------------------------------------------

;-----------------------check_stop_restart-----------------------
check_stop_restart:
	lcall LCD_PB				;evaluate the buttons	;need to wait so dont need lightning button presses
	mov c, start_stop
	jnc stop_protocol
	mov c, reset
	jnc reset_protocol
	ret

stop_protocol:
	mov pwm, #0
	mov FSM1_state, #0
	ret

reset_protocol:
	mov pwm, #0					;turn off oven
	mov FSM1_state, #0			;go back to select stage
	ret


;----------------------------load functions to compare values------
load_soak_temp:
	push acc
	push psw
	clr mf
	clr A

	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a
	

	mov x+0, stemp+0
	mov x+1, stemp+1
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1
	
	pop psw
	pop acc
	ret

load_reflow_temp:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a
	

	mov x+0, rtemp+0
	mov x+1, rtemp+1
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1
	
	pop psw
	pop acc
	ret

load_abort_temp:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a

	mov x+0, #0x50
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1

	pop psw
	pop acc
	ret

load_cooling_temp:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a

	mov x+0, #0x50
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1

	pop psw
	pop acc
	ret



;--------------------------------- FSM --------------------------------
FSM1:
    mov a, FSM1_state 				; initializes which state to start from

FSM1_state0: 						; start or not and edit settings
    cjne a, #0, FSM1_state1 		; if state is not 0, then move to the next
    ljmp FSM_edit
    mov pwm, #0 					; pulse width modulation set to 0
	mov secs_ctr, #0

FSM1_state0_done:
    WriteCommand(#0x01)
    Wait_Milli_Seconds(#2)
	mov FSM1_state, #1      		; change the state to the next
	mov secs_ctr, #0
	mov mins_ctr, #0
	mov buzzer, #1
	
    ljmp forever

check_abort: 
	lcall load_abort_temp		;if temp<50 after a min, abort.
	lcall x_gt_y
	jb mf, abort_process
	ret

abort_process:
	mov pwm, #0
	mov FSM1_state, #0
	ljmp forever

FSM1_state1: 						; ramp to soak
    cjne a, #1, FSM1_state2
    Set_Cursor(1, 1)
    Send_Constant_String(#title1)
	Set_Cursor(1, 13)
	Display_BCD(stemp+1)
	Display_BCD(stemp+0)
    mov pwm, #100 					; set power to full
	mov a, #0x00
	cjne a, mins_ctr, check_abort
	lcall load_soak_temp			;soak temp in x , oven in y
	lcall x_gteq_y
	jb mf, FSM1_state1_done

    mov FSM1_state, #2
	WriteCommand(#0x01)				; clear thje tiomer area
	clr soak_temp_flag
    setb soak_timer_flag    		; start the timer for soak
	mov state_secs_ctr, #0 			;set the timer for states
;-------------------------------abort subroutines---------------------


FSM1_state1_done:
	mov buzzer, #2
    ljmp forever

FSM1_state2: ;soak
    cjne a, #2, FSM1_state3
    Set_Cursor(1, 1)
    Send_Constant_String(#title2)
	Set_Cursor(1, 13)
	Display_BCD(state_secs_ctr)
    mov pwm, #20
    mov a, state_secs_ctr
	clr c 
	subb a, stime
	jc FSM1_state2_done

    mov FSM1_state, #3
	clr soak_timer_flag
	WriteCommand(#0x01)			; clear thje tiomer area

FSM1_state2_done:

    ljmp forever

FSM1_state3: 						;ramp to reflow
    cjne a, #3, FSM1_state4
    Set_Cursor(1, 1)
    Send_Constant_String(#title3)
	Set_Cursor(1, 13)
    Display_BCD(rtemp+1)
	Display_BCD(rtemp+0)
    mov pwm, #100 					;ramp to temp
	lcall load_reflow_temp			;soak temp in x , oven in y
	lcall x_gteq_y
	jb mf, FSM1_state3_done

    mov FSM1_state, #4
	mov state_secs_ctr, #0			;resetting the state_sec_ctr

FSM1_state3_done:
	mov buzzer, #4
    ljmp forever

FSM1_state4: 						;reflow
    cjne a, #4, FSM1_state5
    Set_Cursor(1, 1)
    Send_Constant_String(#title4)
	Set_Cursor(1,13)
	Display_BCD(state_secs_ctr)
    mov pwm, #20 					;keep temp constant
    
	mov a, state_secs_ctr
	clr c 
	subb a, rtime
	jc FSM1_state4_done

    mov FSM1_state, #5


FSM1_state4_done:
	mov buzzer, #5
    ljmp forever

FSM1_state5: 						;cooling
    cjne a, #5, FSM1_state0_buffer
    Set_Cursor(1, 1)
    Send_Constant_String(#title5)
    mov pwm, #0
	lcall load_cooling_temp			;50 in x, oven temp in y
	lcall x_gteq_y
	jnb mf, FSM1_state5_done

    mov FSM1_state, #0


FSM1_state5_done:
	mov buzzer, #6
    ljmp forever

FSM1_state0_buffer: ;buffer to long jump to state0

    ljmp FSM1_state0

;-----------------------------------------------------------------

;---------------------FSM_edit--------------------
FSM_edit:
	mov a, edit_state 

FSM_soaktemp:
	cjne a, #0, FSM_soaktime
	ljmp soaktemp
	

FSM_soaktemp_done:
	mov edit_state, #1
	ljmp FSM_edit

FSM_soaktime:
	cjne a, #1, FSM_reflowtemp
	call soaktime

	

FSM_soaktime_done:
	mov edit_state, #2
	ljmp FSM_edit

FSM_reflowtemp:
	cjne a, #2, FSM_reflowtime
	ljmp reflowtemp

	

FSM_reflowtemp_done:
	mov edit_state, #3
	ljmp FSM_edit

FSM_reflowtime:
	cjne a, #3, FSM_start
	ljmp reflowtime

	

FSM_reflowtime_done:
	mov edit_state, #4
	ljmp FSM_edit

FSM_start: 						;can only start once all parameters are selected
	cjne a, #4, FSM_soaktemp
	lcall LCD_PB
	Wait_Milli_Seconds(#75)
	Set_Cursor(1, 1)            ;messages for starting or editing
    Send_Constant_String(#startmsg)
    Set_Cursor(2, 1)            ;messages for starting or editing
    Send_Constant_String(#editmsg)
	mov c, start_stop			;start the process
	jnc FSM1_state0_done_buffer
	mov c, edit					;keep on editing
	jnc FSM_start_done
	ljmp FSM_start


FSM1_state0_done_buffer:
	ljmp FSM1_state0_done

FSM_start_done:
	mov edit_state, #0          ;clear lcd
    WriteCommand(#0x01)
    Wait_Milli_Seconds(#2)
	ljmp FSM_edit

;--------------edit fsm state functions----------------

; Assumes A register contains the value to be displayed before calling
; This routine converts the binary value in A to ASCII and displays it on the LCD
       
soaktemp:
	lcall LCD_PB			;evaluate the buttons
	Wait_Milli_Seconds(#75)	;need to wait so dont need lightning button presses
	lcall stemp_change

Display_soaktemp:
	Set_Cursor(1, 1)					;displaying the values.
	Send_Constant_String(#param1)
	Set_Cursor(2, 1)
	Display_BCD(stemp+1)
	Display_BCD(stemp+0)
	
	mov c, edit				;since is zero when pushed, if carry is on then replay
	jc soaktemp
	ljmp FSM_soaktemp_done

stemp_change:
    
	mov c, incr				;if increment
	jnc add_stemp
	mov c, decr				;then check if decrement
	jnc sub_stemp
	ret

add_stemp:
    clr c 
	mov a, stemp+0
    addc a, #0x30
    jc Display_soaktemp
    mov a, stemp+0              ;need to re add since changed value
	add a, #0x01
	da a
	mov stemp+0, a
	ret

sub_stemp:
	clr c
    mov a, stemp+0
    subb a, #0x31
    jc Display_soaktemp
    mov a, stemp+0
	add a, #0x99
	da a
	mov stemp+0, a
	ret

soaktime:
	lcall LCD_PB			;evaluate the buttons
	Wait_Milli_Seconds(#75)
	lcall stime_change

Display_soaktime:
	Set_Cursor(1, 1)					;displaying the values.
	Send_Constant_String(#param2)
	Set_Cursor(2, 1)
	Display_BCD(#0)
    Display_BCD(stime)

	mov c, edit				;since is zero when pushed, if carry is on then replay 
	jc soaktime
	ljmp FSM_soaktime_done

stime_change:
	mov c, incr				;if increment
	jnc add_stime
	mov c, decr				;then check if decrement
	jnc sub_stime
	ret

add_stime:
    mov a, stime+0              ;need to re add since changed value
	add a, #0x01
	da a
	mov stime+0, a
	inc stime_bcd
	ret

sub_stime:
    mov a, stime+0
	add a, #0x99
	da a
	mov stime+0, a
	dec stime_bcd
	ret


reflowtemp:
	lcall LCD_PB			;evaluate the buttons
	Wait_Milli_Seconds(#75)
	lcall rtemp_change
	
Display_reflowtemp:
	Set_Cursor(1, 1)					;displaying the values.
	Send_Constant_String(#param3)
	Set_Cursor(2, 1)
	Display_BCD(rtemp+1)
    Display_BCD(rtemp+0)

	mov c, edit				;since is zero when pushed, if carry is on then replay 
	jc reflowtemp
	ljmp FSM_reflowtemp_done

rtemp_change:
	mov c, incr				;if increment
	jnc add_rtemp
	mov c, decr				;then check if decrement
	jnc sub_rtemp
	ret

add_rtemp:
    mov a, rtemp+0              ;need to re add since changed value
	add a, #0x01
	da a
	mov rtemp+0, a
	ret

sub_rtemp:
    mov a, rtemp+0
	add a, #0x99
	da a
	mov rtemp+0, a
	ret

reflowtime:
	lcall LCD_PB			;evaluate the buttons
	Wait_Milli_Seconds(#75)
	lcall rtime_change
	
Display_reflowtime:
	Set_Cursor(1, 1)					;displaying the values.
	Send_Constant_String(#param4)
	Set_Cursor(2, 1)
	Display_BCD(#0x00)
    Display_BCD(rtime)

	mov c, edit				;since is zero when pushed, if carry is on then replay 
	jc reflowtime
	ljmp FSM_reflowtime_done

rtime_change:
	mov c, incr				;if increment
	jnc add_rtime
	mov c, decr				;then check if decrement
	jnc sub_rtime
	ret

add_rtime:
    mov a, rtime+0              ;need to re add since changed value
	add a, #0x01
	da a
	mov rtime+0, a
	inc rtime_bcd
	ret

sub_rtime:
    mov a, rtime+0
	add a, #0x99
	da a
	mov rtime+0, a
	inc rtime_bcd
	ret
;------------------------------------------------------

END