/* 
										Slave_main.asm :

Description of hardware connection :
	IRQ PB0
	SPI : PB7:4
	KEYPAD : PD7:0
	LCD : PA7:0
	BARLED : PC7:0

Summary of subroutines :
	MAIN which include MAIN_LOOP
	INITIALIZATION which include INIT_ALL and all init ports except specific ports
	SPI which include INIT_SSPI, SPI_PROTOCOL and SSPI_TRANSMIT
	PROTOCOL to send value from KEYPAD_to_LCD
	KEYPAD which include INIT_KEYPAD and SCANKEY
	LCD which include all LCD command : LCD_ON, STRING_OUT, CMDWRITE and DATAWRITE
	DELAY which include SDELAY, delay_100us, 2ms, 10ms and 20ms used for LCD protocol
*/

.ORG 0
	JMP MAIN

; LCD configuration
.EQU LCD_PORT = PORTA ; LCD DATA PORT
.EQU LCD_DDR = DDRA ; LCD DATA DDR
.EQU LCD_PIN = PINA ; LCD DATA PIN
.EQU LCD_RS = 0 ; LCD RS
.EQU LCD_RW = 1 ; LCD RW
.EQU LCD_EN = 2 ; LCD EN

; BARLED configuration
.EQU LED_PORT = PORTC ; LED DATA PORT
.EQU LED_PIN = PINC ; LED DATA PIN
.EQU LED_DDR = DDRC ; LED DATA DDR

; KEYPAD configuration
.EQU KEY_PORT = PORTD ; KEY DATA PORT
.EQU KEY_PIN = PIND ; KEY DATA PIN
.EQU KEY_DDR = DDRD ; KEY DATA DDR

; SPI configuration
.EQU PIN_SS = 4
.EQU PIN_MOSI = 5
.EQU PIN_MISO = 6
.EQU PIN_SCK = 7
.EQU SPI_DDR = DDRB
.EQU SPI_PORT = PORTB
.EQU SPI_PIN = PINB

; IRQ configuration
.EQU IRQ_DDR = DDRB
.EQU IRQ_PORT = PORTB
.EQU IRQ_PIN = PINB
.EQU IRQ_PIN_number = 0

; register definition
.DEF TEMP = R16 ; free register
.DEF DATA_TEMP = R17 ; value receive from SPI and send to UART
.DEF LCD_TEMP = R20 ; value to write on the LCD
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MAIN:
        LDI R21, LOW(RAMEND) ; set up stack pointer
		OUT SPL, R21
		LDI R21, HIGH(RAMEND)
		OUT SPH, R21

		CALL INIT_ALL
		CALL STRING_OUT ; display "KEY PRESSED :" on LCD
 
	MAIN_LOOP:
        CALL SCANKEY ; scan keypad and send data if something is pressed
		CALL KEYPAD_to_LCD ; display on LCD (and BARLED)
	RJMP MAIN_LOOP
 
;;;;;;;;;;;;;;;;;;;;;;;;INITIALIZATION;;;;;;;;;;:;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_ALL :
		CALL INIT_BARLED
		CALL INIT_KEYPAD
		CALL LCD_ON
		CALL INIT_IRQ
		CALL INIT_SSPI ; INITIALIZE SLAVE SPI
	RET

INIT_BARLED :
	PUSH TEMP
		LDI TEMP, 0xFF
		OUT LED_DDR, TEMP ; SET PORTC AS OUTPUT TO LEDBAR
	POP TEMP
	RET

INIT_IRQ :
		; SET IRQ_PIN as output
		SBI IRQ_DDR, IRQ_PIN_number
		SBI IRQ_PORT, IRQ_PIN_number ; IRQ rest state is HIGH
	RET
 
;;;;;;;;;;;;;;;;;;;;;;;SPI SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_SSPI:
	PUSH TEMP
        ; ENABLE SPI SLAVE
		LDI TEMP, (1 << PIN_MISO) ; all SPI_PIN are input except MISO
		IN TEMP, SPI_DDR
		ORI TEMP, 0b11110000 ; change only SPI_PIN direction
		OUT SPI_DDR, TEMP
		LDI TEMP, (1 << SPE0) ; Enable SPI operation
		OUT SPCR0, TEMP
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SPI_PROTOCOL:
		MOV DATA_TEMP, R23 ; DATA_TEMP is the GPR reserved for SPI communication
		CBI IRQ_PORT, IRQ_PIN_number ; IRQ to LOW
		CALL SSPI_TRANSMIT
		SBI IRQ_PORT, IRQ_PIN_number ; IRQ to HIGH
	RET
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SSPI_TRANSMIT:
	PUSH TEMP
		; SLAVE SPI DATA TRANSMISSION
		OUT SPDR0, DATA_TEMP
	WAIT_TRANSMIT:
		; WAIT TRANSMISSION COMPLETE
		IN TEMP, SPSR0
		SBRS TEMP, SPIF0
		RJMP WAIT_TRANSMIT
	POP TEMP
	RET

;;;;;;;;;;;;;PROTOCOL to send value from keypad to LCD;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
KEYPAD_to_LCD :
		LDI LCD_TEMP, 0xC0 ; FORCE CURSON TO BEGIN 2ND ROW
		CALL CMDWRITE
		CPI R23, 0xFF ; check if no key is pressed
		BREQ NOKEY
		CALL SPI_PROTOCOL ; SPI sending pressed value
		CPI R23, 10
		BRLO DIGIT
		LDI LCD_TEMP, 55
		ADD LCD_TEMP, R23 ; add to convert the decimal to ASCII
		RJMP DISPLAY
	DIGIT:
		LDI LCD_TEMP, 48
		ADD LCD_TEMP, R23 ; add to convert the decimal to LCD
	DISPLAY:
		CALL DATAWRITE
	NOKEY:
		LDI LCD_TEMP, 32 ; SPACE
		CALL DATAWRITE
		LDI LCD_TEMP, 32 ; SPACE
		CALL DATAWRITE
		OUT LED_PORT, R23
	RET

;;;;;;;;;;;;;;;;;;KEY PAD SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_KEYPAD :
	PUSH TEMP
		LDI TEMP, 0x0F
		OUT KEY_DDR, TEMP ; SET PD0-3 OUTPUT COLUMN, PD4-7 INPUT ROW
		LDI TEMP, 0xF0
		OUT KEY_PORT, TEMP ; PULL-UP
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; R23 : column number then it return the key code so don't push/pop it !
; R24 : row number
; R22 : output the scanning column
SCANKEY:
	PUSH R22
	PUSH R24
		LDI R22, 0b11110111
		LDI R23, 0
		LDI R24, 3
	KEYPAD_SCAN_LOOP:
			OUT KEY_PORT, R22
			CALL DELAY_10ms
			SBIC KEY_PIN, 4
			RJMP KEYPAD_SCAN_CHECK_COL_2
			RJMP KEYPAD_SCAN_FOUND
		KEYPAD_SCAN_CHECK_COL_2:
			SBIC KEY_PIN, 5
			RJMP KEYPAD_SCAN_CHECK_COL_3
			LDI R23, 1
			RJMP KEYPAD_SCAN_FOUND
		KEYPAD_SCAN_CHECK_COL_3:
			SBIC KEY_PIN, 6
			RJMP KEYPAD_SCAN_CHECK_COL_4
			LDI R23, 2
			RJMP KEYPAD_SCAN_FOUND
		KEYPAD_SCAN_CHECK_COL_4:
			SBIC KEY_PIN, 7
			RJMP KEYPAD_SCAN_NEXT_ROW
			LDI R23, 3
			RJMP KEYPAD_SCAN_FOUND
	KEYPAD_SCAN_NEXT_ROW:
		CPI R24, 0
		BREQ KEYPAD_SCAN_NOT_FOUND
		SEC ; Set carry flag to keep all column port set
		ROR R22
		DEC R24
		RJMP KEYPAD_SCAN_LOOP

	KEYPAD_SCAN_FOUND:
		LSL R23
		LSL R23
		ADD R23, R24 ; calculate keycode in R23
	POP R24
	POP R22
	RET
	KEYPAD_SCAN_NOT_FOUND: ; no key is pressed
		LDI R23, 0xFF
	POP R24
	POP R22
	RET


;;;;;;;;;;;;;;;;;;;;;;LCD SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; INIT
LCD_ON: 
	PUSH TEMP
		LDI TEMP, 0b11110111
		OUT LCD_DDR, TEMP ; SET OUTPUT PORT TO LCD (DATA PA4 - PA7, RS =PA0, RW = PA1, EN = PA2)
		CALL DELAY_20ms ; WAIT FOR POWER UP
		LDI LCD_TEMP, 0x02 ; RETURN HOME
		CALL CMDWRITE
		LDI LCD_TEMP, 0x28 ; FUCNTION SET: 4-BIT, 2 LINES, 5x7 DOTS
		CALL CMDWRITE
		LDI LCD_TEMP, 0x0E ; DISPLAY ON, CURSOR ON
		CALL CMDWRITE
		LDI LCD_TEMP, 0x01 ; CLEAR DISPLAY SCREEN
		CALL CMDWRITE
		LDI LCD_TEMP, 0x80 ; FORCE CURSOR TO BEGIN OF 1ST ROW
		CALL CMDWRITE
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;" DISPLAY KEY PRESSED :"
STRING_OUT: 
		PUSH R30
		PUSH R31
			LDI R31, HIGH(LAB_MSG<<1)
			LDI R30, LOW(LAB_MSG<<1)
		STRING:
			LPM LCD_TEMP, Z+
			CPI LCD_TEMP, 0
			BREQ DONE
			CALL DATAWRITE
			RJMP STRING
	DONE:
		POP R31
		POP R30
		RET

LAB_MSG: .DB "KEY PRESSED: ", 0 ; string to be wrotten

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CMDWRITE: 
	PUSH R18
		CALL DELAY_20ms
		MOV R18,LCD_TEMP
		ANDI R18,0xF0 ; MASK LOW NIBBLE
		OUT LCD_PORT, R18 ; SEND HIGH NIBBLE
		SBI LCD_PORT, LCD_EN ; EN = 1 HIGH PULSE
		CALL SDELAY ; EXTEND EN PULSE
		CBI LCD_PORT, LCD_EN ; EN=0 FOR H-to-L PULSE
		CALL DELAY_100us
		SWAP LCD_TEMP
		ANDI LCD_TEMP, 0xF0 ; MASK HIGH NIBBLE
		OUT LCD_PORT, LCD_TEMP ; SEND LOW NIBBLE
		SBI LCD_PORT, LCD_EN ; EN = 1 FOR HIGH PULSE
		CALL SDELAY ; EXTEND EN PULSE
		CBI LCD_PORT, LCD_EN ; EN=0 FOR H-to-L PULSE
		CALL DELAY_100us
	POP R18
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DATAWRITE:
	PUSH R18
		CALL DELAY_20ms
		MOV R18,LCD_TEMP ; mov to operate on R18 without changing LCD_TEMP for next nibble
		ANDI R18,0xF0 ; MASK LOW NIBBLE
		OUT LCD_PORT, R18 ; SEND HIGH NIBBLE
		SBI LCD_PORT, LCD_RS ; RS = 1 TO DATA
		SBI LCD_PORT, LCD_EN ; EN = 1 HIGH PULSE
		CALL SDELAY ; EXTEND EN PULSE
		CBI LCD_PORT, LCD_EN ; EN=0 FOR H-to-L PULSE
		CALL DELAY_100us
		SWAP LCD_TEMP
		ANDI LCD_TEMP, 0xF0 ; MASK HIGH NIBBLE
		OUT LCD_PORT, LCD_TEMP ; SEND LOW NIBBLE
		SBI LCD_PORT, LCD_RS ; RS = 1 TO DATA
		SBI LCD_PORT, LCD_EN ; EN = 1 HIGH PULSE
		CALL SDELAY ; EXTEND EN PULSE
		CBI LCD_PORT, LCD_EN ; EN=0 FOR H-to-L PULSE
		CALL DELAY_100us
	POP R18
	RET

;;;;;;;;;;;;;;;;;;;;;;DELAY SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SDELAY:
		NOP
		NOP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_100us:
	PUSH TEMP
		LDI TEMP,100
	DR0:
		CALL SDELAY
		DEC TEMP
		BRNE DR0
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_2ms:
	PUSH TEMP
		LDI TEMP,20
	LDR0:
		CALL DELAY_100us
		DEC TEMP
		BRNE LDR0
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_10ms:
	PUSH TEMP
		LDI TEMP, 5
	BOUNCER:
		CALL DELAY_2ms
		DEC TEMP
		BRNE BOUNCER
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_20ms:
	PUSH TEMP
		LDI TEMP, 10
	POWERUP:
		CALL DELAY_2ms
		DEC TEMP
		BRNE POWERUP
	POP TEMP
	RET