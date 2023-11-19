/* 
										Master_main.asm :

Description of hardware connection :
	IRQ PB0
	SPI : PB7:4
	UART : PD1
	TEST : PD3 connect to a LED
	LCD : PA7:0
	BARLED : PC7:0

Summary of subroutines :
	MAIN which include MAIN_LOOP
	INITIALIZATION which include INIT_ALL and all init ports except specific ports
	INTERRUPTS which include INIT_INTERRUPTS and ISR_IQR
	UART which include INIT_UART0, UART0_TRANSMIT and Conv_Dec_to_ASCII
	SPI which include INIT_MSPI and MSPI_RECEIVE
	PROTOCOL to send value from SPI_to_LCD
	LCD which include all LCD command : LCD_ON, STRING_OUT, CMDWRITE and DATAWRITE
	DELAY which include SDELAY, delay_100us, 2ms, 10ms and 20ms used for LCD protocol
*/

.ORG 0
	JMP MAIN
.ORG 0x000A ; Interrupt code for PCINT1
	JMP ISR_IQR

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
;.EQU KEY_PORT = PORTD ; KEY DATA PORT
;.EQU KEY_PIN = PIND ; KEY DATA PIN
;.EQU KEY_DDR = DDRD ; KEY DATA DDR


; SPI configuration
.EQU PIN_SS = 4
.EQU PIN_MOSI = 5
.EQU PIN_MISO = 6
.EQU PIN_SCK = 7
.EQU SPI_DDR = DDRB
.EQU SPI_PORT = PORTB

; IRQ configuration
.EQU IRQ_DDR = DDRB
.EQU IRQ_PORT = PORTB
.EQU IRQ_PIN = PINB
.EQU IRQ_PIN_number = 0

; TEST configuration
.EQU TEST_DDR = DDRD
.EQU TEST_PORT = PORTD
.EQU TEST_PIN = PIND
.EQU TEST_PIN_number = 3

; register definition
.DEF TEMP = R16 ; free register
.DEF DATA_TEMP = R17 ; value receive from SPI and send to UART
.DEF LCD_TEMP = R20 ; value to write on the LCD
 
;*********************************************************************;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;; MAIN ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;*********************************************************************;
MAIN:
        LDI TEMP, LOW(RAMEND) ; set up stack pointer
		OUT SPL, TEMP
		LDI TEMP, HIGH(RAMEND)
		OUT SPH, TEMP

		CALL INIT_ALL
		CALL STRING_OUT ; display "KEY PRESSED :" on LCD
 
	MAIN_LOOP:
	; sleep until key is pressed so until pin change interrupt IQR
	RJMP MAIN_LOOP
 
;;;;;;;;;;;;;;;;;;;;;;;;INITIALIZATION;;;;;;;;;;:;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_ALL :
		CALL INIT_BARLED
		CALL LCD_ON
		CALL INIT_IRQ
		CALL INIT_MSPI ; INITIALIZE MASTER SPI
		CALL INIT_UART0
		CALL INIT_INTERRUPTS
		CALL INIT_TEST
	RET

INIT_BARLED :
	PUSH TEMP
		LDI TEMP, 0xFF
		OUT LED_DDR, TEMP ; SET PORTC AS OUTPUT TO LEDBAR
	POP TEMP
	RET

INIT_IRQ :
		; SET IRQ_PIN as input
		CBI IRQ_DDR, IRQ_PIN_number
		;SBI IRQ_PORT, IRQ_PIN ; pull up ?
	RET

INIT_TEST :
		; SET TEST_PIN as output
		SBI TEST_DDR, TEST_PIN_number
		SBI TEST_PORT, TEST_PIN_number ; TEST rest state is HIGH
	RET


 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;INTERRUPTS;;;;;;;;;;;;;;;;;;;;;;;;;;;
 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 INIT_INTERRUPTS :
	PUSH TEMP
		SEI
		LDI TEMP, (1<<PCIE1) ; Enable pin change interrupt on PB
		STS PCICR, TEMP
		LDI TEMP, (1<<PCINT8) ; Enable pin change interrupt on PB1
		STS PCMSK1, TEMP
	POP TEMP
	RET

 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 ISR_IQR :
		CBI TEST_PORT, TEST_PIN_number ;######################################       TEST       ######################################
		SBIC IRQ_PIN, IRQ_PIN_number
	RETI ; return if pin is HIGH
		CALL MSPI_RECEIVE ; read data and store it in DATA_TEMP
		;OUT LED_PORT, DATA_TEMP ; check the value received
		;LDI DATA_TEMP, 0
		CALL SPI_to_LCD ; display data on LCD
		CALL Conv_DEC_to_ASCII
		CALL UART0_TRANSMIT
		SBI TEST_PORT, TEST_PIN_number ;######################################       TEST       ######################################
	RETI

;;;;;;;;;;;;;;;;;;;;;;;UART SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_UART0:
	PUSH TEMP
		CLR TEMP
		STS UBRR0H, TEMP
		LDI TEMP, 51
		STS UBRR0L, TEMP
		LDI TEMP, (1 << TXEN0) ; ENABLE TRANSMISSION
		STS UCSR0B, TEMP
		LDI TEMP, (1 << UCSZ01) | (1 << UCSZ00) ; ASYNC, 1 STOP-BIT, 1-BYTE DATA
		STS UCSR0C, TEMP
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
UART0_TRANSMIT:
	PUSH TEMP
		LDS TEMP, UCSR0A 
		SBRS TEMP, UDRE0 ; check if UDR0 is empty : transmission available ?
		RJMP UART0_TRANSMIT
		STS UDR0, DATA_TEMP ; send data to the transmit shift register
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Conv_DEC_to_ASCII :
		PUSH TEMP
			CPI DATA_TEMP, 10
			BRLO LOWER_10
		LETTER :
			LDI TEMP, 55
			ADD DATA_TEMP, TEMP
			RJMP RETURN
		LOWER_10 :
			LDI TEMP, 48
			ADD DATA_TEMP, TEMP
			RJMP RETURN
	RETURN :
		POP TEMP
		RET

;;;;;;;;;;;;;;;;;;;;;;;SPI SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_MSPI:
	PUSH TEMP
		; ENABLE SPI MASTER, RATE FCK/16
		LDI TEMP, (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS)
		IN TEMP, SPI_DDR
		ORI TEMP, 0b11110000 ; change only SPI_PIN direction
		OUT SPI_DDR, TEMP
		LDI TEMP, (1 << SPE0) | (1 << MSTR0) | (1 << SPR00) ; CLK = Fosc/16
		OUT SPCR0, TEMP
		CBI SPI_PORT, PIN_SS ; Enable SPI communication
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MSPI_RECEIVE:
	PUSH TEMP
		; MASTER SPI DATA RECEIVING
		CLR DATA_TEMP
		OUT SPDR0, DATA_TEMP ; start transmitting
	WAIT_RECEIVE:
		; WAIT TRANSMISSION COMPLETE
		IN TEMP, SPSR0
		SBRS TEMP, SPIF0
		RJMP WAIT_RECEIVE
		IN DATA_TEMP, SPDR0 ; receiving
		OUT LED_PORT, DATA_TEMP ; check the value received ;######################################       ERROR       ######################################
	POP TEMP
	RET

;;;;;;;;;;;;;;;;PROTOCOL to send value from SPI to LCD;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SPI_to_LCD :
		MOV R23, DATA_TEMP ; store value from SPI in LCD compare register
		LDI LCD_TEMP, 0xC0 ; FORCE CURSON TO BEGIN 2ND ROW
		CALL CMDWRITE
		CPI R23, 0xFF ; check if no key is pressed
		BREQ NOKEY
		CPI R23, 10
		BRLO DIGIT
		LDI LCD_TEMP, 55
		ADD LCD_TEMP, R23 ; add to convert the decimal to LCD
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
		;OUT LED_PORT, R23
	
	RET

;;;;;;;;;;;;;;;;;;;;;;LCD SUBROUTINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

LAB_MSG: .DB "KEY PRESSED: ", 0

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
		;CBI LCD_PORT, LCD_RW ; RW = 0 TO WRITE
		SBI LCD_PORT, LCD_EN ; EN = 1 HIGH PULSE
		CALL SDELAY ; EXTEND EN PULSE
		CBI LCD_PORT, LCD_EN ; EN=0 FOR H-to-L PULSE
		CALL DELAY_100us
		SWAP LCD_TEMP
		ANDI LCD_TEMP, 0xF0 ; MASK HIGH NIBBLE
		OUT LCD_PORT, LCD_TEMP ; SEND LOW NIBBLE
		SBI LCD_PORT, LCD_RS ; RS = 1 TO DATA
		;CBI LCD_PORT, LCD_RW ; RW = 0 TO write
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
	PUSH R17
		LDI R17,100
	DR0:
		CALL SDELAY
		DEC R17
		BRNE DR0
	POP R17
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_2ms:
	PUSH R17
		LDI R17,20
	LDR0:
		CALL DELAY_100us
		DEC R17
		BRNE LDR0
	POP R17
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_10ms:
	PUSH R17
		LDI R17, 5
	BOUNCER:
		CALL DELAY_2ms
		DEC R17
		BRNE BOUNCER
	POP R17
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_20ms:
	PUSH R17
		LDI R17, 10
	POWERUP:
		CALL DELAY_2ms
		DEC R17
		BRNE POWERUP
	POP R17
	RET