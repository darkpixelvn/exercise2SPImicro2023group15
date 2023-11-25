.ORG 0
	JMP MAIN

; LCD initialize
.EQU LCD_PORT = PORTA 
.EQU LCD_DDR = DDRA
.EQU LCD_PIN = PINA 
.EQU LCD_RS = 0 
.EQU LCD_RW = 1  
.EQU LCD_EN = 2 

; BARLED initialize
.EQU LED_PORT = PORTC
.EQU LED_PIN = PINC
.EQU LED_DDR = DDRC

; KEYPAD initialize
.EQU KEY_PORT = PORTD
.EQU KEY_PIN = PIND
.EQU KEY_DDR = DDRD

; SPI initialize
.EQU PIN_SS = 4
.EQU PIN_MOSI = 5
.EQU PIN_MISO = 6
.EQU PIN_SCK = 7
.EQU SPI_DDR = DDRB
.EQU SPI_PORT = PORTB
.EQU SPI_PIN = PINB

; IRQ initialize
.EQU IRQ_DDR = DDRB
.EQU IRQ_PORT = PORTB
.EQU IRQ_PIN = PINB
.EQU IRQ_PIN_number = 0

; register REQ
.DEF TEMP = R16
.DEF DATA_TEMP = R17 
.DEF LCD_TEMP = R20 
 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;MAIN;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MAIN:
        LDI R21, LOW(RAMEND) ; set up stack pointer
		OUT SPL, R21
		LDI R21, HIGH(RAMEND)
		OUT SPH, R21

		CALL INIT_ALL
		CALL STRING_OUT ; prompt "KEY :" on LCD
 
	MAIN_LOOP:
        CALL SCANKEY ; scan keypad and send data if something is pressed
		CALL KEYPAD_to_LCD ; display on LCD (and BARLED)
	RJMP MAIN_LOOP
 
;;;;;;;;;;;;;;;;;;;;;;;;INITIALIZATION;;;;;;;;;;:;;;;;;;;;;;;;;;;;;;;;;
INIT_ALL :
		CALL INIT_BARLED
		CALL INIT_KEYPAD
		CALL LCD_ON
		CALL INIT_IRQ
		CALL INIT_SSPI 
	RET

INIT_BARLED :
	PUSH TEMP
		LDI TEMP, 0xFF
		OUT LED_DDR, TEMP
	POP TEMP
	RET

INIT_IRQ :
		SBI IRQ_DDR, IRQ_PIN_number
		SBI IRQ_PORT, IRQ_PIN_number
	RET
 
;;;;;;;;;;;;;;;;;;;;;;;SPI;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INIT_SSPI:
	PUSH TEMP
		LDI TEMP, (1 << PIN_MISO) 
		IN TEMP, SPI_DDR
		ORI TEMP, 0b11110000 
		OUT SPI_DDR, TEMP
		LDI TEMP, (1 << SPE0) 
		OUT SPCR0, TEMP
	POP TEMP
	RET


SPI_PROTOCOL:
		MOV DATA_TEMP, R23 
		CBI IRQ_PORT, IRQ_PIN_number 
		CALL SSPI_TRANSMIT
		SBI IRQ_PORT, IRQ_PIN_number 
	RET

SSPI_TRANSMIT:
	PUSH TEMP
		OUT SPDR0, DATA_TEMP
	WAIT_TRANSMIT:
		IN TEMP, SPSR0
		SBRS TEMP, SPIF0
		RJMP WAIT_TRANSMIT
	POP TEMP
	RET


KEYPAD_to_LCD :
		LDI LCD_TEMP, 0xC0 
		CALL CMDWRITE
		CPI R23, 0xFF 
		BREQ NOKEY
		CALL SPI_PROTOCOL 
		CPI R23, 10
		BRLO DIGIT
		LDI LCD_TEMP, 55
		ADD LCD_TEMP, R23 
		RJMP DISPLAY
	DIGIT:
		LDI LCD_TEMP, 48
		ADD LCD_TEMP, R23 
	DISPLAY:
		CALL DATAWRITE
	NOKEY:
		LDI LCD_TEMP, 32
		CALL DATAWRITE
		LDI LCD_TEMP, 32
		CALL DATAWRITE
		OUT LED_PORT, R23
	RET

;;;;;;;;;;;;;;;;;;KEY PAD ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

INIT_KEYPAD :
	PUSH TEMP
		LDI TEMP, 0x0F
		OUT KEY_DDR, TEMP 
		LDI TEMP, 0xF0
		OUT KEY_PORT, TEMP 
	POP TEMP
	RET


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
		SEC
		ROR R22
		DEC R24
		RJMP KEYPAD_SCAN_LOOP

	KEYPAD_SCAN_FOUND:
		LSL R23
		LSL R23
		ADD R23, R24 
	POP R24
	POP R22
	RET
	KEYPAD_SCAN_NOT_FOUND: 
		LDI R23, 0xFF
	POP R24
	POP R22
	RET


;;;;;;;;;;;;;;;;;;;;;; LCD ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


LCD_ON: 
	PUSH TEMP
		LDI TEMP, 0b11110111
		OUT LCD_DDR, TEMP 
		CALL DELAY_20ms 
		LDI LCD_TEMP, 0x02 
		CALL CMDWRITE
		LDI LCD_TEMP, 0x28 
		CALL CMDWRITE
		LDI LCD_TEMP, 0x0E 
		CALL CMDWRITE
		LDI LCD_TEMP, 0x01 
		CALL CMDWRITE
		LDI LCD_TEMP, 0x80 
		CALL CMDWRITE
	POP TEMP
	RET


;;;;;;;;;;;;;;;;;;;" DISPLAY KEY PRESSED :";;;;;;;;;;;;;;;;;
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

LAB_MSG: .DB "KEY: ", 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CMDWRITE: 
	PUSH R18
		CALL DELAY_20ms
		MOV R18,LCD_TEMP
		ANDI R18,0xF0 
		OUT LCD_PORT, R18 
		SBI LCD_PORT, LCD_EN 
		CALL SDELAY 
		CBI LCD_PORT, LCD_EN 
		CALL DELAY_100us
		SWAP LCD_TEMP
		ANDI LCD_TEMP, 0xF0 
		OUT LCD_PORT, LCD_TEMP 
		SBI LCD_PORT, LCD_EN 
		CALL SDELAY 
		CBI LCD_PORT, LCD_EN 
		CALL DELAY_100us
	POP R18
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DATAWRITE:
	PUSH R18
		CALL DELAY_20ms
		MOV R18,LCD_TEMP 
		ANDI R18,0xF0 
		OUT LCD_PORT, R18 
		SBI LCD_PORT, LCD_RS 
		SBI LCD_PORT, LCD_EN 
		CALL SDELAY 
		CBI LCD_PORT, LCD_EN 
		CALL DELAY_100us
		SWAP LCD_TEMP
		ANDI LCD_TEMP, 0xF0 
		OUT LCD_PORT, LCD_TEMP 
		SBI LCD_PORT, LCD_RS 
		SBI LCD_PORT, LCD_EN
		CALL SDELAY 
		CBI LCD_PORT, LCD_EN 
		CALL DELAY_100us
	POP R18
	RET
;;;;;;;;;;;;;;;;;;;;;;DELAY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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