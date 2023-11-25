
.ORG 0
	JMP MAIN
.ORG 0x000A 
	JMP ISR_IQR

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

; SPI initialize
.EQU PIN_SS = 4
.EQU PIN_MOSI = 5
.EQU PIN_MISO = 6
.EQU PIN_SCK = 7
.EQU SPI_DDR = DDRB
.EQU SPI_PORT = PORTB

; IRQ initialize
.EQU IRQ_DDR = DDRB
.EQU IRQ_PORT = PORTB
.EQU IRQ_PIN = PINB
.EQU IRQ_PIN_number = 0

; TEST initialize
.EQU TEST_DDR = DDRD
.EQU TEST_PORT = PORTD
.EQU TEST_PIN = PIND
.EQU TEST_PIN_number = 3

; register req
.DEF TEMP = R16 
.DEF DATA_TEMP = R17 
.DEF LCD_TEMP = R20 
 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; MAIN ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

MAIN:
        LDI TEMP, LOW(RAMEND) 
		OUT SPL, TEMP
		LDI TEMP, HIGH(RAMEND)
		OUT SPH, TEMP

		CALL INIT_ALL
		CALL STRING_OUT ; prompt "KEY  :" on LCD
 
	MAIN_LOOP:
	RJMP MAIN_LOOP
 
;;;;;;;;;;;;;;;;;;;;;;;;INITIALIZATION;;;;;;;;;;:;;;;;;;;;;;;;;;;;;;;;;

INIT_ALL :
		CALL INIT_BARLED
		CALL LCD_ON
		CALL INIT_IRQ
		CALL INIT_MSPI 
		CALL INIT_UART0
		CALL INIT_INTERRUPTS
		CALL INIT_TEST
	RET

INIT_BARLED :
	PUSH TEMP
		LDI TEMP, 0xFF
		OUT LED_DDR, TEMP
	POP TEMP
	RET

INIT_IRQ :
		CBI IRQ_DDR, IRQ_PIN_number
	RET

INIT_TEST :
		SBI TEST_DDR, TEST_PIN_number
		SBI TEST_PORT, TEST_PIN_number
	RET


 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;INTERRUPTS;;;;;;;;;;;;;;;;;;;;;;;;;;;
 INIT_INTERRUPTS :
	PUSH TEMP
		SEI
		LDI TEMP, (1<<PCIE1)
		STS PCICR, TEMP
		LDI TEMP, (1<<PCINT8) 
		STS PCMSK1, TEMP
	POP TEMP
	RET

 ;;;;;;;;;;;;;;;;;;;;;;;;;;TEST PROMPT;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 ISR_IQR :
		CBI TEST_PORT, TEST_PIN_number ;######################################       TEST       ######################################
		SBIC IRQ_PIN, IRQ_PIN_number
	RETI 
		CALL MSPI_RECEIVE 
		
		
		CALL SPI_to_LCD 
		CALL Conv_DEC_to_ASCII
		CALL UART0_TRANSMIT
		SBI TEST_PORT, TEST_PIN_number ;######################################       TEST       ######################################
	RETI

;;;;;;;;;;;;;;;;;;;;;;;UART ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

INIT_UART0:
	PUSH TEMP
		CLR TEMP
		STS UBRR0H, TEMP
		LDI TEMP, 51
		STS UBRR0L, TEMP
		LDI TEMP, (1 << TXEN0)
		STS UCSR0B, TEMP
		LDI TEMP, (1 << UCSZ01) | (1 << UCSZ00) 
		STS UCSR0C, TEMP
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
UART0_TRANSMIT:
	PUSH TEMP
		LDS TEMP, UCSR0A 
		SBRS TEMP, UDRE0 
		RJMP UART0_TRANSMIT
		STS UDR0, DATA_TEMP 
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

;;;;;;;;;;;;;;;;;;;;;;;SPI ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

INIT_MSPI:
	PUSH TEMP
		LDI TEMP, (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS)
		IN TEMP, SPI_DDR
		ORI TEMP, 0b11110000
		OUT SPI_DDR, TEMP
		LDI TEMP, (1 << SPE0) | (1 << MSTR0) | (1 << SPR00) 
		OUT SPCR0, TEMP
		CBI SPI_PORT, PIN_SS 
	POP TEMP
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MSPI_RECEIVE:
	PUSH TEMP
		CLR DATA_TEMP
		OUT SPDR0, DATA_TEMP
	WAIT_RECEIVE:
		IN TEMP, SPSR0
		SBRS TEMP, SPIF0
		RJMP WAIT_RECEIVE
		IN DATA_TEMP, SPDR0 
		OUT LED_PORT, DATA_TEMP 
	POP TEMP
	RET

;;;;;;;;;;;;;;;;PROTOCOL to send value from SPI to LCD;;;;;;;;;;;;;;;;;

SPI_to_LCD :
		MOV R23, DATA_TEMP
		LDI LCD_TEMP, 0xC0
		CALL CMDWRITE
		CPI R23, 0xFF
		BREQ NOKEY
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
	
	RET

;;;;;;;;;;;;;;;;;;;;;;LCD ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

;;;;;;;;;;;;;;;;;;;;;;DELAY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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