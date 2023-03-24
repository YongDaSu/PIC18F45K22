;************************************************
;* 				p18lcd.asm						*
;* Routines include:                            *
;*   - InitLCD to initialize the LCD panel      *
;*   - putcLCD to write a character to LCD      *
;*   - SendCmd to write a command to LCD        *
;*   - clrLCD to clear the LCD display          *
;*   - L1homeLCD to return cursor to line 1 home*
;*   - L2homeLCD to return cursor to line 2 home*
;*   - PutHexLCD to write a HEX Code to LCD     *
;*   - Hex2ASCII to convert 4 bits to ASCII Code*
;************************************************

list p=18f45k22		
#include 	<p18f45k22.inc>			; 納入定義檔Include file located at defult directory	
;
			global		InitLCD
			global		putcLCD
			global		clrLCD
			global		L1homeLCD
			global		L2homeLCD
			global  	Send_Cmd
			global		PutHexLCD
			global		Hex2ASCII
			global		Delay_xMS
			global		Delay_1mS	
;
; 定義LCD資料匯流排使用腳位與方向控制位元
; PORTD[0:3]-->DB[4:7]
;
		
LCD_ANSEL	equ			ANSELD
LCD_CTRL	equ			TRISD
LCD_DATA	equ			LATD
;定義LCD模組控制使用腳位與方向控制位元
; PORTE,2 --> [E] : LCD operation start signal control 
; PORTE,1 --> [RW]: LCD Read/Write control
; PORTE,0 --> [RS]: LCD Register Select control
;				  : "0" for Instrunction register (Write), Busy Flag (Read)
;				  : "1" for data register (Read/Write)
#define		LCD_E_ANSEL	ANSELE,2	
#define		LCD_RW_ANSEL	ANSELE,1	
#define		LCD_RS_ANSEL	ANSELE,0
#define		LCD_E_DIR	TRISE,2	
#define		LCD_RW_DIR	TRISE,1	
#define		LCD_RS_DIR	TRISE,0
#define		LCD_E		LATE,2
#define		LCD_RW 		LATE,1
#define		LCD_RS		LATE,0	 


; LCD模組相關控制命令
CLR_DISP	equ		b'00000001'		; Clear display and return cursor to home
Cursor_Home	equ		b'00000010'		; Return cursor to home position

ENTRY_DEC	equ		b'00000100'		; Decrement cursor position & No display shift
ENTRY_DEC_S	equ		b'00000101'		; Decrement cursor position & display shift
ENTRY_INC	equ		b'00000110'		; Increment cursor position & No display shift
ENTRY_INC_S	equ		b'00000111'		; Increment cursor position & display shift

DISP_OFF	equ		b'00001000'		; Display off
DISP_ON		equ		b'00001100'		; Display on control
DISP_ON_C	equ		b'00001110'		; Display on, Cursor on
DISP_ON_B	equ 	b'00001111'		; Display on, Cursor on, Blink cursor

FUNC_SET	equ		b'00101000'		; 4-bit interface , 2-lines & 5x7 dots
CG_RAM_ADDR	equ		b'01000000'		; Least Significant 6-bit are for CGRAM address
DD_RAM_ADDR	equ		b'10000000'		; Least Significant 7-bit are for DDRAM address
;

; 宣告資料暫存變數
			UDATA
LCD_Byte	RES		1
LCD_Temp	RES		1
Count_100uS	RES		1
Count_1mS	RES		1
Count_mS	RES		1
W_BUFR		RES		1
Hex_Bfr	 	RES		1

;可攜式程式區塊開始宣告
LCD_CODE 	CODE	
;*******************************************************************
; LCD模組初始化函式
;*******************************************************************
InitLCD
			bcf		LCD_E			; Clear LCD control line to Zero
			bcf		LCD_RW 
			bcf		LCD_RS 
;
			banksel		LCD_ANSEL		; Clear LCD control line Analog Function
			movf		LCD_ANSEL, W, BANKED		; get I/O directional settng
			andlw		0x0F
			movwf		LCD_ANSEL, BANKED		; set LCD bus  DB[4:7] for output
			bcf		LCD_E_ANSEL, BANKED
			bcf		LCD_RW_ANSEL, BANKED
			bcf		LCD_RS_ANSEL, BANKED
			
;
			bcf		LCD_E_DIR		;configure control lines for Output pin
			bcf		LCD_RW_DIR
			bcf		LCD_RS_DIR
;
			movf	LCD_CTRL,W		; get I/O directional settng
			andlw	0x0F
			movwf	LCD_CTRL		; set LCD bus  DB[4:7] for output
;
			movlw	.50				; Power-On delay 50mS
			rcall 	Delay_xMS
;
			movlw   b'00000011'		; #1 , Init for 4-bit interface
			rcall	Send_Low_4bit
;
			movlw	.10				;  Delay 10 mS
			rcall 	Delay_xMS
;
			movlw	b'00000011'		; #2 , Fully Initial LCD module 
			rcall	Send_Low_4bit	; Sent '0011' data  
			rcall 	Delay_1mS
;
			movlw	b'00000011'		; #3 , Fully Initial LCD module 
			rcall	Send_Low_4bit	; Sent '0011' data  
			rcall 	Delay_1mS
;
			movlw	b'00000010'		; #4 , Fully Initial LCD module 
			rcall	Send_Low_4bit	; Sent '0010' data  
			rcall 	Delay_1mS
;
			movlw	FUNC_SET		; #5,#6 , Set 4-bit mode , 2 lines & 5 x 7 dots
			rcall	Send_Cmd
			rcall 	Delay_1mS
;
			movlw	DISP_ON 		; #7,#8 , Turn display on (0x0C)
			rcall	Send_Cmd
			rcall 	Delay_1mS
;
			movlw	CLR_DISP		; #9,#10 , Clear LCD Screen
			rcall	Send_Cmd
			movlw	.5				; Delay 5mS for Clear LCD Command execution
			rcall 	Delay_xMS
;
			movlw	ENTRY_INC		; #11,#12 , Configure cursor movement
			rcall	Send_Cmd
			rcall 	Delay_1mS
;
			movlw	DD_RAM_ADDR		; Set writes for display memory
			rcall	Send_Cmd
			rcall 	Delay_1mS
;
			return
;
;*******************************************************************
;*putcLCD - 將字元符號送到LCD函式*
;*******************************************************************
putcLCD
			banksel LCD_Byte
			movwf	LCD_Byte, BANKED		; Save WREG in Byte variable
			rcall	Send_High_LCD	; Send upper nibble first	
			banksel LCD_Byte
			movf	LCD_Byte,W
			rcall	Send_Low_LCD	; Send lower nibble data
			rcall	Delay_100uS
			return
;			
Send_Low_LCD 
			swapf	WREG,W				; swap high/low nibble
Send_High_LCD 
			bcf		LCD_RW			; set LCD Write Mode
			andlw	0xF0			; Clear high nibble	
			banksel LCD_Temp
			movwf	LCD_Temp	
			movf	LCD_DATA,W		; Read back PORT
			andlw	0x0F			; keep data for PORTD[4:7]
			iorwf	LCD_Temp,W
			movwf 	LCD_DATA		; Write data to LCD bus	for low nibble bus DB[4:7]
			bsf		LCD_RS			; Set for data input
			nop
			bsf		LCD_E			; Clock nibble into LCD
			nop
			bcf		LCD_E
			return
;
;*********************************************************************
;*      將16進位的數值轉換成文字符號傳到LCD
;*********************************************************************
PutHexLCD
			banksel W_BUFR
			movwf	W_BUFR			; Save W Register !!
			swapf	W_BUFR,W		; High nibble first !!	
			rcall	Hex2ASCII
			rcall	putcLCD
;
			movf	W_BUFR,W
			rcall	Hex2ASCII
			rcall	putcLCD
			return
;
;******************************************************************
;*      轉換四位元資料為ASCII編碼資料
;******************************************************************
Hex2ASCII
			andlw	0x0f			; Mask Bit 4 to 7
			banksel Hex_Bfr
			movwf	Hex_Bfr
			sublw	.09
			btfsc	STATUS,C		; If W less than A (C=1) --> only add 30h
			bra	_Add_W_30  
_Add_W_37		movlw	0x37
			bra	_Hex_cont 
_Add_W_30		movlw	0x30
_Hex_cont		
			banksel W_BUFR
			addwf	Hex_Bfr,W		; The correct ASCII code for this char !!
	 		return
;
;*******************************************************************
;* SendCmd - 將LCD控制指令傳到LCD*
;*******************************************************************
Send_Cmd
			banksel LCD_Byte
			movwf	LCD_Byte		; Save WREG in Byte variable
			rcall	Send_High_4bit	; Send upper nibble first	
			banksel LCD_Byte
			movf	LCD_Byte,W
			rcall	Send_Low_4bit	; Send lower nibble data
			rcall	Delay_100uS
			return
;			
Send_Low_4bit
			swapf	WREG,W				; swap high/low nibble
Send_High_4bit
			bcf		LCD_RW			; set LCD Write Mode
			andlw	0xF0			; Clear high nibble	
			banksel LCD_Temp
			movwf	LCD_Temp	
			movf	LCD_DATA,W		; Read back PORT
			andlw	0x0F			; keep data for PORTD[4:7]
			iorwf	LCD_Temp,W
			movwf 	LCD_DATA		; Write data to LCD bus	for low nibble bus DB[4:7]
			bcf		LCD_RS			; Clear for command inut
			nop
			bsf		LCD_E			; Clock nibble into LCD
			nop
			bcf		LCD_E
			return
;
;*******************************************************************
;* clrLCD - 清除LCD模組顯示內容
;*******************************************************************
clrLCD
			movlw	CLR_DISP		; Clear LCD screen
			rcall	Send_Cmd
			movlw	.5				; Delay 5mS for Clear LCD Command execution
			bra 	Delay_xMS
;
;*******************************************************************
;* L1homeLCD - 將輸入游標移至第一行起始位址
;*******************************************************************
L1homeLCD
			movlw	DD_RAM_ADDR|0x00 ; Send command to move cursor to 
			rcall	Send_Cmd		     ; home position on line 1
			bra		Delay_100uS
;
;*******************************************************************
;* L2homeLCD -將輸入游標移至第二行起始位址
;*******************************************************************
L2homeLCD
			movlw	DD_RAM_ADDR|0x40 ; Send command to move cursor to
			rcall	Send_Cmd		 	 ; home position on line 2
			bra		Delay_100uS
;
;*******************************************************************
;*       Delay – 延遲XmS
;*******************************************************************
Delay_xMS
			banksel Count_mS
			movwf	Count_mS
;
_D_mS		call	Delay_1mS
			banksel Count_mS
			decfsz	Count_mS,F
			goto	_D_mS
			return
;
;*******************************************************************
;* Delay_1mS -延遲1mS (1.00mS @ 10MHz)                  *
;* Delay_100uS -延遲100us @ 10Mhz					   *                                             
;*******************************************************************
Delay_1mS
			movlw	.10			; Load 1mS Dealy Value
			banksel Count_1mS
			movwf	Count_1mS		; Save to Count2
_D_1mS		call	Delay_100uS
			banksel Count_1mS
			decfsz	Count_1mS,F		; Count_1ms = 0 ?
			bra		_D_1mS		
			return					
;
Delay_100uS
			movlw	.50			; Load delay 100uS value
			banksel Count_100uS
			movwf	Count_100uS		; Save to Count2
_D_2uS		nop
			nop
			decfsz	Count_100uS,F		; Count2 - 1 = 0 ?
			bra		_D_2uS		
			return					
;
			END

