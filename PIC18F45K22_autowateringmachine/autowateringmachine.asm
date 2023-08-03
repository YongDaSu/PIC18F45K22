;****************************************************************

;****************************************************************

list p=18f45k22		
#include 	<p18f45k22.inc>					; 納入定義檔Include file located at defult directory	
EXTERN	InitLCD, putcLCD, Send_Cmd, L1homeLCD, L2homeLCD, clrLCD, LLhomeLCD
;
; program start
RX_Temp			EQU	0x20
VAL_US			equ	.147					; 1ms 延遲數值。
VAL_MS			equ	.100
VAL_S			equ	.10
			CBLOCK		0x00			; 由暫存器位址0x00開始宣告保留變數的位址
			C_Hold_Delay				; 類比訊號採樣保持時間延遲計數暫存器
			TxD_Flag				; 資料傳輸延遲時間旗標
			Hex_Temp
			COUNT
			count					; 編碼轉換暫存器
			count_ms
			count_s
			
			ENDC					; 結束變數宣告

			CBLOCK		0x020			; 由暫存器位址0x20開始
			WREG_TEMP				; 儲存處理器資料重要變數
			STATUS_TEMP				; 以便在進出函式時，暫存重要資料
			BSR_TEMP	
			AAA
			BBB
			CCC
			DDD
			SEC
			MIN
			ENDC


;
			CBLOCK		0x30
			Data_EE_Addr				; EEPROM資料位址
			Data_EE_Data				; EEPROM資料
			ADRH					; 高位元資料變數
			ADRL					; 低位元資料變數
			ENDC	
;
#define			EEP_ADRL	0			; Define EEPROM Low Byte Address
#define			EEP_ADRH	1			; Define EEPROM High Byte Address
#define			TMR1_VAL	.32768			; 定義Timer1計時器週期1 SEC
;
					       		
;********************************************************************
;****	RESET Vector @ 0x0000
;********************************************************************

			org		0x00 			; 重置向量
 			bra		Init
;
			org		0x08			; 高優先中斷向量
			bra		Hi_ISRs
;
			org		0x18			; 低優先中斷向量
			movlw		a's'
			cpfseq		RCREG
			bra		Low_ISRs
			bra		Strart_to_count

;********************************************************************
;****	The Main Program start from Here !! 
;********************************************************************

			org		0x02A			; 主程式開始位址
Init:
			lfsr		1,Hex_Temp+0x110	; 將編碼暫存器區塊位址載入FSR

			call		Init_IO
			call		Init_Timer1
			call		Init_AD
			call		Init_USART
			call		InitLCD
			call		Init_CCP2
			call		Init_Timer2

;
			bsf		RCON,IPEN		; 啟動中斷優先順序
			bsf		INTCON,GIEH		; 啟動高優先中斷功能，以利用TIMER1計時器中斷
			bsf		INTCON,GIEL		; 啟動並優先中斷功能，以利用USART RD中斷
			clrf		TxD_Flag
			clrf		COUNT; 清除計時旗標
			clrf		AAA
			clrf		BBB
			clrf		CCC
			clrf		DDD
			clrf		MIN
			clrf		SEC
			clrf		PRODL
;
Main:	
			call		AD_Convert
			movf		ADRESH,W
			movwf		CCPR2L
			
			bra		Main			; 旗標等於1，停止傳送類比訊號資料
;
			

;
;******   Send a byte to USART   ******
Tx_a_Byte:
			movwf		TXREG			; 透過USART送出去的位元符號
			nop					 
			btfss		PIR1,TXIF		; 檢查資料傳輸完成與否?
			bra		$-4			; No, 繼續檢查旗標位元TXIF
			bcf		PIR1,TXIF		; Yes, 清楚旗標位元TXIF
			return
;
;******* Convert low nibble to ASCII code   *******
Hex_ASCII:
 			andlw		h'0F'			; 確定high nibble為"0000"
 			movwf		Hex_Temp
 			movlw		h'9'			; 跟9比較
 			cpfsgt  	Hex_Temp
 			bra		Less_9
 			movf		Hex_Temp,W		; > 9, 數字加0x37
 			addlw		h'37'
 			return
Less_9			movf		Hex_Temp,W		; < = 9, 數字加0x30
 			addlw		h'30'
 			return
 			
;
;*****    Connvert the 10-bit A/D   *****
AD_Convert:
			call		C_Hold_Time		; 延遲50uS 完成訊號採樣保持
			bsf		ADCON0,GO		; 開始A/D轉換
			nop					; Nop
			btfsc		ADCON0,GO		; 檢查A/D轉換是否完成
			bra		$-4			; 否，繼續檢查
			return 
;
;***********************************************************************
;****		Initial I/O Port 
;***********************************************************************
Init_IO:							; 設定數位輸入腳位
			banksel ANSELD
			clrf		ANSELD, BANKED		; 將PORTD類比功能解除，設定數位輸入腳位
			bcf		ANSELC, 6, BANKED   	; 將TX1/RX1(RC6/RC7)腳位類比功能解除
			bcf		ANSELC, 7, BANKED
			bcf		ANSELB,	3
			bcf		ANSELC,	1
			
			clrf		TRISD			; 將PORTD數位輸出入埠設為輸出
			clrf		LATD
   			bsf		TRISA,	RA0	
			bcf		TRISB,	3
			bcf		TRISC,	1		; 將腳位RA0設為輸入
			return
;
;***********************************************************************
;****		Initial Timer1 as a 1 Sec Timer 
;***********************************************************************
Init_Timer1:
			movlw		B'10001010'		;16位元模式、1倍前除器、非同步計數模式
			movwf		T1CON			; 使用外部外部震盪器，開啟計時器
;
			movlw		(.65536-TMR1_VAL)/.256	; 計算計時器Timer1高位元組資料
			movwf		TMR1H
			movlw		(.65536-TMR1_VAL)%.256	; 計算計時器Timer1低位元組資料
			movwf		TMR1L
;
			bsf		IPR1,TMR1IP		; 設定Timer1高優先中斷
			bcf		PIR1,TMR1IF		; 清除中斷旗標
			bsf		PIE1,TMR1IE		; 開啟計時器中斷功能
;
			return
;
;***********************************************************************
;****		Initial A/D converter 
;***********************************************************************
Init_AD:
  			movlw		b'00000001'		; 選擇AN0通道轉換，
  			movwf		ADCON0			; 啟動A/D模組
;
			movlw		b'0000000'		; 設定VDD/VSS為參考電壓
			movwf		ADCON1			
;
			movlw		b'00111010'		; 結果向右靠齊並
			movwf		ADCON2			; 設定採樣時間20TAD，轉換時間為Fosc/32
;
			bcf		PIE1,ADIE		; 停止A/D中斷功能

			return
;
;***********************************************************************
;****		Initial USART as 9600,N,8,1 
;***********************************************************************
Init_USART:
			movlw		b'00100110'		; 8位元模式非同步傳輸 
			movwf		TXSTA1			; 低鮑率設定，啟動傳輸功能
;
			movlw		b'10010000'		; 啟動8位元資料接收功能
			movwf		RCSTA1			; 連續接收模式，停止位址偵測點
;
			movlw		0x08			; 設定16位元鮑率參數
			movwf		BAUDCON1
			movlw		0x03			; 設定鮑率為9600
			movwf		SPBRG1
			movlw		0x01						
			movwf		SPBRGH1
;
			bcf		PIR1,TXIF		; 清除資料傳輸中斷旗標
			bcf		PIE1,TXIE		; 停止資料傳輸中斷功能
;
			bcf		IPR1,RCIP	  	; 設定資料接收低優先中斷
			bcf		PIR1,RCIF		; 清除資料接收中斷旗標
			bsf		PIE1,RCIE		; 啟動資料接收中斷
;
			return
Init_CCP2:
			movlw		b'00001100'
			movwf		CCP2CON
			clrf		CCPTMRS0
			return
Init_Timer2:
			movlw		b'00000000'
			movwf		T2CON
			movlw		0xFF
			movwf		PR2
			
			bcf		PIR1,TMR2IF
			bcf		PIE1,TMR2IE
			bcf		IPR1,TMR2IP
			return
;
;***********************************************************************
;****		Sample Hold (Charge) time delay routine (50uS) 
;***********************************************************************
C_Hold_Time:
   			movlw		.200
   			movwf		C_Hold_Delay
   			nop
			nop
   			decfsz		C_Hold_Delay,F
   			bra		$-4
   			return

;***************************************************************************
;****		Hi_ISRs : Hi-Priority Interrupt reotine 
;****
;***************************************************************************
Hi_ISRs

			bcf		PIR1, TMR1IF		; 清除Timer1中斷旗標
			bcf		INTCON, GIEH
			bcf		T2CON, 2
			movlw		(.65536-TMR1_VAL)/.256	; 計算計時器Timer1高位元組資料
			movwf		TMR1H
			movlw		(.65536-TMR1_VAL)%.256	; 計算計時器Timer1低位元組資料
			movwf		TMR1L
			;bsf		TxD_Flag, 0		; 設定1 Sec計時旗標
					
			call		change_LCD
			bsf		INTCON, GIEH
			clrf		WREG
			decf		SEC
			btfsc		STATUS,C
			retfie		FAST
			decf		MIN
			btfsc		STATUS,C
			bra 		$+4
			bra 		Label
			movlw		.59
			movwf		SEC
			
			retfie		FAST		 	; 利用shadow register返回
Label:
			call		WATERING
			bsf		T2CON,2
			bsf		PIE1,RCIE
			bcf		T1CON,0
			retfie		FAST
;***************************************************************************************
;****		Low_ISRs : Low-Priority Interrupt reotine 
;****
;***************************************************************************************
Low_ISRs:
			movff		STATUS,STATUS_TEMP		; 儲存處理器資料重要變數
			movff		WREG,WREG_TEMP
			movff		BSR,BSR_TEMP	
			
			bcf		INTCON,GIEH
;
			;call		change_LCD
			bcf		PIR1,RCIF
			incf		COUNT, F,ACCESS
			movff		RCREG, RX_Temp	 		;
			call		TEST_NUM1
			movlw		a'c'
			cpfseq  	RX_Temp
			bra		$+4
			bra		Exit_Low_ISR
			 
			movf		RX_Temp,W
			call		putcLCD
			
			
			movlw		0x04
			cpfseq		COUNT
			bra 		Exit_Low_ISR
			clrf		COUNT
			call 		PRESS_TO_START
			
			
Exit_Low_ISR
			movff		BSR_TEMP,BSR			; 回復重要暫存器資料
			movff		WREG_TEMP,WREG			
			movff		STATUS_TEMP,STATUS	
			bsf		INTCON,GIEH
			retfie		
; 
Strart_to_count:
			movff		STATUS,STATUS_TEMP		; 儲存處理器資料重要變數
			movff		WREG,WREG_TEMP
			movff		BSR,BSR_TEMP
			
			;bsf		PIE1,TMR1IE
			
			bcf		PIE1,RCIE
			
			movlw		.10
			mulwf		CCC
			movf		PRODL,W
			addwf		DDD,W
			movwf		SEC
			
			movlw		.10
			mulwf		AAA
			movf		PRODL,W
			addwf		BBB,W
			movwf		MIN
			
			bsf		T1CON,0
			call		Indicate_START
			
			movff		BSR_TEMP,BSR			; 回復重要暫存器資料
			movff		WREG_TEMP,WREG			
			movff		STATUS_TEMP,STATUS
			
			retfie		FAST
			
			
;------ INTERNAL EEPROM READ ------
;
READ_EEPROM							;讀取eeprom標準程序
			movff		Data_EE_Addr,EEADR
;
			bcf     	INTCON,GIE  
			bcf		EECON1,EEPGD
			bcf		EECON1,CFGS
			bsf		EECON1,RD
			movf		EEDATA,W
			bsf     	INTCON,GIE  
			return
;	
;----INTERNAL EEPROM WRITE-----
;
WRITE_EEPROM							;寫入eeprom標準程序
	        	movff  		Data_EE_Addr,EEADR
	         	movff  		Data_EE_Data,EEDATA
;
	         	BCF	   	EECON1,EEPGD 
	         	BCF  	 	EECON1,CFGS
	      
	         	BSF   	 	EECON1,WREN 
	         	BCF     	INTCON,GIE   
;  
	         	MOVLW  		0X55
	         	MOVWF   	EECON2
	         	MOVLW   	0XAA
	         	MOVWF   	EECON2      
	         	BSF  	 	EECON1,WR 
;         
	         	BSF   		INTCON,GIE 
	
LOOP1    		BTFSS   	PIR2, EEIF 		; 檢查寫入動作是否完成
 	        	GOTO	    	LOOP1 
;
	         	BCF  	 	EECON1,WREN   
	         	BCF   	 	PIR2,EEIF
	      
	         	RETURN
TEST_NUM1:
			movlw		0x01
			cpfseq		COUNT
			bra 		TEST_NUM2
			call		TEST_0_5
			movlw		0x30
			subwf		RX_Temp,W
			movwf		AAA
			call		InitLCD
			movlw		a'c'
			cpfseq 		RX_Temp
			return
			bra 		CLEAR_DATA
		
		
TEST_NUM2:
			movlw		0x02
			cpfseq		COUNT
			bra 		TEST_NUM3
			call 		TEST_0_9
			movlw		0x30
			subwf		RX_Temp,W
			movwf		BBB
			movlw		a'c'
			cpfseq 		RX_Temp
			return
			bra 		CLEAR_DATA
		
TEST_NUM3:
			movlw		0x03
			cpfseq		COUNT
			bra 		TEST_NUM4
			call 		TEST_0_5
			movlw		0x30
			subwf		RX_Temp,W
			movwf		CCC
			movlw		a'c'
			cpfseq 		RX_Temp
			bra 		$+4
			bra 		CLEAR_DATA
			;call		L1homeLCD
			;call 		LLhomeLCD
			movlw		a':'
			call		putcLCD
			return
TEST_NUM4:
			movlw		0x04
			cpfseq		COUNT
			nop
			call 		TEST_0_9
			movlw		0x30
			subwf		RX_Temp,W
			movwf		DDD
			;call		PRESS_TO_START
			;clrf		COUNT
			movlw		a'c'
			cpfseq 		RX_Temp
			return
			bra 		CLEAR_DATA
					
TEST_0_9
			movlw		a'0'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'1'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'2'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'3'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'4'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'5'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'6'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'7'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'8'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'9'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'c'
			cpfseq 		RX_Temp
			bra 		WRONG_DATA
			return
TEST_0_5
			movlw		a'0'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'1'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'2'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'3'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'4'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'5'
			cpfseq 		RX_Temp
			bra 		$+4
			return
			movlw		a'c'
			cpfseq 		RX_Temp
			bra 		WRONG_DATA
			return
			
			
			
			
CLEAR_DATA	
			call		InitLCD
			clrf		COUNT
			return	
WRONG_DATA  
			call		WRONG_FORMAT
			call		delay_1s
			call		InitLCD
			clrf		COUNT
			;decf		STKPTR
			bra		Exit_Low_ISR
			
PRESS_TO_START
			call		L2homeLCD
			movlw 		'P'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		'e'
			call		putcLCD
			movlw 		's'
			call		putcLCD
			movlw 		's'
			call		putcLCD
			movlw 		' '
			call		putcLCD
			movlw 		's'
			call		putcLCD
			movlw 		' '
			call		putcLCD
			movlw 		't'
			call		putcLCD
			movlw 		'o'
			call		putcLCD
			movlw 		' '
			call		putcLCD
			movlw 		's'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			movlw 		'a'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			return
WRONG_FORMAT	
			call		L1homeLCD
			movlw 		'W'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		'o'
			call		putcLCD
			movlw 		'n'
			call		putcLCD
			movlw 		'g'
			call		putcLCD
			movlw 		' '
			call		putcLCD
			movlw 		'F'
			call		putcLCD
			movlw 		'o'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		'm'
			call		putcLCD
			movlw 		'a'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			return
WATERING	
			call		L1homeLCD
			movlw 		'W'
			call		putcLCD
			movlw 		'a'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			movlw 		'e'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		'i'
			call		putcLCD
			movlw 		'n'
			call		putcLCD
			movlw 		'g'
			call		putcLCD
			return
			
Indicate_START	
			call		InitLCD
			call		L2homeLCD
			movlw 		's'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			movlw 		'a'
			call		putcLCD
			movlw 		'r'
			call		putcLCD
			movlw 		't'
			call		putcLCD
			return
change_LCD	
			movlw		.0
			cpfseq		DDD
			bra 		Label1
			cpfseq		CCC
			bra 		Label2
			cpfseq		BBB
			bra 		Label3
			cpfseq		AAA
			bra 		Label4
			bra 		end_check
Label1:	
			decf		DDD
			bra		end_check
Label2:		
			decf		CCC
			movlw		.9
			movwf		DDD
			bra		end_check
Label3:	
			decf		BBB
			movlw		.5
			movwf		CCC
			movlw		.9
			movwf		DDD
			bra		end_check
Label4:		    
			decf		AAA
			movlw		.9
			movwf		BBB
			bra		end_check
end_check:	
			movlw		0x30
			addwf		AAA,F
			addwf		BBB,F
			addwf		CCC,F
			addwf		DDD,F
					
			call		L1homeLCD
			movf		AAA,W
			call		putcLCD
			movf		BBB,W
			call		putcLCD
			;movlw		a':'
			movlw		':'
			call		putcLCD
			movf		CCC,W
			call		putcLCD
			movf		DDD,W
			call		putcLCD
		
			movlw		0x30
			subwf		AAA,F
			subwf		BBB,F
			subwf		CCC,F
			subwf		DDD,F
			return
			
			
delay_1s:		
			movlw		VAL_S		 
			movwf		count_s
loop_s			call 		delay_100ms
			decfsz		count_s,f
			goto		loop_s
			return	
delay_100ms:	
			movlw		VAL_MS		 
			movwf		count_ms
loop_ms			call 		delay_1ms
			decfsz		count_ms,f
			goto		loop_ms
			return	
;
;-------- 1 ms延遲函式-----------
delay_1ms:	 
			movlw		VAL_US		 		
			movwf		count
dec_loop		call 		D_short		; 2 Ins +(12 Ins)
			decfsz		count,f		; 1 Ins
			goto 		dec_loop		; 2 Ins, Total=(12+5)*VAL_US Ins
			return	
;
;--------  5uS延遲函式 -----------
D_short			call		D_ret			; 4 Ins
			call		D_ret			; 4 Ins
			nop					; 1 Ins
			nop					; 1 Ins
D_ret			return				; 2 Ins, Total=12 Ins =8us (10Mhz)
;		
		 	END					
