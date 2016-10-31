#include <p16F690.inc>
	__config (_INTRC_OSC_NOCLKOUT & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOD_OFF & _IESO_OFF & _FCMEN_OFF)
; ---------------------------
; Configuration word register DS41204
;----------------------------
; ====================================================
; Om assembly-programmering
; ====================================================
; DESTINASJONSREGISTER:
; d = 0 (w): lagre resultatet i WORK,
; d = 1 (f): lagre resultatet i filregister 'f' (default). WORK blir ikke endret.
; Fx:  andwf variable,w -> work AND variable -> result to work
; Fx:  andwf variable   -> work AND variable -> result to variable
;
; STATUS register: [- RP1 RP0 -  - Z - C]
;
; SUBTRAKSJON:
; F - W =   (subtract w FROM f -> f minus w)
; set C før subtraksjon. Hvis C blir 0 -> lånt 1 (2's complement)
;
; ADDISJON:
; clear C før addisjon. Hvis C blir 1 -> 1 i mente.
;
; ROTASJON
; Rotasjon går via C. => 9 cp for full rotasjon
;
; ---------------------------
; Erfaringer
; ---------------------------
; 1. Lag en og en modul
; 2. Debug en og en modul
; 3. Bruk en PORT fx PORTC til debugging.
; 4. Debug: LED på minst en pinne. Sjekk hvor langt programmet kjører riktig.

; Fordel å legge initialisering i subrutiner. 
; Da er det lett å kople ut moduler som ikke er debugget.
; La variabler som skal kunne endres underveis være mest mulig public. 
;  - Da er det lettere å endre seinere når koden er minder kjent.
;  - Da kan variablene seinere hentes fra EE og ev "programmeres" der.
; 
; Test eg:
;###
;	bsf PORTB,4
;###


; ====================================================
; PORGRAM:  8 fotoceller brukes til å bestemme posisjon.
; ====================================================
; Lysstyrken på fotocellene leses av idet laserstrålen farer forbi. 
; Styken på signalet brukes til å beregne posisjonen.
; Dette gjøres på følgende måte:
; 1: Lysstyrken på alle fotocellene AD-konverteres.
; 2: Posisjonen beregnes.
;    Dette gjøres på følgende måte:
;    - 1: Finner høysete verdi:
;    - Bruker denne samt verdiene før og etter i beregningen.  
;    - 2: Finner null-laserlys nivået ved å beregne snittet når de tre verdiene ikke er med.
;    - 3: Trekk fra null-laserlys nivået fra de tre verdiene.
;    - 4: Multipliser de tre verdiene med sitt cellenr og summer opp
;    - 5: Summer opp de tre verdien.
;    - 6: Divider den multipliserte summen med summen. Dette er posisjonen.
; 3: Posisjonen mappes over til det tidsrommet som hydraulikken må være på for å nå null-posisjone.  
; I tillegg håndteres situasjoner når laserstrålen forsvinner, er svak eller andre kilder forstyrrer.

;===================================================== 
; 	Variabler 
;=====================================================
	cblock	0x20 ; - 7F 
; 8 x avmålinger
V1
V2
V3
V4
V5
V6
V7
V8
V12
V34
V56
V78
V14
V58
; 
snitt
delta
deltasnitt
antINullSnitt
Vmax
VmaxV
VmaxH
maxL
maxH
snittL
snittH
VmaxNr
sumMax
snitt_5
snittNull 
VmaxSumL
VmaxSum
VmaxSumH
intA8
intB4
res16L
res16H
mulSumL
mulSumH
bits
a16L
a16H
b16L
b16H
posisjon

;-----------
toPosition
fromPosition
newDirection
oldDirection
patternNr
pattern
RL
event
dV    		; actual pwm voltage 
stepLength	; count*64uS, a0 -> a*16+0 = 160 (*64uS) = 10mS
countUp10mS 
mS_Counts 	; count of 10mS timeslots count_10mS
countTimeR
countTimeL
deltastatus
snittDelta


	endc
;-----------------------------------
;  Blokk for mellomlagring
;----------------------------------------------------
	cblock	0x30 ;40???

	endc

	cblock	0x30 
;Mellomlagring av summerte avmålinger 

	endc

;--------------------------------------
; Work og Status lagres i området som er felles for alle blokker (unbanked area) ved Interrupt.
;--------------------------------------
	cblock 0x7D	
mainWork
mainStatus
	endc

;=========================================
; 	Main program 
;=========================================
;	Start og restart.  Kun 4 instruksjoner tilgjengelige før interrupt seksjonen.
	org 0  
	call OPPSETT_AV_ENHETER  ; Sett opp enheter og sett inn verdier
	call OPPSTART            ; Slå på enheter
	goto MAIN

;========================================
; 	Interrupt program
;========================================
	org 4    
	movwf mainWork   ; W -> mainWork
	movfw STATUS     ; STATUS -> W 
	movwf mainStatus ;           W -> mainStatus
;   Page 00
	bcf	STATUS,RP0 ; Page: x0
	bcf	STATUS,RP1 ; Page: 0x

;-----------------------------
; 	Stopp kjøring av hydraulikk ?
;-----------------------------

;-----------------------------
; 	AD konvertering ?
;-----------------------------

;-----------------------------
; 	Tikk fra klokka ?
;-----------------------------

;-----------------------------
; 	SSI kommunikasjon ?
;-----------------------------

;-----------------------------
; Interrupt: Return
;-----------------------------
	movfw mainStatus
	movwf STATUS
	movfw mainWork
 retfie



;The T0IF bit must be cleared in software. 
;-----------------------------
; 	End dV (the time the PWM is on)
;-----------------------------
TIMER0_IR
; Counting the PWM pulse-width (the dV - the voltage applied. 32 = 100%)
	btfss INTCON,T0IF ; IR when True: Skip to TIMER0_IR_TRUE
	goto TIMER0_IR_END
TIMER0_IR_TRUE
; End of PWM pulse. Set all pins off
	clrf PORTB 
TIMER0_IR_END
;-----------------------------
; 	Set pattern, set dV, tell main to produce new pattern and dV
;-----------------------------
TIMER2_IR
; Start a new PWM pulse
	btfss PIR1,TMR2IF ; IR when True: Skip to TIMER2_IR_TRUE
	goto TIMER2_IR_END
TIMER2_IR_TRUE
; set pattern to use
;	movfw pattern
	movwf PORTB ; PORTB Pin 7,6,5,4 ----
; set pwm time = dV (timer0)
	movfw dV
	movwf TMR0 ; when TMR0 is written, the increment is inhibited for the following two instruction cycles
	bcf INTCON,T0IF  ; clear TMR0-IR
	bcf PIR1,TMR2IF  ; clear TMR2-IR
; tell MAIN to find the pattern and pulselength for the next PWM-pulse. 
	bsf event,0 
TIMER2_IR_END
;-----------------------------
; Interrupt: Return
;-----------------------------
	movfw mainStatus
	movwf STATUS
	movfw mainWork
 retfie

;========================================
; MODULER
;  PINNER, PORTER, KLOKKE, AD, TMR0, TMR2, 
;========================================
OPPSETT_AV_ENHETER

;----------------------------------------
; 	OPPSETT AV KLOKKE: Frekvens 8 MHz    
;----------------------------------------
	bsf	STATUS,RP0 ; Page: 01
;   OSCCON: -xxx ---- = The frequency selected
;    xxx = 7->8Mhz 6->4MHz  5->2MHz 4->1MHz 3->500k 2->250k 1-> 125k 0->ekstern 
;   -xxx ---1 = Selecting freq from internal setting
	movlw 0x8f  ; clearing out the old frequency  
	andwf OSCCON 
	movlw 0x71  ; internal osc at 7-> 8 MHz
	iorwf OSCCON 
	bcf	STATUS,RP0 ; Page: 00

;----------------------------------------
; 	OPPSETT AV PINNER
;----------------------------------------
; Pin-retning: i/p=i, o/p=o
; Pin-type: Analog=A, Digital=D

; 2 PortA5 o D FEIL
; 3 PortA4 i A AD3
; 4 PortA3 i D ADTRIG IR
;17 PortA2 i A AD2
;18 PortA1 i A AD1
;19 PortA0 i A AD0    

;10 PortB7 i D xx Debug?
;11 PortB6 i D SPI_Clk (Slave=i/p, master=o/p)
;12 PortB5 i D xx Debug? 
;13 PortB4 i D SPI_Din

; 9 PortC7 o D SPI_Dut
; 8 PortC6 i D SPI_Sla (Modul aktiv ved 0)
; 5 PortC5 o D OPP
; 6 PortC4 o D NED
; 7 PortC3 i A AD7
;14 PortC2 i A AD6
;15 PortC1 i A AD5
;16 PortC0 i A AD4

;Pin-type: Analog=1, Digital=0
; ANSEL  7654 3210 <- Analog pin-channelnr
; ANSELH ---- BA98 <- Analog pin-channelnr (11-8)
	bsf	STATUS,RP1	; Page: 10
	movlw 0x00 ; TEST 0xff      ; ANSEL: 1111 1111, 0-7 analog
	movwf ANSEL     
	movlw 0x00      ; ANSEH: ---- 0000, 8-B digital
	movwf ANSELH	
	bcf	STATUS,RP1  ; Page: 00

;Pin-retning:  i/p=1,  o/p=0
	bsf	STATUS,RP0 ; Page: 01
	movlw	0x3f  ; PORTA retning --01 1111    
	movwf	TRISA 
	movlw	0x0f  ; PORTB retning 1111 ----    
	movwf	TRISB 
	movlw	0x00 ;0xff  ; PORTC retning 0100 1111 ; TEST: 0000 0000
	movwf	TRISC 
	bcf	STATUS,RP0 ; Page: 00

;Initialisering av port A,B og C
	clrf PORTA ; setter verdiene til 0
	clrf PORTB ; setter verdiene til 0
	clrf PORTC ; setter verdiene til 0

;--------------------------------------------
; OPPSETT AV AD
;--------------------------------------------
; AD-unit on/off, input voltage span, result format
; Enhet PÅ=1, AV=0
	bcf ADCON0,ADON ; AD-modul slås AV , og slås på i OPPSTART
; Spennings-ref: Vdd=0, internal=1
	bcf ADCON0,VCFG ; Bruker Vdd som spennings-ref
; Resultatorganisering: 8+2bits=0, 2+8bits=1
	bcf ADCON0,ADFM ; Bruker 8 fra 8+2
; Konverteringstid (Tad) må være mellom 1,6uS og 6,5uS
; ADCON1 -xxx ---- 
;         101 8Mhz & Fosc/16 -> Tad = 2uS        
;         010 8Mhz & Fosc/32 -> Tad = 4uS
	bsf	STATUS,RP0 ; Page: 01
	movlw 0x50 ; -101 ---- 8Mhz & Fosc/16 -> Tad = 2uS
	andwf ADCON1 
	bcf	STATUS,RP0 ; Page: 00

; Velg startkanal for konverteringene
; ADCON0 --xx xx--  0000=AN0 etc
	movlw 0xc3 ; --00 00-- Velger AN0 først
	andwf ADCON0

;--------------------------------------------
; SEINERE: OPPSETT AV TIMER 2  time = 512 cp -> 64uS
;--------------------------------------------
; IR-flag is OFF - Interrupt gets enabled in STARTUP
	bcf PIR1,TMR2IF ; Clear IR-flag

; Timer interrupt every 512cp:
;   Frequency:   	8 MHz
;   Oscillator:		Fosc/4
;   Pre scaling: 	1:1	 T2CON: _--- --00
;   Count:       	64 	 PR2:   0100 0000
;   Post scaling:	1:2  T2CON: _000 1--- 
;	= 8Mhz/4/1/64/2 = 64 uS interval

;T2CON: _--- --xx Prescaler  00=1:1, 01=1:4, 1x=1:16
;T2CON: _xxx x--- Postscaler 0000=1:1 .. 1111=1:16
;T2CON: _--- -x-- H:Timer2=ON L:Timer2=OFF
;PR2:   xxxx xxxx TMR2 is increased and reset to zero after PR2 = TMR2   
	movlw 0x08 ; T2CON: pre + post + on: _000 1000
	movwf T2CON
	bsf	STATUS,RP0  ; Page: 01 
	movlw 0x40 ; PR2: 0100 0000 = 64
	movwf PR2 
	bcf	STATUS,RP0  ; page: 00

;--------------------------------------------
; SEINERE 	OPPSETT AV TIMER 0  PWM pulse duration (dV) 32 individual voltagelevels)
;--------------------------------------------
; PWM pulse time = 512 cp:
; Timer0 PWM pulse length: Fosc/4 * 1:4 * 32 (max count) = 512 cp
; Interrupt when FF->00

; OPTION_REG: x--- ----  RABPU: pullUp: H=Disabled
; OPTION_REG: -x-- ----  INTEDG: IR: H = on rising edge
; OPTION_REG: --x- ----  T0CS: Clock source: L= internal
; OPTION_REG: ---x ----  T0SE: IncrementEdge: H= 1->0
; OPTION_REG: ---- x---  PSA: Prescaler assignment: L=Timer0
; OPTION_REG: ---- -xxx  PS0-2: Prescaler: 000=1:2 ... 111=1:256
; OPTION_REG: 1000 0001  Prescale 1:4

; IE is disabled - Interrupt gets enabled in STARTUP
	bcf INTCON,T0IE ; Disable IE (INTCON<5>)
	bcf INTCON,T0IF ; Clear IF (INTCON<2>)
; T0 with prescaler
	bsf	STATUS,RP0  ; Page: 01 
	movlw 0x81	; OPTION_REG: 1000 0001  Prescale 1:4
	movwf OPTION_REG	
	bcf	STATUS,RP0  ; page: 00
; Clear T0 - Prescaler also cleared in the same go.
; When TMR0 is written, the increment is inhibited for the following two instruction cycles
	clrf TMR0

OPPSTART
;========================================
;    START ENHETENE
;========================================
	bsf ADCON0,ADON ; ; Set A/D-unit ON
;	bsf T2CON,2 ; Set TMR2-unit on
;   TMR0-unit always on
	bsf	STATUS,RP0 ; Page 01
;	bsf PIE1,TMR2IE  ; Timer2 interrupt enabled
;	bsf INTCON,T0IE  ; Timer0 interrupt enabled
;	bsf INTCON,PEIE  ; Pheripheral units interrupt enabled (TMR2)
;	bsf INTCON,GIE   ; Global interrupt enabled
	bcf	STATUS,RP0 ; Page 00

 return

;========================================
;    SETUP MAIN
;========================================
OPPSTART
	movlw 0xfa ; TEST
	movwf V1
	movwf V2
	movwf V3
	movwf V4
	movwf V5
	movwf V6
	movwf V7
	movwf V8

 return

;========================================
;    MAIN RUN
;========================================
MAIN

MAIN_LOOP
; stopp...
; AD-bit = 1
; tic-bit = 1
; set verdi i SSP-byte

; ----------------------
; FINN POSISJON
; 1: Finner først snittet samt den høyeste verdien. 
; 2: Trekker fra verdien til den høyeste og de to på sidene fra snittet og deler på 5.
;    Har nå funnet null-nivået. 
; 3: Trekker null-nivået fra de tre verdiene som skal brukes i beregningen. 
; 4: Multipliserer og deler og finner posisjonen
; ----------------------
; 1: Finn snitt av alle. 
; (((V1+V2)/2 + (V3+V4)/2)/2 + ((V5+V6)/2 + (V7+V8)/2)/2)/2 -> snittAlle.
; Shift R med Carry => Summer og del med to.
;(V1+V2)/2 
	bcf STATUS,C
	movfw V1
	addwf V2,w
	movwf V12
	rrf V12 ;
;(V3+V4)/2 
	bcf STATUS,C
	movfw V3
	addwf V4,w
	movwf V34
	rrf V34 
;(V5+V6)/2 
	bcf STATUS,C
	movfw V5
	addwf V6,w
	movwf V56
	rrf V56 
;(V7+V8)/2 
	bcf STATUS,C
	movfw V7
	addwf V8,w
	movwf V78
	rrf V78 
;(V12+V34/2) ---
	bcf STATUS,C
	movfw V12
	addwf V34,w
	movwf V14
	rrf V14
;(V56+V78/2)
	bcf STATUS,C
	movfw V56
	addwf V78,w
	movwf V58
	rrf V58
;(V14+V58/2)
	bcf STATUS,C
	movfw V14
	addwf V58,w
	movwf snitt
	rrf snitt
;#########################
	movfw snitt
	movwf PORTC 
;#########################
; 2: Hvis snitt + delta > 255 -> feil
	clrf deltasnitt ; snitt + delta
DELTASNITTSUM
	bcf STATUS,C     ; clear C
	movfw snitt      ; snitt -> w -> deltasnitt
	movwf deltasnitt 
	movfw delta      ; delta -> w -> w+deltasnitt -> deltasnitt
	addwf deltasnitt 
	btfss STATUS,C  ; C=1? : Skip to DELTASNITTSUM_NOT_OK
	goto DELTASNITTSUM_END
DELTASNITTSUM_NOT_OK
	movlw 0x01 ; feil nr 1: Snitt + delta er for høy
    goto FEIL
DELTASNITTSUM_END

; 3: Finn den med høyest nivå samt målingene ved sida
	clrf antINullSnitt ; Antall målinger som nullsnitt består av 
; V1 > Vmax?  (Vmax-V1 -> Låneflagg satt -> V1 er høyest )
V1_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V1       ; V1 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V1 er høyere: Skip to TRUE 
	goto V1_ER_MAX_END
V1_ER_MAX_TRUE
	clrf VmaxV
	movfw V1       ; V1 -> w, w -> Vmax
	movwf Vmax
	movfw V2       ; V1 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x06	   ; 6 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x00	   ; Vmax nr
	movwf VmaxNr
V1_ER_MAX_END
; V2 > Vmax? 
V2_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V2       ; V2 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V2 er høyere: Skip to TRUE 
	goto V2_ER_MAX_END
V2_ER_MAX_TRUE
	movfw V1       ; V1 -> w, w -> VmaxV
	movwf VmaxV
	movfw V2       ; V2 -> w, w -> Vmax
	movwf Vmax
	movfw V3       ; V3 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x01	   ; Vmax nr
	movwf VmaxNr
V2_ER_MAX_END
; V3 > Vmax? 
V3_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V3       ; V3 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V3 er høyere: Skip to TRUE 
	goto V3_ER_MAX_END
V3_ER_MAX_TRUE
	movfw V2       ; V2 -> w, w -> VmaxV
	movwf VmaxV
	movfw V3       ; V3 -> w, w -> Vmax
	movwf Vmax
	movfw V4       ; V4 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x02	   ; Vmax nr
	movwf VmaxNr
V3_ER_MAX_END
; V4 > Vmax? 
V4_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V4       ; V4 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V4 er høyere: Skip to TRUE 
	goto V4_ER_MAX_END
V4_ER_MAX_TRUE
	movfw V3       ; V3 -> w, w -> VmaxV
	movwf VmaxV
	movfw V4       ; V4 -> w, w -> Vmax
	movwf Vmax
	movfw V5       ; V5 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x03	   ; Vmax nr
	movwf VmaxNr
V4_ER_MAX_END
; V5 > Vmax? 
V5_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V5       ; V5 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V5 er høyere: Skip to TRUE 
	goto V5_ER_MAX_END
V5_ER_MAX_TRUE
	movfw V4       ; V4 -> w, w -> VmaxV
	movwf VmaxV
	movfw V5       ; V5 -> w, w -> Vmax
	movwf Vmax
	movfw V6       ; V6 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x04	   ; Vmax nr
	movwf VmaxNr
V5_ER_MAX_END
; V6 > Vmax? 
V6_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V6       ; V6 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V6 er høyere: Skip to TRUE 
	goto V6_ER_MAX_END
V6_ER_MAX_TRUE
	movfw V5       ; V5 -> w, w -> VmaxV
	movwf VmaxV
	movfw V6       ; V6 -> w, w -> Vmax
	movwf Vmax
	movfw V7       ; V7 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x05	   ; Vmax nr
	movwf VmaxNr
V6_ER_MAX_END
; V7 > Vmax? 
V7_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V7       ; V7 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V7 er høyere: Skip to TRUE 
	goto V7_ER_MAX_END
V7_ER_MAX_TRUE
	movfw V6       ; V6 -> w, w -> VmaxV
	movwf VmaxV
	movfw V7       ; V7 -> w, w -> Vmax
	movwf Vmax
	movfw V8       ; V8 -> w, w -> VmaxH
	movwf VmaxH
	movlw 0x05	   ; 5 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x06	   ; Vmax nr
	movwf VmaxNr
V7_ER_MAX_END
; V8 > Vmax? 
V8_ER_MAX
	bsf STATUS,C   ; Sett borrow 
	movfw V8       ; V8 -> w, Vmax-w -> w
	subwf Vmax,w   ; SUB: f-w 
	btfsc STATUS,C ; C=0? : Har lånt, V8 er høyere: Skip to TRUE 
	goto V8_ER_MAX_END
V8_ER_MAX_TRUE
	movfw V7       ; V7 -> w, w -> VmaxV
	movwf VmaxV
	movfw V8       ; V8 -> w, w -> Vmax
	movwf Vmax
	clrf VmaxH
	movlw 0x06	   ; 6 målinger i nullsnitt
	movwf antINullSnitt
	movlw 0x07	   ; Vmax nr
	movwf VmaxNr
V8_ER_MAX_END

; 4. Hvis Vmax < delta+snitt -> feil (deltasnitt-Vmax)
VMAX_UNDER_DELTASNITT
	bsf STATUS,C        ; Sett borrow 
	movfw Vmax          ; Vmax -> w, deltastatus-w -> deltastatus
	subwf deltastatus,w ; SUB: f-w 
	btfss STATUS,C      ; C=1? : Har ikke lånt. Vmax for lav: Skip to FALSE 
	goto VMAX_UNDER_DELTASNITT_END
VMAX_UNDER_DELTASNITT_FALSE
	movlw 0x02 ; feil nr 2: Vmax ikke høy nok til å telle
    goto FEIL
VMAX_UNDER_DELTASNITT_END

; 5: Summer opp max-verdiene - og del dem på 8.
	clrf sumMax
	bcf STATUS,C ; clear minne 
	movfw VmaxV  ; VmaxV -> sumMax
	movwf sumMax
	movfw Vmax    ; sumMax+Vmax -> sumMax
	addwf sumMax  
	rrf sumMax
	bcf STATUS,C  ; clear minne 
	rrf VmaxH     ; ---NB verdien er endret
	bcf STATUS,C  ; clear minne 
	movfw VmaxH   ; sumMax+VmaxH -> sumMax
	addwf sumMax
    rrf sumMax
	bcf STATUS,C  ; clear minne 
    rrf sumMax

; 6. Tekk fra max-verdiene fra snitt
	movfw snitt   ; snitt -> w, w -> snitt5
	movwf snitt_5     
	bsf STATUS,C  ; Sett borrow 
    movfw sumMax  ; snitt5-sumMax -> snitt5
	subwf snitt_5  ; SUB: f-w 

; 7. Dele på 5/6    (snitt_5*33h -> snittNull) 
; IF antINullsnitt == 5
NULLSNITT_DIV
	bsf STATUS,C  ; Sett borrow 
	movfw antINullSnitt
	sublw 0x05    ; SUB: c-w 
	btfss STATUS,Z ; Er null. Skip to DIV_5
	goto NULLSNITT_DIV_6
NULLSNITT_DIV_5
;snitt_5/1 + snitt_5/2 + snitt_5/16 + snitt_5/32 -> 
	movfw snitt_5 
	movwf snittNull 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	movfw snitt_5
	addwf snittNull
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	movfw snitt_5
	addwf snittNull
	goto NULLSNITT_DIV_END
NULLSNITT_DIV_6
	movfw snitt_5 
	movwf snittNull 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	movfw snitt_5
	addwf snittNull
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	movfw snitt_5
	addwf snittNull
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	bcf STATUS,C  ; Clear minne
	rrf snitt_5 
	movfw snitt_5
	addwf snittNull
NULLSNITT_DIV_END

; 8: Korriger max-verdier: max - snittNull
	bsf STATUS,C    ; Set borrow
	movfw snittNull
	subwf VmaxV     ; f-w
	btfss STATUS,C  ; C=1? : Har ikke lånt. Skip nullstilling
	clrf VmaxV      ; nullstill hvis VmaxV er negativ
	bsf STATUS,C    ; Set borrow
	movfw snittNull
	subwf Vmax      ; f-w
	btfss STATUS,C  ; C=1? : Har ikke lånt. Skip nullstilling
	clrf Vmax       ; nullstill hvis VmaxV er negativ
	bsf STATUS,C    ; Set borrow
	movfw snittNull
	subwf VmaxH     ; f-w
	btfss STATUS,C  ; C=1? : Har ikke lånt. Skip nullstilling
	clrf VmaxH      ; nullstill hvis VmaxV er negativ

; 9: summer: VmaxL, Vmax, VmaxH
	movfw VmaxV
	movwf VmaxSumL
	movfw Vmax
	addwf VmaxSumL
	movlw 0x00
	addwf VmaxSumH
	movfw VmaxH
	addwf VmaxSumL
	movlw 0x00
	addwf VmaxSumH
;10: multipliser: VmaxL*n, Vmax*(n+1), VmaxH*(n+2)
	;VmaxL * VmaxNr
	movfw VmaxV		;VmaxL -> intA8
	movwf intA8
	movfw VmaxNr	;VmaxNr -> intB4
	movwf intB4
	call MULT8x4
	movfw res16L	;res16L -> resMultL
	movwf mulSumL
	movfw res16H	;res16H -> resMultH
	movwf mulSumH	
	;VmaxL * (VmaxNr+1)
	incf VmaxNr		;VmaxNr+1
	movfw Vmax		;VmaxL -> intA8
	movwf intA8
	movfw VmaxNr	;VmaxNr -> intB4
	movwf intB4
	call MULT8x4
	movfw res16L	;res16L +> resMultL
	addwf mulSumL
	movfw res16H	;res16H +> resMultH
	addwf mulSumH	
	;VmaxL * (VmaxNr+2)
	incf VmaxNr		;VmaxNr+1
	movfw VmaxH		;VmaxL -> intA8
	movwf intA8
	movfw VmaxNr	;VmaxNr -> intB4
	movwf intB4
	call MULT8x4
	movfw res16L	;res16L -> resMultL
	addwf mulSumL
	movfw res16H	;res16H -> resMultH
	addwf mulSumH
	
; 11: Divisor ganges med 8. 
	bcf STATUS,C 	; Clear borrow
	rlf VmaxSumL 		; ganger med 8
	rlf VmaxSumH
	rlf VmaxSumL
	rlf VmaxSumH
	rlf VmaxSumL
	rlf VmaxSumH

; 12: Divider 16:16=8
;  intA16:intB16 = (posisjon)
; AA-BB < 0? -> 0/1 shiftes inn i posisjon
DIV16
	clrf posisjon
	movlw 0x08  ; 8 counts 
	movwf bits
DIV16_LOOP
; telt ned?
	movfw bits
	btfsc STATUS,Z 	; 0 = not zero: Skip to MULT16_TRUE
	goto DIV16_END 	; ferdig
	decf bits
DIV16_TRUE     		; a16L-b16L, a16H-b16L 
	bsf STATUS,C 	; Set borrow
	movfw a16L
	subwf b16L,w
	movfw a16H
	subwf b16H,w
	rrf posisjon		; Shift inn 1 eller 0 
	btfss STATUS,C 	; H = Ikke lånt - kan trekke fra: Skip to SUBTRACT
	goto DIV16_SUB_END
	bsf STATUS,C 	; Set borrow
	movfw a16L
	subwf b16L,w
	movwf a16L
	movfw a16H
	subwf b16H,w
	movwf b16H
	bcf STATUS,C 	; Clear borrow
	rrf b16H 		; result bit shifted in 
	rrf b16L
DIV16_SUB_END
	goto DIV16_LOOP
DIV16_END





; --- SUB -----------------------------
;           MULTIPLY 8x4->16
;---------------------------------------
;intA8 x intB4 = res16
MULT8x4          
	clrf res16L
	clrf res16H
	movlw 0x04  ; 4 shifts
	movwf bits ; 
MULT8
; shift and add
	movfw bits
	btfsc STATUS,Z ; 0 = not zero: Skip to MULT16_TRUE
	goto MULT8_END ; is zero
MULT8_TRUE
;Rotate intB4 -> C -> intB4 -> C 
	bcf STATUS,C
	rrf intB4
; if C = H add
MULT8_ADD
	btfss STATUS,C    ; C=1: Skip to MULT8_ADD_TRUE
	goto MULT8_ADD_END
MULT8_ADD_TRUE
	bcf STATUS,C
; Add intA8 to res16H - shiftes mot R
	movfw intA8
	addwf res16H
MULT8_ADD_END
; Shift res32 ->
	rrf res16H
	rrf res16L
	decf bits
	goto MULT8
MULT8_END
	return



FEIL





;-----------------
TIC
	btfss event,0  ; Bit0 = True?: Skip to TIC_TRUE
	goto TIC_END	
TIC_TRUE
	bcf event,0  ; event handled
	movfw PORTC

;--------------------------
;  Count each 10 mS
;--------------------------
COUNT_UP_10mS
	incf countUp10mS
	movfw countUp10mS
	xorlw 0xa0 ; 10*16 * 64uS = 160 * 64uS -> 10mS
	btfss STATUS,Z ; True: Skip to COUNT_UP_10mS_TRUE
	goto COUNT_UP_10mS_END
COUNT_UP_10mS_TRUE
	clrf countUp10mS
;--------------------------
;  Forced left stepping - one step each 100mS
;--------------------------
LEFTWARDS
	movfw RL
	xorlw 0x40 ; -L-- 0---
	btfss STATUS,Z ; True: Skip to LEFTWARDS_TRUE
	goto LEFTWARDS_FALSE
LEFTWARDS_TRUE
	incf countTimeL ; add 10mS
LEFT_STEP
	movfw countTimeL 
	xorlw 0x0a ; 10 * 10mS = 100mS
	btfss STATUS,Z ; True: Skip to LEFT_STEP_TRUE
	goto LEFT_STEP_END
LEFT_STEP_TRUE
; Sier at fromPosition er et step til høyre. Må da gå et steg til venstre..
     decf fromPosition ; 
     clrf countTimeL
LEFT_STEP_END
	goto LEFTWARDS_END
LEFTWARDS_FALSE
  clrf countTimeL
LEFTWARDS_END


;--------------------------
;  AD - each 100mS
;--------------------------
	incf mS_Counts
START_AD ; every 100mS
	movfw mS_Counts
	xorlw 0x0a ; = 10mS*10 = 100mS
	btfss STATUS,Z ; True: Skip to START_AD_TRUE
	goto START_AD_END
START_AD_TRUE
	clrf mS_Counts
; Get new toPosition
	call AD_CONVERT ; Result in work
	movwf toPosition 
;   bcf STATUS,C
; 	rrf toPosition  ; 128 positions max
START_AD_END
COUNT_UP_10mS_END

;--------------------------
;	Direction 
;--------------------------
; newDirection -> oldDirection
	movfw newDirection
	movwf oldDirection
; toPosition - fromPosition => STATUS -> newDirection.  C=1 -> Right
; STATUS ---- -Z-C >> newDirection
; 		 ---- -1-x : Rest (Zero)
; 		 ---- -0-0 : Left
; 		 ---- -0-1 : Right
	movfw toPosition
	bsf STATUS,C 	; if C goes L, the subtraction has borrowed (2-compl addition)
	subwf fromPosition,w ; fromPosition - toPosition => w.  C=1 -> R
	movfw STATUS
	andlw 0x05  ; ---- -x-x
	movwf newDirection 

;--------------------------
;	Pattern to use (decided for each PWM)
;--------------------------
	movfw patternNr
	andlw 0x03  ; ---- --xx
    addlw 0x30 ; pattern area: 30-33
	movwf FSR  ; File index Register
	movfw INDF ; Value in indexedFile[FSR]
    movwf pattern

FREE_WHEELING ; if (RL == 11) : pattern = 0000 
	movfw RL
	xorlw 0x48 ; -L-- R---
	btfss STATUS,Z ; True: Skip to FREE_WHEELING_TRUE
	goto FREE_WHEELING_END
FREE_WHEELING_TRUE
    clrf pattern
FREE_WHEELING_END

TIC_END
	goto MAIN_LOOP


;========================================
;         SUBROUTINES  
;========================================
AD_CONVERT ; result in work
	bsf	ADCON0,GO   		; Start convertion 
AD_CONVERTION_GOING_ON 		; Wait for conversion to complete
	btfsc ADCON0,GO 		; Finished when L: Skip to AD_CONVERT_FINISHED_END
	goto AD_CONVERTION_GOING_ON
AD_CONVERT_TRUE
	; Change scale to 128 bit 
	bcf STATUS,C  ; Rotate (bit0 -> C -> bit7)
	rrf ADRESH,w  ; 8bit + 2bit : ADRESH + ADRESL: ADRESH->w
	bcf STATUS,C  ; clear carry 
AD_CONVERT_END
	return







	end
