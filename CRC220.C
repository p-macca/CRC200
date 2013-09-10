//CRC220.C
//Version: 1.0 - 25/2/2005
//Version: 1.01 - 
//Version: 1.02 - 14/11/06 - Random card option (switch 7 on)
//Version: 1.03 - 10/01/07 - Fixed atb
//Version: 1.04 - 02/03/07 - Fixed random cards > 5 digits (int16 instead of int32)
//Version: 1.05 -
//Version: 1.06 - 05/06/07 - Arming inputs (aux in2 = rdr1, aux in3 = rdr2)
//Version: 1.07 - 14/01/08 - Inputs 4-8 implented
//Version: 1.08 - 25/11/08 - ATB fixed, speedup of reader io, 
//						 	 aux-out1 on if tamper or door forced or door left open
//							 atb reset now also resets atb
//							 outputs can be controlled from PC 
//Version: 1.09 - 05/12/08 -
//Version: 1.10 - 10/03/09 - Rework door monitoring etc.
//Version: 2.00 - 11/03/09 - PIN + Card added
//Version: 2.01 - 06/07/09 - Access level 0, card always rejected (accesslevel)
//Version: 2.02 - 22/09/09 - Single door mode, buddy mode, Open all doors on input 4
//Version: 2.03 - 18/01/10 - removed call to iocheck for auxin 7 and 8
//Version: 2.04 - 08/02/10 - Auxo1 = door1 alarm, Auxo2 = door2 alarm
//							 Rdr1 cancels door 1 alarm, Rdr2 cancels door 2 alarm
//Version: 2.06 - 26/05/10 - Passback added
//Version: 2.07 - 09/07/10 - Fixed error in single door mode on reader 2 still active
//							 when door open
//Version: 2.08 - 13/09/10 - Fixed Relay 2 0.5 second option 
//Version: 2.09 - 21/12/10 - Option to compile for single door only. (define ONEDOOR)
//Version: 2.10 - 16/03/11 - Buddy mode now sends both card numbers to host.
//							 Improvements for TCP timing.
//Version: 2.11 - 04/10/11 - Latching output relay added
//						  	 Muster controller option added.
//Version: 2.12 - 01/03/12 - 16 door open timezones. Anti-tailback off option
//							 Latching output changed - latch cards presented 3x will latch
//							 Door Normal will now cancel door alarm providing door is closed
//							 Changed relay behaviour. Relays will operate regardless of door 
//							 state providing card/access level/time period/arming all valid
//							 Fixed reader 2 LED operation errors in single door mode
//							 Fixed latching open when not armed
//							 Door open timezone now closes a door set/latched open
//Version: 2.13 - 02/10/12 - Toggledoor LED flash moved to inside timer1
//							 Relay activation counts added (command 0xf2) - this version is no good. 2.16 fixes
//							 Default settings applied on initial power up
//							 Accomodation cards added
//							 Door open timezone now always unlocks regardless of door state
//							 Fixed door 1 left open state failing to report door closed
//Version: 2.14 - 27/06/13 - PIN only functionality added - with access levels
//Version: 2.15 - 18/07/13 - Fixed card/REX while DOOR_RELOP resetting door alarm timer
//							 Fixed PIN on rdr2 causing green LED on rdr1 remaining lit
//Version: 2.16 - xx/xx/xx - Fixed relay counts not storing larger numbers

#include <18F6722.H>

//#define	ONEDOOR

#if defined(ONEDOOR)
#define MODVER 5
#else
#define MODVER 4
#endif 

#byte	porta		=	0x0f80
#byte	portb		=	0x0f81
#byte	portc		=	0x0f82
#byte	portd		=	0x0f83
#byte	porte		=	0x0f84
#byte	portf		=	0x0f85

#byte	RCREG		=	0x0FAE		//uart receive reg
#byte	INDF0		=	0x0FEF		//indirect reg
#byte	RCON		=	0x0FD0
#byte	LDVCON		=	0x0FD2		//low voltage detect	
#byte	STATUS		=	0x0FD8
#byte	FSR0H		=	0x0FEA
#byte	FSR0L		=	0x0FE9

#byte	CCP1CON		=	0x0FBD
#byte	CCP2CON		=	0x0FBA
#byte	CCP3CON		=	0x0FB7
#byte	CCP4CON		=	0x0F73
#byte	CCP5CON		=	0x0F70

#byte	T4CON		=	0x0F76

#byte	RCSTA		=	0x0F8B
#byte	TXSTA		=	0x0FAC

#byte	_pir1		= 	0x0F9E		//PIR1 reg

#byte	INTCON		=	0x0FF2
#byte	INTCON2		=	0x0FF1
#byte	INTCON3		=	0x0FF0

#bit	_carry		= 	STATUS.0
#bit	_zero		= 	STATUS.2
#bit	_rp0		= 	STATUS.5	//bank select 0

#bit	rel1		=	porta.5

#bit	TX9D		=	TXSTA.0
#bit	TRMT		=	TXSTA.1
#bit	TX9			=	TXSTA.6

#bit	RX9			=	RCSTA.6


#define major_ver	0x02			//software version
#define minor_ver	15

#use delay(clock=40000000, RESTART_WDT)
#use rs232(baud = 9600, xmit = PIN_C6, rcv = PIN_C7, RESTART_WDT, ERRORS)
#fuses H4, PROTECT, BROWNOUT, WDT, WDT16, BORV27, PUT, NOLVP, NOCPD, NOSTVREN, NODEBUG, NOCPB, MCLR
//#fuses H4, NOPROTECT, NOBROWNOUT, NOWDT, WDT16, BORV27, NOPUT, NOLVP, NOCPD, NOSTVREN, DEBUG, NOCPB, MCLR
//*************************************************************************************************
//	H4			-	PLL - 4 x clock
//	PROTECT		-	Protect program memory
//	BROWNOUT	-	Brown-out enabled
//	BORV42		-	Brown-out voltage = 4.2V
//	WDT8		-	Watch-dog postscale = 8
//	PUT			-	Power up timer on
//	NOLVP		-	No low voltage program
//*************************************************************************************************

#include <ctype.h> 
#include <string.h>
#include <stdlib.h>
#include "mem200.c"
#include "25C512spi.c"
//#include <bootloader.h>
//
#define	relay1	PIN_A5			//relay 1
//#define	relay2	PIN_A4		//relay 2 (Ver 1.1)
#define relay2	PIN_C1			//relay 2 (Ver 1.2)

#define rdr2out	PIN_B0			//reader2 out clk
#define rdr2in	PIN_B1			//reader2 in clk
#define rdr1out	PIN_B2			//reader1 out clk
#define rdr1in	PIN_B3			//reader1 in clk
#define rdr2dat	PIN_B0			//reader2 data
#define rdr1dat	PIN_B2			//reader1 data

#define rex1	PIN_D0			//REX 1 input	
#define door1	PIN_D1			//door 1 input	
#define rex2	PIN_D2			//REX 2 input	
#define door2	PIN_D3			//door 2 input	
#define rled2	PIN_C2			//reader 2 red led (capture)
#define	gled2	PIN_C5			//reader 2 green led
#define rled1	PIN_C0			//reader 1 red led (capture)
#define	gled1	PIN_A4			//reader 1 green led

#define	tamper	PIN_A0			//tamper input
#define auxi1	PIN_A0			//AUX input 1	
#define auxi2	PIN_A1			//AUX input 2	
#define auxi3	PIN_A2			//AUX input 3	
#define auxi4	PIN_A3			//AUX input 4	
#define auxi5	PIN_G4			//AUX input 5	
#define auxi6	PIN_G0			//AUX input 6	
#define auxi7	PIN_E0			//AUX input 7	
#define auxi8	PIN_E1			//AUX input 8	
#define auxo1	PIN_E2			//AUX output 0	
#define auxo2	PIN_E3			//AUX output 1	
#define auxo3	PIN_E4			//AUX output 2	
#define auxo4	PIN_E5			//AUX output 3	
#define	led		PIN_E6			//on-line led
#define	txon	PIN_E7	      	//RS485 tx on


//#define	SDOOR	PIN_F6			//single door
//#define	wieg	PIN_F7		//sw8 wiegand if on

#define	KCARD	0x00			//aborted due to card
#define	KGOOD	0x01			//good number
#define	KPROG	0x02			//aborted due to prog key
#define	KZERO	0x03			//CR pressed, no data

#define	STX	0x02				//start of text
#define	ETX	0x03				//end of text
#define	ACK	0x06				//acknowledge
#define	NAK	0x15				//not acknowledge
#define	CR	0x0D				//carriage return

#define DBOUNCE		15			//input debounce value
//card database
#define	MAXCARD		20000		//max cards    
#define	CRDSIZE		3			//# of bytes per card    
#define	CRDTOP		MAXCARD * CRDSIZE		
#define	EVTBASE		20010		//start of event storage
#define	EVTMAX		1300		//max number of events
#define	EVTSIZE		9			//number of bytes in event
#define	EVTTOP		EVTBASE + (EVTMAX * EVTSIZE)
#define	CBASE		32000		//configuration base
//Random card database
#define	MAXCARDR	9000		//max cards    
#define	CRDSIZER	7			//# of bytes per card
#define	PINSIZE		2			//# of bytes per pin    
#define	CRDTOPR		MAXCARDR * CRDSIZER		
#define	EVTMAXR		1300		//max number of events
#define	EVTSIZER	9			//number of bytes in event
#define	EVTTOPR		EVTBASE + (EVTMAXR * EVTSIZER)

#define	MAGDIGS		40			//max mag card digits 
//RS485 Timing
#define TXDEL		5
#define TXDE		50
//Reader status
#define	CARD_NO		0			//no card data
#define	CARD_PRO	1			//card read in progress
#define	CARD_RDY	2			//card has been read
#define	CARD_PIN	3			//PIN digit has been read
#define CARD_WAIT	4			//wait for PIN
#define	CARD_HOST	5			//card to host
#define CARD_SENT   8			//card data sent to host
#define CARD_ACC    9			//card accepted
#define CARD_NAK    10			//card rejected
#define CARD_ACT    11			//relay activated
#define CARD_DOOR   12			//door open
#define CARD_NOPEN  13			//door not opened
#define CARD_LEFT   14			//door left open
//Reader status
enum RDR_STATS {RDR_NO, RDR_PIN};		
//Door action status
//Door status
#define	DOOR_NO		0			//no activity
#define	DOOR_REL	1			//relay active
#define	DOOR_RELOP	2			//opened
#define	DOOR_LOP	3			//left open
#define	DOOR_FOP	4			//forced open
#define	DOOR_SOP	5			//set open	
#define	DOOR_TZOP	6			//opened on timezone
#define	DOOR_LOCK	7			//locked	
//PIN Status
#define	PIN_NO		0			//no PIN in progress
#define	PIN_PRO		1			//PIN in progress
#define	PIN_RDY		2			//PIN ready
//RS485 receive status
#define RX_NO		0x00		//no RS485 message
#define	RX_PRO		0x01		//message in progress
#define	RX_POLL		0x02		//single byte poll ready
#define	RX_DATA		0x03		//data command available
//Mag card decode status
#define MAG_NO      0x00        //no mag data
#define MAG_ST      0x01        //start char found
#define MAG_END     0x02        //end char found 
//Door status
#define	DR_CLOSE	0			//door closed
#define	DR_OPEN		1			//door open
//Mode
#define	MD_LOCK		0			//door locked
#define	MD_CARD		1			//card only
#define MD_CORP		2			//card or pin
#define MD_CAP		3			//card and pin
#define	MD_ULOCK	4			//door unlocked
#define	MD_FAC		5			//fac only
//Card read error codes
#define	RD_OK		0			//card ok
#define	RD_PARITY	1			//card parity error
#define	RD_LRC		2			//card lrc error
#define	RD_SITE		3			//site error
#define	RD_UNK		4			//card error unknown
#define RD_FMT		5			//card format error

#define	FMT_MAG		0			//Mag card format
#define	FMT_WIEG	1			//Wiegand card format
//Modes of operation
#define OFF_LINE	0			//off line operation
#define	ON_LINE		1			//on line operation
//Card database options
#define	DB_STD		0			//65,536 sequential cards, no PIN
#define	DB_PIN		1			//20,000 sequential cards + PIN
#define	DB_RAN		2			//9,000 random cards + PIN
#define	DB_PRAN		3			//
//Log events
#define	E0_TPOK		0x0C		//tamper ok
#define	E0_TPON		0x0D		//tamper on
#define	E0_ARM1		0x0E		//reader 1 armed
#define	E0_NARM1	0x0F		//reader 1 not armed
#define	E0_ARM2		0x10		//reader 2 armed
#define	E0_NARM2	0x11		//reader 2 not armed
#define	E0_IO4		0x12
#define	E0_NIO4		0x13
#define	E0_IO5		0x14
#define	E0_NIO5		0x15
#define	E0_IO6		0x16
#define	E0_NIO6		0x17
#define	E0_IO7		0x18
#define	E0_NIO7		0x19
#define	E0_IO8		0x1A
#define	E0_NIO8		0x1B
#define	E0_OPALL	0x1C		//open all doors

#define	E0_POR		0x20		//Power on reset
#define	E0_BOR		0x21		//Brown out reset
#define	E0_SLEEP	0x22		//Sleep
#define	E0_WDT		0x23		//watchdog timeout

#define	E1_OPTZ		0x25		//door1 opened on timezone
#define	E1_CLTZ		0x26		//door2 closed on timezone
#define	E2_OPTZ		0x27		//door1 opened on timezone
#define	E2_CLTZ		0x28		//door2 closed on timezone
#define	E1_APINOK	0x29		//aux PIN 1 OK
#define	E2_APINOK	0x2A		//aux PIN 2 OK
#define	E1_APINNF	0x2B		//aux PIN 1 not found
#define	E2_APINNF	0x2C		//aux PIN 2 not found
#define	E0_INP		0x30		//Input status

#define	E1_LATO		0x3B		//Reader 1 latched open
#define	E1_LATC		0x3C		//Reader 1 latched closed
#define	E2_LATO		0x3D		//Reader 2 latched open
#define	E2_LATC		0x3E		//Reader 2 latched closed

#define	E1_ACC		0x41		//reader 1 accept
#define	E1_AL		0x42		//reader 1 access-level
#define	E1_TZ		0x43		//reader 1 timezone error
#define	E1_APB		0x44		//reader 1 APB error
#define	E1_REX		0x45		//reader 1 request to exit
#define	E1_DFORCE	0x46		//reader 1 door forced open
#define	E1_DNOPEN	0x47		//reader 1 door not opened
#define	E1_DLOPEN	0x48		//reader 1 door left open
#define	E1_DCLOSE	0x49		//reader 1 door closed
#define	E1_CARDNF	0x4A		//reader 1 card not found
#define	E1_WPIN		0x4B		//reader 1 wrong pin
#define	E1_WSITE	0x4C		//reader 1 wrong site
#define	E1_CFMT		0x4D		//reader 1 card format
#define	E1_PIN		0x4E		//reader 1 wait for pin
#define	E1_BUD		0x4F		//buddy mode

#define	E2_ACC		0x51		//reader 2 accept
#define	E2_AL		0x52		//reader 2 access-level
#define	E2_TZ		0x53		//reader 2 timezone
#define	E2_APB		0x54		//reader 2 APB error
#define	E2_REX		0x55		//reader 2 request to exit
#define	E2_DFORCE	0x56		//reader 2 door forced open
#define	E2_DNOPEN	0x57		//reader 2 door not opened
#define	E2_DLOPEN	0x58		//reader 2 door left open
#define	E2_DCLOSE	0x59		//reader 2 door closed
#define	E2_CARDNF	0x5A		//reader 2 card not found
#define	E2_WPIN		0x5B		//reader 2 wrong pin
#define	E2_WSITE	0x5C		//reader 2 wrong site
#define	E2_CFMT		0x5D		//reader 2 card format
#define	E2_PIN		0x5E		//reader 1 wait for pin
#define	E2_BUD		0x5F		//buddy mode

//EEPROM Memory Storage
//Card data checksum
#define	CRDSUML		20004			//low byte
#define	CRDSUMH		20005			//high byte

#define REL1_TM		CBASE + 0		//Relay1 time
#define	REL2_TM		CBASE + 1		//Relay2 time
#define	CTR_FLAGS	CBASE + 2		//Controller flags
#define	DOOR1_TM	CBASE + 3		//Door open time
#define	DOOR2_TM	CBASE + 4		//Door open time
#define	DOOR1_TZ	CBASE + 5		//door 1 open timezone
#define	DOOR2_TZ	CBASE + 6		//door 2 open timezone
#define	PIN1TZH		CBASE + 7		//PIN 1 timezone high
#define	PIN1TZL		CBASE + 8		//PIN 1 timezone low
#define	PIN2TZH		CBASE + 9		//PIN 2 timezone high
#define	PIN2TZL		CBASE + 10		//PIN 2 timezone low

#define	DMODE1		CBASE + 11		//door 1 mode
#define	DMODE2		CBASE + 12		//door 2 mode
#define MTEST		CBASE + 13		//check for memory ready

#define	CRD_FMT		CBASE + 14		//Card format (0 = mag, 1 = wieg)
#define C_LENGTH	CBASE + 15		//Wiegand bits
#define	W_EVEN		CBASE + 16		//Wiegand even parity
#define	W_ODD		CBASE + 17		//Wiegand odd parity
#define	C_SITE_B	CBASE + 18		//Wiegand site # of bits
#define	C_SITE_L	CBASE + 19		//Wiegand site location
#define	C_CARD_B	CBASE + 20		//Wiegand card # of bits
#define	C_CARD_L	CBASE + 21		//Wiegand card location
#define	C_SITE_1	CBASE + 22		//Wiegand site 0
#define	C_SITE_0	CBASE + 23		//Wiegand site 1
//(0=65536 cards, 1=20000+PIN, 2=9,000random+PIN)
#define	CARD_DB		CBASE + 24		//card database 
#define	PIN_LEN		CBASE + 25		//PIN length (default 4)

#define	APBMODE		CBASE + 26		//anti-passback mode
#define	APB_RST0	CBASE + 27		//Hour for APB reset
#define	APB_RST1	CBASE + 28		// "
#define	APB_RST2	CBASE + 29		// "
#define	APB_RST3	CBASE + 30		// "
#define	ATBTM1		CBASE + 31		//rdr1 atb minutes
#define	ATBTM2		CBASE + 32		//rdr2 atb minutes

#define	NODE_ADD	CBASE + 33		//Node address
#define	OP_MODE		CBASE + 34		//Mode of operation  
#define	PASSL		CBASE + 35		//password low byte
#define	PASSH		CBASE + 36		//password high byte
#define	USE_PASS	CBASE + 37		//use password
#define EVTS_ON		CBASE + 38		//0 = events off
#define	CRD_OFF1	CBASE + 39		//card offset high
#define	CRD_OFF0	CBASE + 40		//card offset high
#define	ALTIME		CBASE + 41		//local alarm time-out
#define	ALTRIGS		CBASE + 42		//local alarm triggers

//PIN numbers
 #define	PIN1_L		CBASE + 43		//pin 1 low byte
 #define	PIN1_H		CBASE + 44		//pin 1 high byte
 #define	PIN2_L		CBASE + 45		//pin 2 low byte
 #define	PIN2_H		CBASE + 46		//pin 2 high byte
 #define	PIN3_L		CBASE + 47		//pin 3 low byte
 #define	PIN3_H		CBASE + 48		//pin 3 high byte
 #define	PIN4_L		CBASE + 49		//pin 4 low byte
 #define	PIN4_H		CBASE + 50		//pin 4 high byte
 #define	PIN5_L		CBASE + 51		//pin 5 low byte
 #define	PIN5_H		CBASE + 52		//pin 5 high byte
 #define	PIN6_L		CBASE + 53		//pin 6 low byte
 #define	PIN6_H		CBASE + 54		//pin 6 high byte
 #define	PIN7_L		CBASE + 55		//pin 7 low byte
 #define	PIN7_H		CBASE + 56		//pin 7 high byte
 #define	PIN8_L		CBASE + 57		//pin 8 low byte
 #define	PIN8_H		CBASE + 58		//pin 8 high byte
 #define	PIN9_L		CBASE + 59		//pin 9 low byte
 #define	PIN9_H		CBASE + 60		//pin 9 high byte
 #define	PIN10_L		CBASE + 61		//pin 10 low byte
 #define	PIN10_H		CBASE + 62		//pin 10 high byte  
//PIN Acess Levels
 #define	PINAL1		CBASE + 63
 #define	PINAL2		CBASE + 64
 #define	PINAL3		CBASE + 65
 #define	PINAL4		CBASE + 66
 #define	PINAL5		CBASE + 67
 #define	PINAL6		CBASE + 68
 #define	PINAL7		CBASE + 69
 #define	PINAL8		CBASE + 70
 #define	PINAL9		CBASE + 71
 #define	PINAL10		CBASE + 72
	
//Timezone 2
#define	TZ21FH		CBASE + 70		//timezone 2 from hour
#define	TZ21FM		CBASE + 71		//timezone 2 from minute
#define	TZ21TH		CBASE + 72		//timezone 2 to hour
#define	TZ21TM		CBASE + 73		//timezone 2 to minute
#define	TZ21DAY		CBASE + 74		//timezone 2 days
#define	TZ22FH		CBASE + 75		//timezone 2 from hour
#define	TZ22FM		CBASE + 76		//timezone 2 from minute
#define	TZ22TH		CBASE + 77		//timezone 2 to hour
#define	TZ22TM		CBASE + 78		//timezone 2 to minute
#define	TZ22DAY		CBASE + 79		//timezone 2 days
//Timezone 3
#define	TZ31FH		CBASE + 80		//timezone 3 from hour
#define	TZ31FM		CBASE + 81		//timezone 3 from minute
#define	TZ31TH		CBASE + 82		//timezone 3 to hour
#define	TZ31TM		CBASE + 83		//timezone 3 to minute
#define	TZ31DAY		CBASE + 84		//timezone 3 days
#define	TZ32FH		CBASE + 85		//timezone 3 from hour
#define	TZ32FM		CBASE + 86		//timezone 3 from minute
#define	TZ32TH		CBASE + 87		//timezone 3 to hour
#define	TZ32TM		CBASE + 88		//timezone 3 to minute
#define	TZ32DAY		CBASE + 89		//timezone 3 days
//Timezones
#define	TZ2			CBASE + 100		//timezone 2
#define	TZ3			CBASE + 105		//timezone 3
#define	TZ4			CBASE + 110		//timezone 4
#define	TZ5			CBASE + 115		//timezone 5
#define	TZ6			CBASE + 120		//timezone 6
#define	TZ7			CBASE + 125		//timezone 7
#define	TZ8			CBASE + 130		//timezone 8
#define	TZ9			CBASE + 135		//timezone 9
#define	TZ10		CBASE + 140		//timezone 10
#define	TZ11		CBASE + 145		//timezone 11
#define	TZ12		CBASE + 150		//timezone 12
#define	TZ13		CBASE + 155		//timezone 13
#define	TZ14		CBASE + 160		//timezone 14
#define	TZ15		CBASE + 165		//timezone 15
#define	TZ16		CBASE + 170		//timezone 16
#define TZSIZE		5				//size of each timezone
#define TZQTY		15				//number of timezones
//Access Levels
#define	ACC0		CBASE + 180		//Accesslevel 0
#define	ACC1		CBASE + 184		//Accesslevel 1
#define	ACC2		CBASE + 188		//Accesslevel 2
#define	ACC3		CBASE + 192		//Accesslevel 3
#define	ACC4		CBASE + 196		//Accesslevel 4
#define	ACC5		CBASE + 200		//Accesslevel 5
#define	ACC6		CBASE + 204		//Accesslevel 6
#define	ACC7		CBASE + 208		//Accesslevel 7
#define	ACC8		CBASE + 212		//Accesslevel 8
#define	ACC9		CBASE + 216		//Accesslevel 9
#define	ACC10		CBASE + 220		//Accesslevel 10
#define	ACC11		CBASE + 224		//Accesslevel 11
#define	ACC12		CBASE + 228		//Accesslevel 12
#define	ACC13		CBASE + 232		//Accesslevel 13
#define	ACC14		CBASE + 236		//Accesslevel 14
#define	ACC15		CBASE + 240		//Accesslevel 15
#define ACCSIZE		4				//size of each accesslevel
#define	ACCQTY		16				//number of accesslevels
//Event storage variables
#define	evtputl		CBASE + 260		//points to next storage location
#define	evtputh		CBASE + 261		//points to next storage location
#define	evtgetl		CBASE + 262		//points to next event to retrieve
#define	evtgeth		CBASE + 263		//points to next event to retrieve
//Configuration checksum
#define	CFGSUML		CBASE +	265		//low byte
#define	CFGSUMH		CBASE +	266		//high byte
//Anti-timeback storage
#define ATBSIZE		10				//size of atb buffer

#define	ATB1PUT		CBASE + 308		//next atb location
#define	ATB2PUT		CBASE + 309		//next atb location
#define	ATB10		CBASE + 310
#define	ATB11		CBASE + 315
#define	ATB12		CBASE + 320
#define	ATB13		CBASE + 325
#define	ATB14		CBASE + 330
#define	ATB15		CBASE + 335
#define	ATB16		CBASE + 340
#define	ATB17		CBASE + 345
#define	ATB18		CBASE + 350
#define	ATB19		CBASE + 355
#define	ATB20		CBASE + 360
#define	ATB21		CBASE + 365
#define	ATB22		CBASE + 370
#define	ATB23		CBASE + 375
#define	ATB24		CBASE + 380
#define	ATB25		CBASE + 385
#define	ATB26		CBASE + 390
#define	ATB27		CBASE + 395
#define	ATB28		CBASE + 400
#define	ATB29		CBASE + 405
//Door open timezones 
#define	DTZ1		CBASE + 410		//timezone 1 
#define	DTZ2		CBASE + 415		//timezone 2 
#define	DTZ3		CBASE + 420		//timezone 3 
#define	DTZ4		CBASE + 425		//timezone 4 
#define	DTZ5		CBASE + 430		//timezone 5 
#define	DTZ6		CBASE + 435		//timezone 6 
#define	DTZ7		CBASE + 440		//timezone 7 
#define	DTZ8		CBASE + 445		//timezone 8 
#define	DTZ9		CBASE + 450		//timezone 9 
#define	DTZ10		CBASE + 455		//timezone 10 
#define	DTZ11		CBASE + 460		//timezone 11 
#define	DTZ12		CBASE + 465		//timezone 12 
#define	DTZ13		CBASE + 470		//timezone 13 
#define	DTZ14		CBASE + 475		//timezone 14 
#define	DTZ15		CBASE + 480		//timezone 15 
#define	DTZ16		CBASE + 485		//timezone 16 
#define DTZSIZE		5				//size of each timezone
#define DTZQTY		16				//number of timezones
//Door Access Levels
#define	DACC0		CBASE + 490		//Accesslevel 0
#define	DACC1		CBASE + 492		//Accesslevel 1
#define DACCSIZE	2				//size of each accesslevel
#define	DACCQTY		2				//number of accesslevels

//IO settings
#define INP1		CBASE + 320		//input 1 settings
#define INP2		CBASE + 328		//input 2 settings
#define INP3		CBASE + 330		//input 3 settings
#define INP4		CBASE + 338		//input 4 settings
#define INP5		CBASE + 340		//input 5 settings
#define INP6		CBASE + 348		//input 6 settings
#define INP7		CBASE + 350		//input 7 settings
#define INP8		CBASE + 358		//input 8 settings
//Holidays
#define	HOLS		CBASE + 420 

//Relay activation counters
#define R1ACT		CBASE + 494		//relay 1 activation count (max 10^6)
#define R2ACT		CBASE + 497		//relay 2 activation count (max 10^6)

#define FIRSTUSE	CBASE + 500		//flag to determine initial use

//Accommodation cards
#define ACCOMCARDS		CBASE + 501		//20 cards @ 4 bytes each. 1st 10 = door 1; 2nd 10 = door 2

enum INP_TYPES {INP_NO, INP_TAMP, INP_ARM, INP_SW, INP_PULSE, INP_REP};
 
//Door monitoring flags
#define	DOOR_N		0x00		//no door monitoring
#define	DOOR_1		0x01		//door 1 only
#define	DOOR_2		0x02		//door 2 only
#define	DOOR_B		0x03		//both doors
#define	DOOR_S		0x04		//single door in/out
//Defines for FM31256 RTC
#define	RTCCON		0x00		//rtc control
#define	RTCCAL		0x01		//rtc calibration
#define	RTCSEC		0x02		//rtc seconds
#define	RTCMIN		0x03		//rtc minutes
#define	RTCHOUR		0x04		//rtc hours
#define	RTCDAY		0x05		//rtc day of week
#define RTCDATE		0x06		//rtc date
#define	RTCMON		0x07		//rtc month
#define	RTCYEAR		0x08		//rtc year
#define	RTCWDT1		0x09		//rtc watchdog 1
#define	RTCWDT2		0x0A		//rtc watchdog 2
#define	RTCCOMP		0x0B		//rtc companion
//Card info storage bits (1 byte per card)
#define	CRD_APB0	0x00		//reader 1 apb
#define	CRD_PASS	0x01		//card passback
#define	CRD_CAP0	0x02		//reader 1 capture
#define	CRD_CAP1	0x03		//reader 2 capture
//Latch door from 3x card present
#define	SECONDREAD1	0x01		//card previously read once reader 1
#define THIRDREAD1	0x02		//card previously read twice reader 1
#define	SECONDREAD2	0x01		//card previously read once reader 2
#define THIRDREAD2	0x02		//card previously read twice reader 2

union join16
{
	char b[2];
	long w;
};

union join32
{
	int8 b[4];
	int32 w;
};

char tick, rd1itick, rd2itick, r1tick, r2tick, crcflags;
char rd1idat[32], rd1ibits, *rd1ip, rd1istat, rd1iax;
char rd2idat[32], rd2ibits, *rd2ip, rd2istat, rd2iax;
 char pin1dat[6], pin1cnt, pin1stat, *pin1p, pin1tick, pinlen, pinlist; 
char pin2dat[6], pin2cnt, pin2stat, *pin2p, pin2tick;
char rxdat[1300], rxstat, rxsize, rxtick, rxcnt;
char rex1cnt, rex2cnt, door1cnt, door2cnt, node, drmon1, drmon2;
short tout, t2tout, rex1on, rex2on, evtson, clrcrd;
short r1tout, r2tout, dr1tout, dr2tout, alarmtout, arm1, arm2;
static char cardfmt, pollnext, pollseq, onesec, tmr2tick, cap1tick, cap2tick;
char dr1stat, dr2stat, mode1, mode2, dr1tick, dr2tick;
char alarmtick;
char bf[128], in1cnt, in2cnt, in3cnt, in4cnt, in5cnt, in6cnt, in7cnt, in8cnt, instat;
char tbuf[256], evtsent;
char buff[MAGDIGS + 1];    
char apb, curhour, newhour;
long drtc;
int32 lastread1, lastread2, r1count, r2count;
int readcount1, readcount2, readtimer1, readtimer2;
char latchrly1, latchrly2, latchtime;
char buddytime;

struct evtst
{
	char year, mon, date, day, hour, min, sec;
	union join32 card;
	char evt;
} evt;  

struct atbstruct
{
	long card;
	char date;
	long min;
} atbst1[10], atbst2[10], *atbptr;

char atbput1, atbput2;
struct crdstruct
{
	long index;
	long addr;
	char data;
	union join32 pin;
} crdst;

struct lastcard
{
	int32 card;
	int32 sec;
} lastcard1, lastcard2;

char const DEFAULT_SET[] = {1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 10, 0, 0, 4, 2, 4, 6, 0, 0, 0,
							0, 0, 0};

char wiegcard(char rdr);
char magcard(char rdr);
short magcard_par(char cx);
char check_card(char rdr);
short check_pin(char *ps, char rdr); 
long dig2b(char *s);
void reset_apb(void);
void store_event(void);
void print_events(void);
void read_rtc(void);
short check_timezone(char tz);
void sendack(nd);
void clear_events(short cf);
short tz_check(long addr);
short txpoll(void);
short txpoll_data(void);
void relay_on(char rly, char tm);
void powerup_check(void);
short update_doortz(char rdr);
void clear_cards(void);
void zeroevent(char ev);
void send_mod(char rpl);
long mem_check(void);
void init_rtc(void);
void set_rtc(void);
char bcd2hex(char b);
short check_atb(char rdr);
short check_access(char rdr, char acc);
void initcrc(void);
void reset_atb(void);
char read_rtc_val(char addr);
void setup_readers(void);
void strcpyn(char *s1, char *s2, char n);

char findcard(char sz);
short findspace(void);
short writecard(void);
short writeblock(char *s);
void armcheck(short i);
void tpcheck(short i);
void iocheck(char i);
void ioinit(void);
short card_block(void);
void set_defaults(short cf);
long dayofyear(void);
char chkpin(char *pdat);
char check_apb(char rdr);
char buddycheck(char rdr);
void loadfw(void);
void ToggleDoor(unsigned char dr);
char LEDflash, LEDcnt, LEDcntloop, LEDstop;
float LEDcntfp;
void rcount(void);
char dr1atg, dr2atg;
void rcountsave(char i);

//---------------------------------------------------------------------
//Timer 1 interrupt routine (52mS)
//---------------------------------------------------------------------
#int_timer1
void clock_isr(void)
{
char x;
	if (!--tick)				//250mS timer
	{
		tout = 1;
		tick = 5;
	}
	if (rd1istat == CARD_PRO) 
	{
		if (!--rd1itick)
		{
			x = rd1ibits;
			while (x & 0x07)
			{
				rd1iax <<= 1;
				++x;
			}
			*rd1ip = rd1iax;
			if (rd1ibits > 8) rd1istat = CARD_RDY; else rd1istat = CARD_PIN;
		}
	}
	if (rd2istat == CARD_PRO) 
	{
		if (!--rd2itick)
		{
			x = rd2ibits;
			while (x & 0x07)
			{
				rd2iax <<= 1;
				++x;
			}
			*rd2ip = rd2iax;
			if (rd2ibits > 8) rd2istat = CARD_RDY; else rd2istat = CARD_PIN;
		}
	}
 	if (pin1stat == PIN_PRO)  
	{ 
 		if (!--pin1tick) pin1stat = PIN_NO;	//PIN 1 time-out 
	} 
 	if (pin2stat == PIN_PRO)  
 	{ 
 		if (!--pin2tick) pin2stat = PIN_NO;	//PIN 2 time-out 
 	} 


	readtimer1++;						//Timers for 3x swipe = latch lock
	readtimer2++;

	if (LEDflash != 0)					//Flash LED to indicate toggledoor
	{
		LEDcntfp = LEDcnt;
		if ((LEDcntfp / 2) == (LEDcnt / 2))	//even cnt = LED off
		{
			if (LEDflash == 1) 
			{
				output_low(gled1);
	        	if (crcflags & 0x02) output_low(gled2);
			}
			else
			{
				output_low(gled2);
	        	if (crcflags & 0x02) output_low(gled1);
			}
		}
		else 								//odd cnt = LED on
		if (LEDflash == 1) 
		{
			output_high(gled1);
        	if (crcflags & 0x02) output_high(gled2);
		}
		else
		{
			output_high(gled2);
        	if (crcflags & 0x02) output_high(gled1);
		}
		LEDcntloop++;
		if (LEDcntloop == 3)				//3 runs = 156mS per flash
		{
			LEDcnt++;
			LEDcntloop = 0;
		}
		if (LEDcnt >= LEDstop)				//stop LED flash
		{
			LEDcnt = 0;
			LEDflash = 0;
			r1tout = true;					//prevent gled remaining lit for strike time
			r2tout = true;
		}
	}
}
//---------------------------------------------------------------------
//Timer 2 interrupt routine 2.4mS
//---------------------------------------------------------------------
#int_timer2
void timer2_isr(void)
{
	if (rxstat == RX_PRO) 
	{
		if (!--rxtick) rxstat = RX_NO;		//Serial rx timer
	}
	if (!--tmr2tick)						//500mS counter
	{
		tmr2tick = 208;
		if (!--r1tick) r1tout = true;		//relay 1 timer
		if (!--r2tick) r2tout = true;		//relay 2 timer
		if (!--cap1tick) output_low(rled1);	//capture off
		if (!--cap2tick) output_low(rled2);	//capture off
		if (!--dr1tick) dr1tout = true;		//door 1 timer 
		if (!--dr2tick) dr2tout = true;		//door 2 timer 
	}
	t2tout = true;
}
//---------------------------------------------------------------------
//Interrupt 0 routine (Portb0 rdr2 - mag=dat, wieg=D0)
//---------------------------------------------------------------------
#int_ext
void ext0_isr(void)
{
	if (rd2istat == CARD_NO)			//0 = no rdr2 data
	{
		rd2ip = rd2idat;
		rd2iax = 0x00;					//zero bit
		rd2ibits = 1;
		rd2istat = CARD_PRO;
		rd2itick = 2;					//reader 1 timer (300mS)
	}
	else 
	if (rd2istat == CARD_PRO)		//1 = rdr2 data in progress
	{
		if (rd2ibits < 80)
		{
			rd2iax <<= 1;
			bit_clear(rd2iax, 0);		//zero bit
			++rd2ibits;
			if (!(rd2ibits & 0x07)) *rd2ip++ = rd2iax;
			rd2itick = 2;
		}
	}
}
//---------------------------------------------------------------------
//Interrupt 1 routine (portb1 rdr2 - mag=clk, wieg=D1)
//---------------------------------------------------------------------
#int_ext1
void ext1_isr(void)
{
	if (cardfmt == FMT_MAG)					//mag card
	{
		if (rd2istat == CARD_NO)			//0 = rdr2 idle
		{
			rd2ip = rd2idat;
			if (!input(rdr2dat)) rd2iax = 0x01;	//one bit
			else rd2iax = 0x00;				//zero bit
			rd2ibits = 1;
			rd2itick = 2;					//reader 2 timer (300mS)
			rd2istat = CARD_PRO;			//card in progress
		}
		else 
		if (rd2istat == CARD_PRO)		//1 = rdr2 data in progress
		{
			if (rd2ibits < 248)
			{
				rd2iax <<= 1;
				if (!input(rdr2dat)) bit_set(rd2iax, 0);
				else bit_clear(rd2iax, 0);
				++rd2ibits;
				if (!(rd2ibits & 0x07)) *rd2ip++ = rd2iax;
				rd2itick = 2;
			}
		}
	}
	else								//wiegand card
	if (rd2istat == CARD_NO)			//0 = no rdr2 data
	{
		rd2ip = rd2idat;
		rd2iax = 0x01;					//one bit
		rd2ibits = 1;
		rd2istat = CARD_PRO;
		rd2itick = 2;					//reader 1 timer (300mS)
	}
	else 
	if (rd2istat == CARD_PRO)		//1 = rdr2 data in progress
	{
		if (rd2ibits < 80)
		{
			rd2iax <<= 1;
			bit_set(rd2iax, 0);			//one bit
			++rd2ibits;
			if (!(rd2ibits & 0x07)) *rd2ip++ = rd2iax;
			rd2itick = 2;
		}
	}
}
//---------------------------------------------------------------------
//Interrupt 2 routine (Portb2 rdr1 - mag=dat, wieg=D0)
//---------------------------------------------------------------------
#int_ext2
void ext2_isr(void)
{

	if (rd1istat == CARD_NO)			//0 = no rdr1 data
	{
		rd1ip = rd1idat;
		rd1iax = 0x00;					//zero bit
		rd1ibits = 1;
		rd1istat = CARD_PRO;
		rd1itick = 2;					//reader 1 timer (300mS)
	}
	else 
	if (rd1istat == CARD_PRO)		//1 = rdr1 data in progress
	{
		if (rd1ibits < 80)
		{
			rd1iax <<= 1;
			bit_clear(rd1iax, 0);		//zero bit
			++rd1ibits;
			if (!(rd1ibits & 0x07)) *rd1ip++ = rd1iax;
			rd1itick = 2;
		}
	}
}
//---------------------------------------------------------------------
//Interrupt 3 routine (Portb3 rdr1 - mag=clk, wieg=D1) 
//---------------------------------------------------------------------
#int_ext3
void ext3_isr(void)
{

	if (cardfmt == FMT_MAG)					//mag card
	{
		if (rd1istat == CARD_NO)			//0 = rdr1 idle
		{
			rd1ip = rd1idat;
			if (!input(rdr1dat)) rd1iax = 0x01;	//one bit
			else rd1iax = 0x00;				//zero bit
			rd1ibits = 1;
			rd1itick = 2;					//reader 1 timer (300mS)
			rd1istat = CARD_PRO;			//card in progress
		}
		else 
		if (rd1istat == CARD_PRO)		//1 = rdr1 data in progress
		{
			if (rd1ibits < 248)
			{
				rd1iax <<= 1;
				if (!input(rdr1dat)) bit_set(rd1iax, 0);
				else bit_clear(rd1iax, 0);
				++rd1ibits;
				if (!(rd1ibits & 0x07))	*rd1ip++ = rd1iax;
				rd1itick = 2;   
			}
		}
	}
	else								//wiegand card
	if (rd1istat == CARD_NO)			//0 = no rdr1 data
	{
		rd1ip = rd1idat;
		rd1iax = 0x01;					//one bit
		rd1ibits = 1;
		rd1istat = CARD_PRO;
		rd1itick = 2;					//reader 1 timer (300mS)
	}
	else 
	if (rd1istat == CARD_PRO)			//1 = rdr1 data in progress
	{
		if (rd1ibits < 80)
		{
			rd1iax <<= 1;
			bit_set(rd1iax, 0);			//one bit
			++rd1ibits;
			if (!( rd1ibits & 0x07)) *rd1ip++ = rd1iax;
			rd1itick = 2;
		}
	}
}
//---------------------------------------------------------------------------
//Serial interrupt for RS485 comms
//rxstat is message status. RX_NO = no msg, RX_PRO = msg in progress
//RX_POLL = poll msg, RX_SATA = command msg
//---------------------------------------------------------------------
#int_rda
void serial_isr(void) 
{
	char ch;
	ch = getc();

	if (rxstat == RX_NO)						//if no message in progress
	{
		rxdat[0] = ch;
		if (!bit_test(ch, 7)) rxstat = RX_POLL;	//poll ready
		else 
		{
			rxcnt = 1;
			rxtick = 10;						//serial rx time-out
			rxstat = RX_PRO;					//message in progress
		}
	}
	else 
	if (rxstat == RX_PRO)						//if message in progress
	{
		if (rxcnt == 1)
		{
			rxsize = ch;
		}
		if (rxcnt < 250) rxdat[rxcnt] = ch;
		if (rxcnt > rxsize)	
		rxstat = RX_DATA;
		++rxcnt;
		rxtick = 10;							//serial rx time-out
	}
}
//---------------------------------------------------------------------------
//Main program
//---------------------------------------------------------------------
void main(void)
{
char c, x, n;
int32 rcx;
static char oldmin, oldio;

#use fast_io(A)
#use fast_io(B)
#use standard_io(C)
//#use fast_io(D)
#use standard_io(D)
#use standard_io(E)
#use fast_io(F)
#use fast_io(G)

	setup_adc(ADC_OFF);
    setup_adc_ports(NO_ANALOGS);				//port a = digital
	CCP1CON = 0;
	CCP2CON = 0;
	CCP3CON = 0;
	CCP4CON = 0;
	CCP5CON = 0;

	porta = 0x00;
    set_tris_a(0xcf);
    portb = 0x00;
    set_tris_b(0x3f);
	portd = 0x00;
    setup_psp(PSP_DISABLED);
    porte = 0x00;
    set_tris_e(0x03);							//porte
	set_tris_f(0xff);							//portf = all inputs
	set_tris_g(0x1f);
    setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);	//52mS interval
	setup_timer_2(T2_DIV_BY_16, 250, 6);
    enable_interrupts(INT_TIMER1);				//Timer1 int enable
	enable_interrupts(INT_TIMER2);				//timer2 int enable

	LDVCON = 0x8b;								//low voltage detect

	output_low(txon);							//RS485 = RX
  	enable_interrupts(int_rda);					//serial int enable
	output_low(auxo1);							//local alarm off
	output_low(auxo2);							//door2 alarm off
    rd1istat = CARD_NO;							//clear reader 1 status
    rd2istat = CARD_NO;							//clear reader 2 status
	pin1stat = PIN_NO;							//clear pin1 status
	pin2stat = PIN_NO;							//clear pin2 status
//setup rex regs
	rex1on = false;								//no rex
	rex1cnt = DBOUNCE;
	rex2on = false;
	rex2cnt = DBOUNCE;                        
	door1cnt = DBOUNCE;
	door2cnt = DBOUNCE;
	mode1 = MD_CARD;							//card only
	mode2 = MD_CARD;
	dr1stat = DOOR_NO;							//no door activity
	dr2stat = DOOR_NO;

	rxstat = RX_NO;
    init_ext_eeprom();							//initialize eeprom
	do
	{
		x = read_ext_eeprom(MEM, MTEST);
		if (x != 0x55) write_ext_eeprom(MEM, MTEST, 0x55);
		restart_wdt();
	}
	while (x != 0x55);
	output_high(led);

	init_spi_eeprom();
	init_rtc();     							//initialize real time clock
	initcrc();									//initialize controller
	setup_readers();
	onesec = 0;
	output_low(txon);							//RS485 = RX
	newhour = 99;								//force a apb reset

//If first ever use set to known state
	if (read_ext_eeprom(MEM, FIRSTUSE) != 0x101010)
	{
		set_defaults(1);
		rcountsave(0);							//ensure relay counts are 0
		write_ext_eeprom(MEM, FIRSTUSE, 0x101010);
	}	
 
//read relay activation counters
	r1count = 0;
	for (n = 0; n < 3; n++)
	{
		rcx = read_ext_eeprom(MEM, (R1ACT + n));	
		r1count = r1count + (rcx <<= (n*8));
	}
	r2count = 0;
	for (n = 0; n < 3; n++)
	{
		rcx = read_ext_eeprom(MEM, (R2ACT + n));	
		r2count = r2count + (rcx <<= (n*8));
	}

	if (!input(PIN_F5))							//DIP SW6 - Perform Reset to defaults
	{
		output_high(led); 
		clear_events(0);
		clear_cards();
		set_defaults(1);
		output_low(led);
	}
	output_low(led); 
	delay_ms(100);
	output_high(led);
	delay_ms(100);
	output_low(led);
	crcflags = read_ext_eeprom(MEM, CTR_FLAGS);	//controller flags
#if defined(ONEDOOR)
	crcflags |= 0x02;
#endif
	lastcard1.card = 0x00000000;				//last card used
	lastcard2.card = 0x00000000;
//Setup variables for 3x swipe = latch
	lastread1 = 0x00000000;
	lastread2 = 0x00000000;
	readcount1 = 0x00;
	readcount2 = 0x00;
	readtimer1 = 0;
	readtimer2 = 0;
	latchrly1 = 0;
	latchrly2 = 0;
	latchtime = 25;

	LEDflash = 0;

	buddytime = 10;

	ioinit();									//initialize io

while (true)
    {
		restart_wdt();
//check for illegal pin1stat
		if (pin1stat > PIN_RDY) pin1stat = PIN_NO;
//check for illegal pin2stat
		if (pin2stat > PIN_RDY) pin2stat = PIN_NO;
//check door modes
		if (LEDflash == 0)		//only if not performing a toggledoor LED flash
		{
			if ((dr1stat == DOOR_TZOP) || (dr1stat == DOOR_SOP))
			{									//if open on timezone or set open
				output_high(relay1);
				output_high(gled1);
				if (crcflags & 0x02) output_high(gled2);
			}
			if ((dr2stat == DOOR_TZOP) || (dr2stat == DOOR_SOP))
			{									//if open on timezone or set open
				output_high(relay2);
				output_high(gled2);
			}
		}
//check reader1 in
		if (rd1istat == CARD_RDY)				//reader1 in
		{
			if (!input(auxi2)) rd1istat = CARD_NO;	//not armed, reset rd1istat or card read while not armed gets processed once armed
			else								// is armed
			{
				if (cardfmt == FMT_MAG) c = magcard(1);
				else c = wiegcard(1);
				if (c == RD_OK)
				{
					evt.evt = check_card(1);
					if (evt.evt == E1_PIN)
					{
						if (pin1stat == PIN_RDY)
						{
							if (chkpin(pin1dat)) evt.evt = E1_ACC;
							else 
							{
								evt.evt = E1_WPIN;					//wrong pin
								bit_clear(crdst.data, CRD_CAP0);	//do not capture
							}
						}
						else 
						{
							evt.evt = E1_WPIN;						//wrong pin
							bit_clear(crdst.data, CRD_CAP0);		//do not capture
						}
					}
					if (evt.evt == E1_ACC)							//if card ok
					{
						if (check_atb(1))							//check atb first
						{
							if (check_apb(1))						//anti-passback?
							{
								crcflags = read_ext_eeprom(MEM, CTR_FLAGS);
#if defined(ONEDOOR)
								crcflags |= 0x02;
#endif
								if (crcflags & 0x04)				//buddy mode?
								{
									if (!buddycheck(1)) evt.evt = E1_BUD;
								}
								if (evt.evt == E1_ACC)				//card accepted
								{
									latchrly1 = 0x00;
									if ((readtimer1 < latchtime) && (evt.card.w == lastread1)) //re-present of a card within time
									{
										evt.evt = 0x00;				//no need for multiple 'Granted'
										switch (readcount1)
										{
											case SECONDREAD1:
												lastread1 = evt.card.w;
												readcount1++;
												break;
											case THIRDREAD1:
												lastread1 = 0x00000000;
												latchrly1 = 0x01;
												readcount1 = 0x00;
												break;
										}
									}
									else 							 //different to previous read
									{
										readcount1 = 0x01;
										lastread1 = evt.card.w;
									}
									readtimer1 = 0;					//reset counter
									relay_on(1, 0);
									if (drmon1 != 0)
									{
										if (!input(door1)) output_low(auxo1);				//if door closed turn local alarm off
										else if ((crcflags & 0x08) == 0x00) evt.evt = 0x00;	//door is open - if not muster controller do not report
									}
									if ((dr1stat == DOOR_SOP) && ((crcflags & 0x08) == 0x00)) evt.evt = 0x00; //copes with no door monitoring and toggledoor
									if ((crdst.data & 0x01) && (latchrly1 == 0x01))	ToggleDoor(1);										
								}
							}
							else 
							{
								evt.evt = E1_APB;					//apb error
								bit_clear(crdst.data, CRD_CAP0);	//not capture
							}
						}
						else 
						{
							evt.evt = E1_APB;						//apb error
							bit_clear(crdst.data, CRD_CAP0);		//not capture
						}
					}
					if (bit_test(crdst.data, CRD_CAP0))				//capture?
       				{
						cap1tick = 2;								//1 second
           				output_high(rled1);							//capture card
           			}
					if (evt.evt != 0x00) store_event();
				}
				else 
				if (c == RD_SITE)
				{
					evt.evt = E1_WSITE;								//wrong sitecode
					store_event();
				}
				else 
				if (c == RD_FMT)
				{ 
					evt.evt = E1_CFMT;								//card format error
					store_event();
       			}
				pin1stat = PIN_NO;
        		rd1istat = CARD_NO;
			}
		}
 		else 
		if (rd1istat == CARD_PIN)									//pin data 
 		{ 
			if ((pin1stat == PIN_RDY) && (pin1cnt >= 0x04))	pin1stat = PIN_PRO; //key pressed during wait for card - cancel wait
			if (pin1stat == PIN_NO) 
 			{ 
 				pin1tick = 100;										//5 secs time-out to enter whole PIN
 				pin1p = pin1dat;									//point to pin data 
 				pin1stat = PIN_PRO;									//pin in progress 
				pin1cnt = 0; 
 				pinlen = read_ext_eeprom(MEM, PIN_LEN); 
 			} 
 			if (pin1stat == PIN_PRO) 
 			{ 
 				x = rd1idat[0]; 
 				if (rd1ibits == 4) x >>= 4; 
 				if (x == 0x23) x = 0xb;		//RH320 # 
 				x &= 0x0F; 
 				x |= 0x30; 
 				if (isdigit(x)) *pin1p++ = x; 
 				if (x == 0x3a) pin1stat = PIN_NO;
				else 
				if (++pin1cnt == 0x04)
				{
					if (pinlen == 0x04) 							//if system PIN len = 4 act on it now
					{
						pin1tick = 60;								//3 secs to present card
						pin1stat = PIN_RDY;
					}
				}
				else 
				if (pin1cnt == 0x05)								//check for 4 digits plus #
				{
					if (x == 0x3b)									//# is last key  pressed - act on the 4 digit pin
					{									
						pin1dat[4] = 0x00;							//clear out digits 5 & 6 as they cause
						pin1dat[5] = 0x00;							//incorrect calc if previous PIN was 6 digits
						pinlist = check_pin(pin1dat,1);			
						if (pinlist == 0x00)						//PIN is not in the list of 10
						{
							evt.evt = E1_WPIN;						//report wrong PIN
							pin1stat = PIN_NO;
						}
						else 
						if (pinlist == 0x01)						//PIN is in list of 10 and acc lev valid
						{
							relay_on(1, 0);							//relay 1 on
							dr1stat = DOOR_REL;						//door released
							evt.evt = E1_APINOK;					//report PIN ok
							pin1stat = PIN_NO;
						}
						else 
						if (pinlist == 0x02)						//PIN is in list but acc lev not currently valid
						{
							evt.evt = E1_APINNF;						//report PIN currently invalid
							pin1stat = PIN_NO;
						}
						store_event();
					}
				}
				else 
				if ((pin1cnt == 0x06) && (pinlen == 0x06))
				{
					if (check_pin(pin1dat,1) == 0x00)
					{
						pin1tick = 60;								//3 secs to present card
						pin1stat = PIN_RDY;
					}
				}
			} 
			rd1istat = CARD_NO;
		} 
		else 
		if (rd1istat > CARD_PIN) rd1istat = CARD_NO;
		//check reader2 in
		if (rd2istat == CARD_RDY)					//reader2 in
		{
			if (!input(auxi3)) rd2istat = CARD_NO;	//not armed, reset rd2istat or card read while not armed gets processed once armed
			else									//is armed
			{
				crcflags = read_ext_eeprom(MEM, CTR_FLAGS);
#if defined(ONEDOOR)
				crcflags |= 0x02;
#endif
				if (cardfmt == FMT_MAG) c = magcard(2);
				else c = wiegcard(2);
				if (c == RD_OK)
				{
					evt.evt = check_card(2);
					if (evt.evt == E2_PIN)
					{
						if (pin2stat == PIN_RDY)
						{
							if (chkpin(pin2dat)) evt.evt = E2_ACC;
							else 
							{
								evt.evt = E2_WPIN;					//wrong pin
								bit_clear(crdst.data, CRD_CAP1);	//do not capture
							}
						}
						else 
						{
							evt.evt = E2_WPIN;						//wrong pin
							bit_clear(crdst.data, CRD_CAP1);		//do not capture
						}
					}
					if (evt.evt == E2_ACC)							//if card ok
					{
						if (check_atb(2))							//check atb first
						{
							if (check_apb(2))						//anti-passback?
							{	
								if (crcflags & 0x02)				//if single door
								{
									if (crcflags & 0x04)			//buddy mode?
									{
										if (!buddycheck(2)) evt.evt = E2_BUD;
									}
									if (evt.evt == E2_ACC)
									{
										latchrly1 = 0x00;
										if (readtimer2 < latchtime && evt.card.w == lastread2) //re-present of a card within time
										{
											evt.evt = 0x00;			//no need for multiple 'Granted'
											switch (readcount2)
											{
												case SECONDREAD2:
													lastread2 = evt.card.w;
													readcount2++;
													break;
												case THIRDREAD2:
													lastread2 = 0x00000000;
													readcount2 = 0x00;
													latchrly1 = 0x01;
													break;
											}
										}
										else  						//different to previous read
										{
											readcount2 = 0x01;
											lastread2 = evt.card.w;
										}
										readtimer2 = 0;				//reset counter
										relay_on(1, 0);
										if (drmon1 != 0)
										{
											if (!input(door1)) output_low(auxo1);				//if door closed turn local alarm off
											else if ((crcflags & 0x08) == 0x00) evt.evt = 0x00;	//door is open - if not muster controller do not report
										}
										if ((dr1stat == DOOR_SOP) && ((crcflags & 0x08) == 0x00)) evt.evt = 0x00; //copes with no door monitoring and toggledoor
										if ((crdst.data & 0x01) && (latchrly1 == 0x01))	ToggleDoor(1);
									}		
								}
								else								//2 door
								{
									if (crcflags & 0x04)				//buddy mode?
									{
										if (!buddycheck(2)) evt.evt = E2_BUD;
									}
									if (evt.evt == E2_ACC)
									{
										latchrly2 = 0x00;
										if (readtimer2 < latchtime && evt.card.w == lastread2) //re-present of a card within time
										{
											evt.evt = 0x00;				//no need for multiple 'Granted'
											switch (readcount2)
											{
												case SECONDREAD2:
													lastread2 = evt.card.w;
													readcount2++;
													break;
												case THIRDREAD2:
													lastread2 = 0x00000000;
													readcount2 = 0x00;
													latchrly2 = 0x01;
													break;
											}
										}
										else  							//different to previous read
										{
											readcount2 = 0x01;
											lastread2 = evt.card.w;
										}
										readtimer2 = 0;	//reset counter
										relay_on(2, 0);
										if (drmon2 !=0)
										{
											if (!input(door2)) output_low(auxo2);				//if door closed turn local alarm off
											else if ((crcflags & 0x08) == 0x00) evt.evt = 0x00;	//door is open - if not muster controller do not report
										}
										if ((dr2stat == DOOR_SOP) && ((crcflags & 0x08) == 0x00)) evt.evt = 0x00; //copes with no door monitoring and toggledoor
										if ((crdst.data & 0x01) && (latchrly2 == 0x01))	ToggleDoor(2);
									}
								}
							}
							else 
							{
								evt.evt = E2_APB;					//apb error
								bit_clear(crdst.data, CRD_CAP1);	//not capture
							}
						}
						else 
						{
							evt.evt = E2_APB;						//apb error
							bit_clear(crdst.data, CRD_CAP1);		//not capture
						}
					}
					if (bit_test(crdst.data, CRD_CAP1))				//capture?
        			{
						cap2tick = 2;								//1 second
            			output_high(rled2);							//capture card
            		}
					if (evt.evt != 0x00) store_event();
				}
				else 
				if (c == RD_SITE)
				{
					evt.evt = E2_WSITE;
					store_event();
				}
				else 
				if (c == RD_FMT)
				{ 
					evt.evt = E2_CFMT;
					store_event();
    	    	}
			}
			pin2stat = PIN_NO;
        	rd2istat = CARD_NO;
		}
		else 
		if (rd2istat == CARD_PIN)						//pin data
		{
			if ((pin2stat == PIN_RDY) && (pin2cnt >= 0x04))	pin2stat = PIN_PRO; //key pressed during wait for card
			if (pin2stat == PIN_NO)
			{
				pin2tick = 100;							//5 secs time-out to enter whole PIN				
				pin2p = pin2dat;						//point to pin data
				pin2stat = PIN_PRO;						//pin in progress
				pin2cnt = 0;
				pinlen = read_ext_eeprom(MEM, PIN_LEN);
			}
			if (pin2stat == PIN_PRO)
			{
				x = rd2idat[0]; 
				if (rd2ibits == 4) x >>= 4;				//4 bit burst
				if (x == 0x23) x = 0x0b;				//RH320 #
				x &= 0x0F;			//get pin
				x |= 0x30;
				if (isdigit(x)) *pin2p++ = x;
				if (x == 0x3a) pin2stat = PIN_NO;					
				else 
				if (++pin2cnt == 0x04)
				{
					if (pinlen == 0x04)
					{
						pin2tick = 60;					//3 secs to present card
						pin2stat = PIN_RDY;
					}
				}
				else 
				if (pin2cnt == 0x05)
				{
					if (x == 0x3b)						//# is last key  pressed
					{
						pin2dat[4] = 0x00;				//clear out digits 5 & 6 - causes incorrect calc
						pin2dat[5] = 0x00;
						pinlist = check_pin(pin2dat,1);			
						if (pinlist == 0x00)			//Not in list of 10
						{
							evt.evt = E1_WPIN;
							pin2stat = PIN_NO;
						}
						else 
						if (pinlist == 0x01)			//PIN is in list of 10 stored PINs and acc lev valid
						{
							if (crcflags & 0x02)		//if single door
							{
								relay_on(1, 0);			//relay 1 on
								dr1stat = DOOR_REL;			//door released
							}
							else
							{
								relay_on(2, 0);			//relay 2 on
								dr2stat = DOOR_REL;			//door released
							}
							evt.evt = E2_APINOK;	
							pin2stat = PIN_NO;
						}
						else 
						if (pinlist == 0x02)			//PIN is in list but acc lev not currently valid
						{
							evt.evt = E2_APINNF;
							pin2stat = PIN_NO;
						}
						store_event();
					}
				}
				else 
				if ((pin2cnt == 0x06) && (pinlen == 0x06))
				{
					if (check_pin(pin2dat,1) == 0x00)
					{
						pin2tick = 60;					//3 secs to present card
						pin2stat = PIN_RDY;
					}
				}
			} 
			rd2istat = CARD_NO;
		} 
		else 
		if (rd2istat > CARD_PIN) rd2istat = CARD_NO;
		if (t2tout)
		{
		//Check for rex1 activity			
			if (!input(rex1))							//if rex1 low
			{
				if (input(auxi2))
				{
					if (rex1cnt > 0)
					{
						if (!--rex1cnt)
						{
							//report rex1 event only if door normal or door is open AND a muster controller
							if (dr1stat == DOOR_NO) zeroevent(E1_REX);	//rex 1 event
							else if ((crcflags & 0x08) != 0x00) zeroevent(E1_REX);	//rex 1 event
							relay_on(1, 0);			//relay 1 on
						}
					}
				}
			}
			else
			{
				rex1cnt = DBOUNCE;
			}
			//Check door1
			if (drmon1 != 0)
			{
				crcflags = read_ext_eeprom(MEM, CTR_FLAGS);	//controller flags
				if (input(door1))	//door open?
				{
					if (door1cnt)
					{
						if (!--door1cnt) 
						{
							if (dr1stat == DOOR_NO)
							{
								output_high(auxo1);			//local alarm
								zeroevent(E1_DFORCE);		//door forced open
								x = read_ext_eeprom(MEM, ALTRIGS);
								if (bit_test(x, 1)) 
								{
									alarmtick = read_ext_eeprom(MEM, ALTIME);
									alarmtout = false;
								}
								dr1stat = DOOR_FOP;			//door forced open
							}
							else 
							if (dr1stat == DOOR_REL)
							{
								if (crcflags & 0x10) dr1atg = 1;	//anti-tailgate disabled so flag and don't lock
								else
								{
									output_low(relay1);		//relay 1 off
									output_low(gled1);		//green led 1 off
									if (crcflags & 0x02) output_low(gled2);	//if single door
								}
								dr1stat = DOOR_RELOP;	//door released & open
								x = read_ext_eeprom(MEM, DOOR1_TM);	//door open time
								dr1tick = x * 2;
								dr1tout = 0;
							}
						}
					}
				}
				else
				if (door1cnt < DBOUNCE)
				{
					if (++door1cnt >= DBOUNCE)			//door is closed
					{
						if (dr1stat == DOOR_FOP)		//forced open
						{
							dr1stat = DOOR_NO;	
							zeroevent(E1_DCLOSE);		//door close
						}
						else 
						if (dr1stat == DOOR_LOP)		//left open?
						{
//							if (!input(door2) && !input(auxi1)) 
							if (!input(door1)) 			//not sure why it checked door2 or tamper input?? - meant it didn't work!
							{
								output_low(auxo1);		//local alarm off
								dr1stat = DOOR_NO;		//door normal
								zeroevent(E1_DCLOSE);	//door close
							}
						}
						else 
						if (dr1stat == DOOR_RELOP) dr1stat = DOOR_NO;						
					}
				}
				if (dr1tout)								//door open time-out
				{
					if ((dr1stat == DOOR_NO) && (dr1atg == 1) && (input(door1))) //atg disabled & door is open
					{
						zeroevent(E1_DLOPEN);				//left open
						dr1stat = DOOR_LOP;					//door left open
						output_high(auxo1);					//local alarm on
						dr1atg = 0;
					}
					if (dr1stat == DOOR_RELOP)				//if released and open
					{
						zeroevent(E1_DLOPEN);				//left open
						dr1stat = DOOR_LOP;					//door left open
						output_high(auxo1);					//local alarm on
					}
					dr1tout = 0;
				}
			}
			if (r1tout)										//relay 1 timer 
			{
				if (dr1stat == DOOR_REL)
				{
					output_low(relay1);						//relay 1 off
					output_low(gled1);						//green led 1 off
					if (crcflags & 0x02) output_low(gled2);	//if single door
					if ((drmon1 != 0) && (!input(door1))) zeroevent(E1_DNOPEN);	//not opened
					dr1stat = DOOR_NO;
				}
				else 
				if (dr1stat == DOOR_NO)
				{
					output_low(relay1);						//relay 1 off
					output_low(gled1);						//green led 1 off
					if (crcflags & 0x02) output_low(gled2);	//if single door
					dr1stat = DOOR_NO;
				}
//added to lock door if ((ATG disabled) or (relay operated while door was open))
				else 
				if (((dr1stat == DOOR_RELOP) && (dr1atg == 1)) | ((dr1stat == DOOR_FOP) | (dr1stat == DOOR_LOP)))
				{
					output_low(relay1);						//relay 1 off
					output_low(gled1);						//green led 1 off
					if (crcflags & 0x02) output_low(gled2);	//if single door
				}
				r1tout = false;
			}
//Check for rex2 activity
			if (!input(rex2))								//if rex2 low
			{
//				if ((dr2stat == DOOR_NO) && input(auxi3)) **changed - unlock regardless of door state
				if (input(auxi3))
				{
					if (rex2cnt > 0)
					{
						if (!--rex2cnt)
						{
							//report rex2 event only if door normal or door is open AND a muster controller
							if (dr2stat == DOOR_NO) zeroevent(E2_REX);		//rex 1 event
							else if ((crcflags & 0x08) != 0x00) zeroevent(E2_REX);		//rex 1 event
							relay_on(2, 0);					//relay 2 on
						}
					}
				}
			}
			else
			{
				rex2cnt = DBOUNCE;
			}
//Check door2
			if ((drmon2 != 0) && !(crcflags & 0x02))
			{
				crcflags = read_ext_eeprom(MEM, CTR_FLAGS);	//controller flags
				if (input(door2))							//door open?
				{
					if (door2cnt)
					{
						if (!--door2cnt) 
						{
							if (dr2stat == DOOR_NO)
							{
								output_high(auxo2);			//door2 alarm
								zeroevent(E2_DFORCE); 		//door forced open
								x = read_ext_eeprom(MEM, ALTRIGS);
								if (bit_test(x, 1)) 
								{
									alarmtick = read_ext_eeprom(MEM, ALTIME);
									alarmtout = false;
								}
								dr2stat = DOOR_FOP;			//door forced open
							}
							else 
							if ((dr2stat == DOOR_REL))
							{
								if (crcflags & 0x20) dr2atg = 1;	//anti-tailgate disabled so flag and don't lock
								else
								{
									output_low(relay2);		//relay 2 off
									output_low(gled2);		//green led 2 off
								}
								dr2stat = DOOR_RELOP;	//door released & open
								x = read_ext_eeprom(MEM, DOOR2_TM);	//door open time
								dr2tick = x * 2;
								dr2tout = 0;
							}
						}
					}
				}
				else
				if (door2cnt < DBOUNCE)
				{
					if (++door2cnt >= DBOUNCE)			//door is closed
					{
						if (dr2stat == DOOR_FOP) 		//forced open
						{
							dr2stat = DOOR_NO;		
							zeroevent(E2_DCLOSE);		//door close
						}
						else 
						if (dr2stat == DOOR_LOP)	//left open?
						{
							if (!input(door2))
							{
								output_low(auxo2);		//door2 alarm off
								dr2stat = DOOR_NO;		//door normal
								zeroevent(E2_DCLOSE);	//door close
							}
						}
						else 
						if (dr2stat == DOOR_RELOP) dr2stat = DOOR_NO;						
					}
				}
				if (dr2tout)								//door open time-out
				{
					if ((dr2stat == DOOR_NO) && (dr2atg == 1) && (input(door2))) //atg is disabled and door is open
					{
						zeroevent(E2_DLOPEN);				//left open
						dr2stat = DOOR_LOP;					//door left open
						output_high(auxo2);					//local alarm on
						dr2atg = 0;
					}
					if (dr2stat == DOOR_RELOP)				//if released and open
					{
						zeroevent(E2_DLOPEN);				//left open
						dr2stat = DOOR_LOP;					//door left open
						output_high(auxo2);					//door2 alarm on
					}
					dr2tout = 0;
				}
			}
			if (r2tout)										//relay 2 timer 
			{
				if (dr2stat == DOOR_REL)
				{
					output_low(relay2);						//relay 2 off
					output_low(gled2);						//green led 2 off
					if ((drmon2 != 0) && (!input(door2))) zeroevent(E2_DNOPEN);	//not opened
					dr2stat = DOOR_NO;
				}
				else 
				if (dr2stat == DOOR_NO)
				{
					output_low(relay2);						//relay 2 off
					output_low(gled2);						//green led 2 off
					dr2stat = DOOR_NO;
				}
//added to lock door if ((ATG disabled) or (relay operated while door was open))
				else 
				if (((dr2stat == DOOR_RELOP) && (dr2atg == 1)) | ((dr2stat == DOOR_FOP) | (dr2stat == DOOR_LOP)))
				{
					output_low(relay2);						//relay 2 off
					output_low(gled2);						//green led 2 off
				}
				r2tout = false;
			}
			//Check aux input 1 (Tamper)
			oldio = instat;									//old input status			
			if (input(auxi1))								//input high?
			{
				if (!bit_test(instat, 0))					//already high?
				{
					if (++in1cnt == 0)
					{
						zeroevent(E0_TPON);					//Tamper ON
						bit_set(instat, 0);					//input is high
						output_high(auxo1);					//local alarm on		
					}
				}
			}
			else 
			if (bit_test(instat, 0))						//already low?
			{
				if (++in1cnt == 0)
				{
					zeroevent(E0_TPOK);						//Tamper is OK
					bit_clear(instat, 0);					//input is low
					if (!input(door1) && !input(door2)) output_low(auxo1);	//local alarm off
				}
			}
//			else 
//			if (!((dr1stat == DOOR_FOP) | (dr1stat == DOOR_LOP))) output_low(auxo1);
			//Above added in 2.13 because tamper alarm was not resetting when alarm removed
			//Removed in 2.14 because it breaks FOP alarm handling

			//Check aux input 2 (Arming Reader 1)
			if (input(auxi2))								//input high?
			{
				if (!bit_test(instat, 1))					//already high?
				{
					if (++in2cnt == 0)
					{
						zeroevent(E0_ARM1);					//reader 1 armed
						bit_set(instat, 1);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 1))						//already low?
			{
				if (++in2cnt == 0)
				{
					zeroevent(E0_NARM1);					//reader 1 not armed
					bit_clear(instat, 1);					//input is low
				}
			}
			//Check aux input 3 (Arming Reader 2)
			if (input(auxi3))								//input high?
			{
				if (!bit_test(instat, 2))					//already high?
				{
					if (++in3cnt == 0)
					{
						zeroevent(E0_ARM2);					//reader 2 armed
						bit_set(instat, 2);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 2))						//already low?
			{
				if (++in3cnt == 0)
				{
					zeroevent(E0_NARM2);					//reader 2 not armed
					bit_clear(instat, 2);					//input is low
				}
			}
			//aux in 4
			if (input(auxi4))								//input high?
			{
				if (!bit_test(instat, 3))					//already high?
				{
					if (++in4cnt == 0)
					{
						zeroevent(E0_NIO4);					//input 4 open
						bit_set(instat, 3);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 3))						//alredy low?
			{
				if (++in4cnt == 0)
				{
					crcflags = read_ext_eeprom(MEM, CTR_FLAGS);
#if defined(ONEDOOR)
					crcflags |= 0x02;
#endif
					if (crcflags & 0x01) zeroevent(E0_OPALL);	//open all doors
					else zeroevent(E0_IO4);					//input 4 closed 
					bit_clear(instat, 3);					//input is low
				}
			}

			//aux in 5
			if (input(auxi5))								//input high?
			{
				if (!bit_test(instat, 4))					//already high?
				{
					if (++in5cnt == 0)
					{
						zeroevent(E0_NIO5);					//input 5 open
						bit_set(instat, 4);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 4))						//alredy low?
			{
				if (++in5cnt == 0)
				{
					zeroevent(E0_IO5);						//input 5 closed
					bit_clear(instat, 4);					//input is low
				}
			}
			//aux in 6
			if (input(auxi6))								//input high?
			{
				if (!bit_test(instat, 5))					//already high?
				{
					if (++in6cnt == 0)
					{
						zeroevent(E0_NIO6);					//input 6 open
						bit_set(instat, 5);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 5))						//alredy low?
			{
				if (++in6cnt == 0)
				{
					zeroevent(E0_IO6);						//input 6 closed
					bit_clear(instat, 5);					//input is low
				}
			}
			//aux input 7 
			if (input(auxi7))								//input high?
			{
				if (!bit_test(instat, 6))					//already high?
				{
					if (++in7cnt == 0)
					{
						zeroevent(E0_NIO7);					//input 7 open
						bit_set(instat, 6);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 6))						//alredy low?
			{
				if (++in7cnt == 0)
				{
					zeroevent(E0_IO7);						//input 7 closed
					bit_clear(instat, 6);					//input is low
				}
			}
			//aux input 8
			if (input(auxi8))								//input high?
			{
				if (!bit_test(instat, 7))					//already high?
				{
					if (++in8cnt == 0)
					{
						zeroevent(E0_NIO8);					//input 8 open
						bit_set(instat, 7);					//input is high
					}
				}
			}
			else 
			if (bit_test(instat, 7))						//alredy low?
			{
				if (++in8cnt == 0)
				{
					zeroevent(E0_IO8);						//input 8 closed
					bit_clear(instat, 7);					//input is low
				}
			}
			t2tout = 0;
		}
		if (tout)		//every 10secs
		{
			//Check for new minute
			x = read_rtc_val(RTCMIN);
			if (x != oldmin)
			{
				oldmin = x;
				if (update_doortz(1)) store_event();
				if (update_doortz(2)) store_event();
			}
			if (++onesec > 3) onesec = 0;					//once per second
			tout = false;
		}
		//Check for new hour (APB reset)            
		if (apb)
		{
			curhour = newhour;
			newhour = read_ext_eeprom(RTC, RTCHOUR);
			if (newhour != curhour) 
			{
				x = read_ext_eeprom(MEM, APB_RST0);
				if (x == newhour) reset_apb();
				else
				{
					x = read_ext_eeprom(MEM, APB_RST1);
					if (x == newhour) reset_apb();   
					else
					{
						x = read_ext_eeprom(MEM, APB_RST2);
						if (x == newhour) reset_apb();   
						else
						{
							x = read_ext_eeprom(MEM, APB_RST3);
							if (x == newhour) reset_apb();   
						}
					}
				}
			}
		}
		//Check for serial comms
		//Poll sequence 0 - always return ACK
		if ((rxstat == RX_POLL) || (rxstat == RX_DATA))
		{
			node = ~input_f();
			node &= 0x1f;
			x = rxdat[0] & 0x1f;						//check correct node
			if (x == node) 
			{
				pollnext = rxdat[0] & 0x60;				//poll sequence
				if (pollnext == pollseq)
				{
					if (rxstat == RX_POLL) txpoll();
					else txpoll_data();					
				}
				else
				{
					sendack(node);
					pollseq = 0x00;
					evtsent = 0;
				}
				pollseq += 0x20;						//next sequence
				if (pollseq > 0x60) pollseq = 0x20;
			}                       
			rxstat = RX_NO;
		}
		else 
		if (rxstat > RX_DATA) rxstat = RX_NO;			//invalid rx status?
	}           
}
//---------------------------------------------------------------------
//Extracts Mag card info from reader buffer
//returns 0 if yes, 1 if wrong format, 2 if wrong site
//---------------------------------------------------------------------
char magcard(char rdr)
{
char acc, ax, bts, bts5, bts8, mstat, lrc, digs, *bp, *bf;
char sx[6];
union join16 site;

//1st find start char (0x0b)
	mstat = MAG_NO;						//status = no start char
	if (rdr == 1) 
	{
		bts = rd1ibits;					//reader 1 in
		bp = rd1idat;					//point to rdr1 data
	}
	else 
	if (rdr == 2)
	{
		bts = rd2ibits;					//reader 2 in
		bp = rd2idat;					//point to rdr2 data
	}
	bts8 = 8;							//8 bits per byte
	acc = *bp++;						//1st byte
	ax = 0;
	bf = buff;							//point to buffer
	*bf = 0x00; 
	digs = 0;							//zero digits counter
	while (bts > 0)
	{
		if (bit_test(acc, 7)) bit_set(ax, 4);	//set 1 bit
		acc <<= 1;						//shift byte
		if(--bts8 == 0)
		{
			acc = *bp++;				//get next byte
			bts8 = 8;
		}
		if (mstat == MAG_NO)
		{
			if (ax == 0x0b)
			{
				mstat = MAG_ST;			//start char found
				bts5 = 5;				//5 bits per char
				ax = 0;					//clear mag char
				lrc = 0x0b;				//init lrc check
			}
		}
		else
		if (--bts5 == 0)
		{
			if (!magcard_par(ax)) return RD_PARITY;	//parity error
			lrc ^= ax;
			if (mstat == MAG_END) 
			{
				lrc &= 0x0f;
				if (lrc != 0x00) return RD_LRC;		//lrc error
				break;
			}
			else 
			if (ax == 0x1f) mstat = MAG_END;	//end char
			else
			{
				*bf++ = (ax & 0x0f) + 0x30;			//store char
				++digs;								//count char
				if (digs > MAGDIGS) return RD_UNK;	//too many bits
				*bf = 0x00;
			}
			bts5 = 5;				//5 bits per char
			ax = 0;					//clear mag char
		}
		ax >>= 1;
		--bts;                          
	} 
	if (mstat != MAG_END) return RD_UNK;
	//Check number of digits in card
	//	bts = read_ext_eeprom(MEM, C_LENGTH);
	//	if (digs != bts) return 2;			
	//Extract and check sitecode/Card number
	bts = read_ext_eeprom(MEM, C_SITE_B);		//number of digits
	if (bts > 0)
	{
		bts5 = read_ext_eeprom(MEM, C_SITE_L);	//location of site
		strcpyn(sx, &buff[bts5], bts);
		evt.card.w = atol(sx);					//convert to hex
		site.b[0] = read_ext_eeprom(MEM, C_SITE_0);	//get stored site
		site.b[1] = read_ext_eeprom(MEM, C_SITE_1);
		if (evt.card.w != site.w) return RD_SITE;	//correct site?
	}
//Extract and return card number
	bts = read_ext_eeprom(MEM, C_CARD_B);		//number of digits
	bts5 = read_ext_eeprom(MEM, C_CARD_L);		//location of card
	buff[bts + bts5] = 0x00;
	evt.card.w = atoi32(&buff[bts5]);
	return RD_OK;
}
//---------------------------------------------------------------------
//Checks mag char for odd parity.
//return 1 if ok
//---------------------------------------------------------------------
short magcard_par(char cx)
{
char n, par;

	par = 0;					//parity check = 0
	for (n = 0; n < 5; n++)
	{
		if (bit_test(cx, 0)) ++par;
		cx >>= 1;
	}
	if (bit_test(par, 0)) return 1; else return 0;
}
//---------------------------------------------------------------------
//Checks wiegand card is correct format and sitecode
//returns 0 if yes, 1 if wrong format, 2 if wrong site
//---------------------------------------------------------------------
char wiegcard(char rdr)
{
char ax, ebts, obts, par, cb, x, *bp, *bps;
int16 site;
int32 crd;

	//must have correct number of bits
	x = read_ext_eeprom(MEM, C_LENGTH);
	if (rdr == 1)
	{
		if (rd1ibits != x) return RD_FMT;
		bp = rd1idat;
	}
	else
	{
		if (rd2ibits != x) return RD_FMT;
		bp = rd2idat;
	}
	bps = bp;									//save
	//Check even parity on 1st part of card
    ebts = read_ext_eeprom(MEM, W_EVEN);		//even parity bits
    if (ebts > 0)								//if parity check
    {
	    par = 0;								//parity count
    	cb = 8;									//8 bits per byte
    	ax = *bp++;
		for (x = 0; x < ebts; ++x)
		{
			if (bit_test(ax, 7)) ++par;			//count 1 bit
			ax <<= 1;							//shift for next bit
			if (--cb == 0)
			{
				cb = 8;
				ax = *bp++;						//next byte
			}
		}
		if (bit_test(par, 0)) return RD_PARITY;	//even parity error
	}
	//Check odd parity on 2nd part of card		
    obts = read_ext_eeprom(MEM, W_ODD);			//odd parity bits
    if (obts > 0)								//if parity check
    {	
    	par = 0;								//parity count
		for (x = 0; x < obts; ++x)
		{
			if (bit_test(ax, 7)) ++par;		//count 1 bit
			ax <<= 1;							//shift for next bit
			if (--cb == 0)
			{
				cb = 8;
				ax = *bp++;						//next byte
			}
		}
		if (!bit_test(par, 0)) return RD_PARITY;//odd parity error
	}
	//Card format ok, now check sitecode
	ebts = read_ext_eeprom(MEM, C_SITE_L);		//location of sitecode
	obts = read_ext_eeprom(MEM, C_SITE_B);		//bits in sitecode
	if (obts > 0)								//if check sitecode
	{
		bp = bps;								//point to data start
		bp += (ebts / 8);						//byte location
		x = ebts % 8;
		cb = 8 - x;								//bits location
		ax = *bp++;
		ax <<= x;
		site = 0;
		while (obts > 0)
		{
			if (bit_test(ax, 7)) shift_left(&site, 2, 1);
			else shift_left(&site, 2, 0);
			rotate_left(&ax, 1);
			if (--cb == 0)
			{
				cb = 8;
				ax = *bp++;
			}
			--obts;
		}
		evt.card.w = site;
		par = site >> 8;						//site high byte
		x = read_ext_eeprom(MEM, C_SITE_1);		//get stored site
		if (x != par) return RD_SITE;			//wrong site
		par = site;								//site low byte
		x = read_ext_eeprom(MEM, C_SITE_0);
		if (x != par) return RD_SITE;			//wrong site
	}
	//Sitecode ok, now extract card number
	ebts = read_ext_eeprom(MEM, C_CARD_L);		//location of card #
	obts = read_ext_eeprom(MEM, C_CARD_B);		//bits in card #
	bp = bps;									//point to data start
	bp += (ebts / 8);							//byte location
	x = ebts % 8;
	cb = 8 - x;									//bits location
	ax = *bp++;
	ax <<= x;
	crd = 0;
	while (obts > 0)	
	{
		if (bit_test(ax, 7)) shift_left(&crd, 4, 1);
		else shift_left(&crd, 4, 0);
		rotate_left(&ax, 1);
		if (!--cb)
		{
			cb = 8;
			ax = *bp++;
		}
		--obts;
	}
	evt.card.w = crd;
	return 0;							//ok
}
//---------------------------------------------------------------------
//check_card()
//Process card info, format and site already checked
//card number in global variable card
//---------------------------------------------------------------------
char check_card(char rdr)
{
char acc, db, bf[6];
char c1, c2, c3, c4, isaccomm;
long n;

	read_rtc();								//get current time
	db = read_ext_eeprom(MEM, CARD_DB);		//card database
	crdst.pin.w = 0x00000000;
	crdst.data = 0x00;

	//Check if the card is an accommodation card first; if not run db searches
	//ACCOMCARDS has room 1 cards in first 10 locations (4 bytes per card)
	//room 2 in second 10 locations
    isaccomm = 0;
	c1 = evt.card.b[3];
	c2 = evt.card.b[2];
	c3 = evt.card.b[1];
	c4 = evt.card.b[0];
	crdst.addr = ACCOMCARDS;
	for (n = 0; n < 20; n++)
	{
		bf[0] = 4;
		readb_ext_eeprom(crdst.addr, bf);
		if (bf[1] == c1)
		{
			if (bf[2] == c2)
			{
				if (bf[3] == c3)
				{
					if (bf[4] == c4)	//card found?
					{
						if ((n < 10) & (rdr == 1)) isaccomm = 1;
						if ((n > 9) & (rdr == 2)) isaccomm = 2;
					}
				}
			}
		}
		restart_wdt();
		crdst.addr += 4;
	}

	if (rdr == 1)
	{
		if (db == DB_PIN)			//card and PIN
		{
			crdst.index = evt.card.w;
			if (crdst.index > 16000) return E1_CARDNF;	//card out of range
			crdst.index *= (long)4;
			bf[0] = 4;
			readb_spi_eeprom(crdst.index, bf);	//read card info
			crdst.data = bf[1];
			crdst.pin.w = make32(0, bf[2], bf[3], bf[4]);
		}
		else 
		if (db == DB_RAN)	//random
		{	
			if (!findcard(5)) return E1_CARDNF;	//card not found
		}
		else 
		if (db == DB_PRAN)	//random + pin
		{
			if (!findcard(8)) return E1_CARDNF;	//card not found
		}
		else		//sequential card only
		{
			crdst.index = evt.card.w;
			crdst.data = read_spi_eeprom(evt.card.w);	//read card info
		}
		acc = (crdst.data >> 4) & 0x0f;
		if (acc == 0x00) return E1_AL;
		if (isaccomm == 1) return E1_ACC;
		if (!check_access(0, acc)) return E1_TZ;
		if (crdst.pin.w != 0x00000000) 
		{
			if (check_access(2, 0))	return E1_PIN;
		}
		return E1_ACC;								//card ok
	}
	else 
	if (rdr == 2)
	{       
		if (db == DB_PIN)			//card and PIN
		{
			crdst.index = evt.card.w;
			if (crdst.index > 20000) return E2_CARDNF;	//card out of range
			crdst.index *= (long)4;
			bf[0] = 4;
			readb_spi_eeprom(crdst.index, bf);	//read card info
			crdst.data = bf[1];
			crdst.pin.w = make32(0, bf[2], bf[3], bf[4]);
		}
		else 
		if (db == DB_RAN)	//random
		{	
			if (!findcard(5)) return E2_CARDNF;	//card not found
		}
		else 
		if (db == DB_PRAN)	//random + pin
		{
			if (!findcard(8)) return E2_CARDNF;	//card not found
		}
		else		//sequential card only
		{
			crdst.index = evt.card.w;
			crdst.data = read_spi_eeprom(crdst.index);	//read card info
		}
		evt.evt = E2_TZ;								//timezone            
		acc = (crdst.data >> 4) & 0x0f;
		if (acc == 0x00) return E2_AL;
		if (isaccomm == 2) return E2_ACC;
		if (!check_access(1, acc)) return E2_TZ;
		if (crdst.pin.w != 0x00000000) 
		{
			if (check_access(3, 0)) return E2_PIN;
		}
		return E2_ACC;
	}
}
//---------------------------------------------------------------------
//reset_apb()
//Reset Anti-passback
//---------------------------------------------------------------------
void reset_apb(void)
{

	apb = read_ext_eeprom(MEM, APBMODE);
	if (apb == 0) return;			//apb not enabled
	zero_eeprom(0, 16384);	
}
//---------------------------------------------------------------------
//store_event()
//Store current event into event memory and adjust pointers etc
//---------------------------------------------------------------------
void store_event(void)
{
union join16 getad, putad;
char bf[16];

	bf[0] = 4;
	readb_ext_eeprom(evtputl, bf);
	putad.b[0] = bf[1];				//put address
	putad.b[1] = bf[2];
	getad.b[0] = bf[3];				//get address
	getad.b[1] = bf[4];
	bf[0] = 9;
	bf[1] = evt.date;
	bf[2] = evt.hour;
	bf[3] = evt.min;
	bf[4] = evt.sec;
	bf[5] = evt.card.b[3];
	bf[6] = evt.card.b[2];
	bf[7] = evt.card.b[1];
	bf[8] = evt.card.b[0];
	bf[9] = evt.evt;
	writeb_ext_eeprom(putad.w, bf);
	putad.w += EVTSIZE;
	if (putad.w > EVTTOP)								//if top then
	{											//back to bottom
		putad.w = EVTBASE;	
	}
	bf[0] = 2;
	bf[1] = putad.b[0];
	bf[2] = putad.b[1];
	writeb_ext_eeprom(evtputl, bf);
	if (getad.w == putad.w)								//if = then full 
	{										
		getad.w += EVTSIZE;							//adjust get address
		if (getad.w > EVTTOP)							//if top then
		{										//back to bottom
			getad.w = EVTBASE;
		}
		write_ext_eeprom(MEM, evtgetl, getad.b[0]);		//save get address	
		write_ext_eeprom(MEM, evtgeth, getad.b[1]);	
	}
}
//---------------------------------------------------------------------
//read_rtc()
//Reads current value of real time clock into evt structure
//added validity checks - 29/10/04
//---------------------------------------------------------------------
void read_rtc(void)
{
char x;

	x = read_ext_eeprom(RTC, RTCCON);
	bit_set(x, 0);
	write_ext_eeprom(RTC, RTCCON, x);
	delay_ms(1);
    evt.sec = read_ext_eeprom(RTC, RTCSEC);
    evt.min = read_ext_eeprom(RTC, RTCMIN);
    evt.hour = read_ext_eeprom(RTC, RTCHOUR);
    evt.day = read_ext_eeprom(RTC, RTCDAY);
    evt.date = read_ext_eeprom(RTC, RTCDATE);
    evt.mon = read_ext_eeprom(RTC, RTCMON);
    evt.year = read_ext_eeprom(RTC, RTCYEAR);
	bit_clear(x, 0);
	write_ext_eeprom(RTC, RTCCON, x);
}
//---------------------------------------------------------------------
//check_timezone()
//Checks timezone of card. Returns 1 if card ok else 0.
//Need to call read_rtc() before calling this function
//---------------------------------------------------------------------
short check_timezone(char tz)
{
long addr;

	if (tz == 0) return false;			//never
	if (tz == 1) return true;			//always
	if (tz == 2) addr = TZ21FH;			//timezone 2
	else if (tz == 3) addr = TZ31FH;	//timezone 3
	else return false;
	if (tz_check(addr)) return 1;
	else return tz_check(addr + 5);
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
short tz_check(long addr)
{
char x, z;

	x = read_ext_eeprom(MEM, addr);			//hour from
	if (evt.hour < x) return false;			//out of tz
	++addr;
	if (evt.hour == x)
	{
		x = read_ext_eeprom(MEM, addr);		//minute from
		if (evt.min < x) return false;		//out of tz
	}                                   
	++addr;
	x = read_ext_eeprom(MEM, addr);			//hour to
	if (evt.hour > x) return false;			//out of tz
	++addr;
	if (evt.hour == x)
	{
		x = read_ext_eeprom(MEM, addr);		//minute to
		if (evt.min > x) return false;		//out of tz
	}                               
	++addr;
	x = read_ext_eeprom(MEM, addr);			//days of week
	z = evt.day - 1;
	if (bit_test(x, z)) return true;		//tz valid today
	return false; 
}
//---------------------------------------------------------------------------
//Acknowledge host, single byte (nd)
//control tx enable line if RS485 (sw7 off)
//---------------------------------------------------------------------------
void sendack(char nd)
{

	output_low(led);						//flash online led
	delay_ms(1);
	output_high(txon);						//RS485 = TX
//	delay_ms(TXDEL);
	putchar(nd);
	while (!TRMT) restart_wdt();
//****
//delay_ms(10);
//****
	output_low(txon);						//RS485 = RX
	output_high(led);
}
//---------------------------------------------------------------------------
//clear_events()
//Clear all log events and reset pointers etc
//---------------------------------------------------------------------------
void clear_events(short cf)
{
union join16 addr;

	addr.w = EVTBASE;
	write_ext_eeprom(MEM, evtputl, addr.b[0]);
	write_ext_eeprom(MEM, evtputh, addr.b[1]);
	write_ext_eeprom(MEM, evtgetl, addr.b[0]);
	write_ext_eeprom(MEM, evtgeth, addr.b[1]);
	evtsent = false;
}
//---------------------------------------------------------------------------
//txpoll()
//Reply to poll with no data. Send next event if avialable
//---------------------------------------------------------------------------
short txpoll(void)
{
union join16 getad, putad;
char x, cs, bf[16], *p;

	bf[0] = 4;
	readb_ext_eeprom(evtputl, bf);
	putad.b[0] = bf[1];				//put address
	putad.b[1] = bf[2];
	getad.b[0] = bf[3];				//get address
	getad.b[1] = bf[4];
	if (evtsent)							//event to be acknowledged
	{
        getad.w += EVTSIZE;				//next get address
		if (getad.w > EVTTOP)				//if top then
		{							//back to bottom
			getad.w = EVTBASE;
		}
		bf[0] = 2;
		bf[1] = getad.b[0];				//update get address
		bf[2] = getad.b[1];
		writeb_ext_eeprom(evtgetl, bf);
		evtsent = false;				//event has been acknowledged	
	}
	if (getad.w != putad.w)
	{
		delay_ms(1);
		output_high(txon);					//RS485 = TX
		delay_ms(TXDEL);
		x = 0x80 | node;
		putchar(x);
		cs = x;
		putchar(12);
		cs += 12;
		p = bf;
		*p++ = EVTSIZE;
		readb_ext_eeprom(getad.w, bf);
		for (x = 0; x < 4; x++)
		{
			putchar(*p);
			cs += *p++;
		}
		putchar(*p);			//card high byte
		cs += *p++;		
		putchar(*p);			//card low byte
		cs += *p++;
		putchar(*p);			//card high byte
		cs += *p++;		
		putchar(*p);			//card low byte
		cs += *p++;
		putchar(*p);			//event
		cs += *p++;
		putchar(0x00);						//reason
		putchar(0x00);						//location
		putchar(cs);
		while (!TRMT) restart_wdt();
		output_low(txon);				//RS485 = RX
		evtsent = 1;						//event has been sent
	}
	else sendack(rxdat[0]);
	return 1;
}
//---------------------------------------------------------------------------
//txpoll_data()
//Reply to poll with data.
//---------------------------------------------------------------------------
short txpoll_data(void)
{
char cnt, x, cs, rpl;
short ok;

	ok = 0;
	rpl = rxdat[0] & 0x7f;
	cnt = rxcnt - 1;
	cs = 0;
	for (x = 0; x < cnt; x++) cs += rxdat[x];
	if (cs != rxdat[cnt]) return 0;
	x = rxdat[2];
	switch (x)
	{
		case 0x61:	//program card
					if (writecard()) sendack(rpl);
					break;
		case 0x62:	//program block of cards
					if (card_block()) sendack(rpl);
					break;
/*		case 0x62:	//write block
					writeblock(&rxdat[3]);
					sendack(rpl);
					break;*/
        case 0x63:	//card format
					rxdat[2] = 12;
					writeb_ext_eeprom(CRD_FMT, &rxdat[2]);
					cardfmt = rxdat[3];
					setup_readers();
					sendack(rpl); 
					break;
		case 0x64:	//relay times + Door monitoring + Door open times
					if (rxdat[1] == 13) rxdat[2] = 11;
					else rxdat[2] = 7;
					writeb_ext_eeprom(REL1_TM, &rxdat[2]);
					drmon1 = rxdat[6];
					drmon2 = rxdat[7];
					update_doortz(1);					
					update_doortz(2);
					dr1stat = DOOR_NO;	//Added statuses because setting door mon on without contacts
					dr2stat = DOOR_NO;	//causes alarms which do not clear when door mon then removed
					sendack(rpl); 
					break;
		case 0x65:	//timezones  
					rxdat[2] = 20;
					writeb_ext_eeprom(TZ21FH, &rxdat[2]);   
					sendack(rpl);
					break;
		case 0x66:	//date and time           
					set_rtc();
					sendack(rpl);            
					break;
		case 0x67:	//PIN Code
/*					card = PIN1_L;
					x = rxdat[3];
					if (x < 10)
					{
						card += x * 2;			//point to location
						write_ext_eeprom(MEM, card++, rxdat[5]);
						write_ext_eeprom(MEM, card, rxdat[4]);
						sendack(rpl);
					}*/
					sendack(rpl);
					break;
		case 0x68:	//Clear Events
					clear_events(false);
					sendack(rpl);
//					send_mod(rpl);
//					sendack(rpl);
					break;                  
		case 0x69:	//Enable/disable APB
					rxdat[2] = 5;
					writeb_ext_eeprom(APBMODE, &rxdat[2]);
					if (rxdat[1] > 7) 
					{
						write_ext_eeprom(MEM, ATBTM1, rxdat[8]);
						write_ext_eeprom(MEM, ATBTM2, rxdat[9]);
					}
					sendack(rpl);
					break;							
		case 0x6A:	//Reset APB
					sendack(rpl);
					reset_atb();
					reset_apb();
					break;							
		case 0x6B:	//Operate output relay
					relay_on(rxdat[3], rxdat[4]);
					sendack(rpl);
					break;							
		case 0x6c:	//Clear memory
					clear_cards();
					sendack(rpl);
					break;
		case 0x6d:	//Set door mode (open/active)
					if (rxdat[3] == 1)
					{
						if (rxdat[4] == 1)			//close door
						{
							if (dr1stat == DOOR_SOP)
							{
								output_low(relay1);	//relay1 off
								output_low(gled1);	//green led1 off
								if (crcflags & 0x02) output_low(gled2);
								dr1stat = DOOR_NO;	//set normal
								if (update_doortz(1)) store_event();
							}
							else 
							if (dr1stat == DOOR_NO) output_low(auxo1);		  //turn off alarm
						}

						else						//open door
						{
							dr1stat = DOOR_SOP;		//set open
							r1count++;
							rcountsave(1);									//increment relay count
							output_high(relay1);	//relay1 on
							output_high(gled1);		//green led1 on
							if (crcflags & 0x02) output_high(gled2);
						}
					}
					else 
					if (rxdat[3] == 2)
					{
						if (rxdat[4] == 1)			//close door
						{
							if (dr2stat == DOOR_SOP)
							{
								output_low(relay2);	//relay2 off
								output_low(gled2);	//green led2 off
								dr2stat = DOOR_NO;	//set normal
								if (update_doortz(2)) store_event();
							}
							else 
							if (dr2stat == DOOR_NO) output_low(auxo2);  	//turn off alarm
						}
						else						//open door
						{
							dr2stat = DOOR_SOP;		//set open
							r2count++;
							rcountsave(2);									//increment relay count
							output_high(relay2);	//relay2 on
							output_high(gled2);		//green led2 on
						}
					}
					sendack(rpl);
					break;          
		case 0x6e:	//set extended timezones
					rxdat[2] = TZQTY * TZSIZE;
					writeb_ext_eeprom(TZ2, &rxdat[2]);
					sendack(rpl);
					break;          
		case 0x6f:	//set accesslevels
					rxdat[2] = ACCQTY * ACCSIZE;
					writeb_ext_eeprom(ACC0, &rxdat[2]);
					sendack(rpl);
					break; 
		case 0x71:	//local alarm control
					if (rxdat[3] == 1) 
					{
						if (rxdat[4] != 0) output_high(auxo1);
						else output_low(auxo1);
					}
					else if (rxdat[3] == 2) 
					{
						if (rxdat[4] != 0) output_high(auxo2);
						else output_low(auxo2);
					}
					else if (rxdat[3] == 3) 
					{
						if (rxdat[4] != 0) output_high(auxo3);
						else output_low(auxo3);
					}
					else if (rxdat[3] == 4) 
					{
						if (rxdat[4] != 0) output_high(auxo4);
						else output_low(auxo4);
					}
					sendack(rpl);
					break;
		case 0x72:	//set door timezones
					rxdat[2] = DTZQTY * DTZSIZE + 4;
					writeb_ext_eeprom(DTZ1, &rxdat[2]);
					sendack(rpl);
					break;          
		case 0x73:	//Config controller
					rxdat[2] = rxdat[1] - 4;
					rxdat[30] = node;		//do not change node
					writeb_ext_eeprom(CBASE, &rxdat[2]);
					setup_readers();
//					writeb_spi_eeprom(CBASE, &rxdat[2]);
					sendack(rpl);
					break;
		case 0x74:	//events on/off
					write_ext_eeprom(MEM, EVTS_ON, rxdat[3]);
					evtson = rxdat[3];
					sendack(rpl);
					break;
		case 0x75:	//set holidays
					rxdat[2] = 64;
					writeb_ext_eeprom(HOLS, &rxdat[2]);
					sendack(rpl);
					break;
		case 0x76:	//Accommodation cards
					rxdat[2] = 80;
					writeb_ext_eeprom(ACCOMCARDS, &rxdat[2]);
					sendack(rpl);
					break;
		case 0x77:	//Store up to 10 pins
					rxdat[2] = 33;
					writeb_ext_eeprom(PIN1_L, &rxdat[2]);
					sendack(rpl);
					break;
		case 0xf1:	//Set new keypad password
					write_ext_eeprom(MEM, PASSH, rxdat[3]);
					write_ext_eeprom(MEM, PASSL, rxdat[4]);
					sendack(rpl);
					break;
		case 0xf2:	//report relay counts
					rcount();
					break;
		case 0xf3:	//model of controller
					send_mod(rpl);
					break;
/*		case 0xf5:	//blank eeprom (0xff)
					blank_eeprom(0, 0x7FFF);
					sendack(rpl);
					break;
*/
		case 0xf5:	//load new firmware
					loadfw();
					break;
	}                               
	return 1;
}
//--------------------------------------------------------------------------- 
//relay_on()
//Operate output relay. rly = relay 1 or 2. 
//tm = time 1/10 secs (0 = use stored value)
//---------------------------------------------------------------------------
void relay_on(char rly, char tm)
{
	char x;
	if (rly == 1)
	{
		if (dr1stat == DOOR_NO)							//don't count non relay fires
		{												//an already open door won't fire relay
			r1count++;
			rcountsave(1);
		}
		output_high(relay1);							//relay 1 on
		output_high(gled1);								//green led on
		if (crcflags & 0x02) output_high(gled2);		//if single door
		if (tm > 0) r1tick = tm;
		else 
		{
			x = read_ext_eeprom(MEM, REL1_TM);			//relay time
			if (x == 0x00)								//0.5sec
			{
				tmr2tick = 208;
				r1tick = 1;
			}
			else r1tick = x * 2;
		}
		r1tout = false;
		if (drmon1 != 0)								//if door monitoring
		{
			dr1tout = false;
			if (dr1stat == DOOR_NO) 
			{
				dr1tick = x * 2 + 2;
				dr1stat = DOOR_REL;
			}
		}
	}
	else 
	{
		if (dr2stat == DOOR_NO)							//don't count non relay fires
		{												//an already open door won't fire relay
			r2count++;
			rcountsave(2);
		}
		output_high(relay2);							//relay 2 on
		output_high(gled2);								//green led on
		if (crcflags & 0x02) output_high(gled1);		//if single door
		if (tm > 0) r2tick = tm;
		else 
		{
			x = read_ext_eeprom(MEM, REL2_TM);			//relay time
			if (x == 0x00)								//0.5sec
			{
				tmr2tick = 208;
				r2tick = 1;
			}
			else r2tick = x * 2;
		}
		r2tout = false;
		if (drmon2 != 0)								//if door monitoring
		{
			dr2tout = false;
			if (dr2stat == DOOR_NO) 
			{
				dr2tick = x * 2 + 2;
				dr2stat = DOOR_REL;
			}
		}
	}
}

//---------------------------------------------------------------------------
//powerup_check()
//Create a power up event.
//---------------------------------------------------------------------------
void powerup_check(void)
{

	read_rtc();
	evt.card.w = 0; 
	if (!bit_test(RCON, 0)) evt.evt = E0_BOR;		//brown out
	else if (!bit_test(RCON, 1)) evt.evt = E0_POR;	//power on
	else if (!bit_test(RCON, 3)) evt.evt = E0_WDT;	//watch dog timeout
	RCON |= 0x13;  
	store_event();
}
//---------------------------------------------------------------------------
//Check if door timezone is active
//call read_rtc() before this function
//return true if door tz status has changed
//---------------------------------------------------------------------------
short update_doortz(char rdr)
{
short chg, op;
unsigned char n, x;
unsigned long tz;

	chg = 0;
	read_rtc();
	op = 0;
	if (rdr == 1)
	{
		x = read_ext_eeprom(MEM, DOOR1_TZ);	//door 1 timezone
		if (x == 0) goto updz1;
		else 
		if (x == 1)
		{
			op = 1;
			goto updz1;
		}
		bf[0] = 2;
		readb_ext_eeprom(DACC0, bf);
		if ((bf[1] == 0x00) && (bf[2] == 0x00)) 
		{
			if (dr1stat == DOOR_TZOP) goto updz1;
			else return 0;
		}
		x = bf[2];
		tz = DTZ1;								//point to tz1
		for (n = 0; n < 8; n++)					//1st 8 door timezones
		{
			if (bit_test(x, n))
			{
				if (tz_check(tz)) 
				{
					op = 1;		//timezone valid
					goto updz1;
				}
			}
			tz += (unsigned long)DTZSIZE;		//point to next tz
		}
		x = bf[1];
		for (n = 0; n < 8; n++)					//next 8 timezones
		{
			if (bit_test(x, n))
			{
				if (tz_check(tz)) 
				{
					op = 1;	//timezone valid
					goto updz1;
				}
			}
			tz += (unsigned long)DTZSIZE;		//point to next tz
		}
updz1:
		if (op == 1)
		{
			if ((dr1stat != DOOR_TZOP))			//if not already open on TZ
			{							
				r1count++;
				rcountsave(1);					//increment relay count
				output_high(relay1);			//relay 1 on
				output_high(gled1);				//green led 1 on
				if (crcflags & 0x02) output_high(gled2);
				evt.card.w = 0;
				evt.evt = E1_OPTZ;			//dr1 opened
				chg = true;
				dr1stat = DOOR_TZOP;
			}
		}
		else
		if (dr1stat == DOOR_TZOP)			//if currently open on TZ
		{
			evt.card.w = 0;
			evt.evt = E1_CLTZ;				//dr1 closed
			chg = true;
			output_low(relay1);				//relay 1 off
			output_low(gled1);				//green led 1 off
			if (crcflags & 0x02) output_low(gled2);
			dr1stat = DOOR_NO;
		}
	}
	else
	{
		x = read_ext_eeprom(MEM, DOOR2_TZ);	//door 2 timezone
		if (x == 0) goto updz2;
		else 
		if (x == 1)
		{
			op = 1;
			goto updz2;
		}
		bf[0] = 2;
		readb_ext_eeprom(DACC1, bf);
		if ((bf[1] == 0x00) && (bf[2] == 0x00)) 
		{
			if (dr2stat == DOOR_TZOP) goto updz2;
			else return 0;
		}
		x = bf[2];
		tz = DTZ1;							//point to tz1
		for (n = 0; n < 8; n++)				//1st 8 door timezones
		{
			if (bit_test(x, n))
			{
				if (tz_check(tz)) 
				{
					op = 1;					//timezone valid
					goto updz2;
				}
			}
			tz += (unsigned long)DTZSIZE;	//point to next tz
		}
		x = bf[1];
		for (n = 0; n < 8; n++)				//next 8 timezones
		{
			if (bit_test(x, n))
			{
				if (tz_check(tz)) 
				{
					op = 1;	//timezone valid
					goto updz2;
				}
			}
			tz += (unsigned long)DTZSIZE;	//point to next tz
		}
updz2:
		if (op == 1)
		{
			if ((dr2stat != DOOR_TZOP))		//if not already open on TZ
			{							
				r2count++;
				rcountsave(2);									//increment relay count
				output_high(relay2);		//relay 2 on
				output_high(gled2);			//green led 2 on
				evt.card.w = 0;
				evt.evt = E2_OPTZ;			//dr2 opened
				chg = true;
				dr2stat = DOOR_TZOP;
			}
		}
		else
		if (dr2stat == DOOR_TZOP)			//if currently open on TZ
		{
			evt.card.w = 0;
			evt.evt = E2_CLTZ;				//dr2 closed
			chg = true;
			output_low(relay2);				//relay 2 off
			output_low(gled2);				//green led 2 off
			dr2stat = DOOR_NO;
		}
	}
	return chg;
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void clear_cards(void)
{
	zero_spi_eeprom();
	zero_eeprom(0, 16384);	
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void zeroevent(char ev)
{
	read_rtc();
	evt.card.w = 0; 
	evt.evt = ev;
	store_event();
}
//---------------------------------------------------------------------------
//Send model of controller to host
//---------------------------------------------------------------------------
void send_mod(char rpl)
{
char cs;

	output_high(txon);					//RS485 = TX
	delay_ms(TXDEL);
	cs = rpl + 0x80;
	putchar(cs);
	putchar(0x04);
	cs += 0x04;
	putchar(MODVER);
	cs += MODVER;
	putchar(major_ver);
	cs += major_ver;
	putchar(minor_ver);
	cs += minor_ver;
	putchar(cs);
	while (!TRMT) restart_wdt();
	delay_us(TXDE);
	output_low(txon);					//RS485 = RX
}
//---------------------------------------------------------------------------
//Calculate memory checksums
//---------------------------------------------------------------------------
long mem_check(void)
{
long addr, cs;
char x;

	cs = 0;
	for (addr = 1; addr <= CRDTOP; addr++)
	{
		x = read_ext_eeprom(MEM, addr);
		cs += x & 0xFC;					//ignore apb bits
	}
	write_ext_eeprom(MEM, CRDSUML, cs);
	write_ext_eeprom(MEM, CRDSUMH, cs >> 8);
	return cs;
}
//---------------------------------------------------------------------------
//Calculate memory checksums
//---------------------------------------------------------------------------
long cfg_check(void)
{
long addr, cs;
char x;

	cs = 0;
	for (addr = CBASE + 1; addr <= CBASE + 90; addr++)
	{
		x = read_ext_eeprom(MEM, addr);
		cs += x & 0xFC;					//ignore apb bits
	}
	write_ext_eeprom(MEM, CFGSUML, cs);
	write_ext_eeprom(MEM, CFGSUMH, cs >> 8);
	return cs;
}
//---------------------------------------------------------------------------
//Initialize RTC
//---------------------------------------------------------------------------
void init_rtc(void)
{
char x, bf[32], *p;

	x = read_ext_eeprom(MEM, DMODE1);
	if (x == 0xff)
	{
		p = bf;
		*p++ = sizeof(DEFAULT_SET);
		for (x = 0; x < sizeof(DEFAULT_SET); x++) *p++ = DEFAULT_SET[x];
		writeb_ext_eeprom(CBASE, bf);
	}
	write_ext_eeprom(RTC, RTCCON, 0x00);
	write_ext_eeprom(RTC, RTCCAL, 0x00);
	write_ext_eeprom(RTC, RTCCOMP, 0x00);
}
//---------------------------------------------------------------------------
//Set RTC
//---------------------------------------------------------------------------
void set_rtc(void)
{
char x;

	x = read_ext_eeprom(RTC, RTCCON);
	bit_set(x, 1);
	write_ext_eeprom(RTC, RTCCON, x);
	delay_ms(10);
	write_ext_eeprom(RTC, RTCSEC, rxdat[3]);
	write_ext_eeprom(RTC, RTCMIN, rxdat[4]);
	write_ext_eeprom(RTC, RTCHOUR, rxdat[5]);
	write_ext_eeprom(RTC, RTCDAY, rxdat[6]);
	write_ext_eeprom(RTC, RTCDATE, rxdat[7]);
	write_ext_eeprom(RTC, RTCMON, rxdat[8]);
	write_ext_eeprom(RTC, RTCYEAR, rxdat[9]);
	bit_clear(x, 1);
	write_ext_eeprom(RTC, RTCCON, x);
}
//---------------------------------------------------------------------------
//Process anti-timeback
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//Process anti-timeback
//---------------------------------------------------------------------------
short check_atb(char rdr)
{
char n, x, t, atbnext;
long mins, minrtc, crd;

	if (crdst.data & 0x02) return true;		//passback
	if (rdr == 1)
	{
		atbptr = atbst1;		//point to atb store
		atbnext = atbput1;		//next atb location
		t = read_ext_eeprom(MEM, ATBTM1);	//atb time
		if (t > 30) 
		{
			write_ext_eeprom(MEM, ATBTM1, 0);
			t = 0;
		}
	}
	else
	{
		atbptr = atbst2;		//point to atb store
		atbnext = atbput2;		//next atb location
		t = read_ext_eeprom(MEM, ATBTM2);	//atb time
		if (t > 30) 
		{
			write_ext_eeprom(MEM, ATBTM2, 0);
			t = 0;
		}
	}
	if (t == 0) return true;	//no atb
	minrtc = bcd2hex(evt.hour);
	minrtc *= 60;
	x = bcd2hex(evt.min);
	minrtc += x;
	for (n = 0; n < ATBSIZE; n++)
	{
		crd = evt.card.w;
		if (crd == atbptr->card)
		{
			if (evt.date == atbptr->date)
			{
				mins = minrtc - atbptr->min;
				if (t >= mins) return false;
			}
			atbptr->date = evt.date;
			atbptr->min = minrtc;
			return true;
		}
		++atbptr;
	}
//card not found in atb store
	if (rdr == 1)
	{
		if (atbput1 >= ATBSIZE) atbput1 = 0;
		atbptr = &atbst1[atbput1];
		atbptr->card = evt.card.w;
		atbptr->date = evt.date;
		atbptr->min = minrtc;
		if (++atbput1 >= ATBSIZE) atbput1 = 0;	
	}
	else
	{
		if (atbput2 >= ATBSIZE) atbput2 = 0;
		atbptr = &atbst2[atbput2];
		atbptr->card = evt.card.w;
		atbptr->date = evt.date;
		atbptr->min = minrtc;
		if (++atbput2 >= ATBSIZE) atbput2 = 0;	
	}
	return true;
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
char bcd2hex(char b)
{
char x;

	x = (b >> 4) * 10;
	x += b & 0x0f;
	return x;
}
//---------------------------------------------------------------------------
//check_access() - checks if card has a valid access level/timezone
//rdr = 0 or 1, pin = 2 or 3
//---------------------------------------------------------------------------
short check_access(char rdr, char acc)
{
long addr, tz;
char x, z, n;
	
	if (rdr == 0) addr = ACC0 + (ACCSIZE * acc); //locate accesslevel
	else if (rdr == 1) addr = ACC0 + (ACCSIZE * acc) + 2;
	else if (rdr == 2) addr = PIN1TZH;
	else if (rdr == 3) addr = PIN2TZH; 
	z = read_ext_eeprom(MEM, addr++);		//high byte
	x = read_ext_eeprom(MEM, addr);			//low byte
	if ((x == 0) && (z == 0)) return false;	//no timezone
	if (bit_test(x, 0)) return true;		//tz always = bit 0
	tz = TZ2;								//point to tz2
	for (n = 1; n < 8; n++)					//1st 7 user timezones
	{
		if (bit_test(x, n))
		{
			if (tz_check(tz)) return true;	//timezone valid
		}
		tz += TZSIZE;						//point to next tz
	}
	for (n = 0; n < 8; n++)					//next 8 timezones
	{
		if (bit_test(z, n))
		{
			if (tz_check(tz)) return true;	//timezone valid
		}
		tz += TZSIZE;						//point to next tz
	}
	return false;
}
//---------------------------------------------------------------------
//Initialize controller
//---------------------------------------------------------------------------
void initcrc(void)
{
union join16 lx;
char x;

//check event pointers
	lx.b[0] = read_ext_eeprom(MEM, evtputl);
	lx.b[1] = read_ext_eeprom(MEM, evtputh);
	if ((lx.w < EVTBASE) || (lx.w >= EVTTOP)) clear_events(0);
//check card format
	cardfmt = read_ext_eeprom(MEM, CRD_FMT);
	if (cardfmt > FMT_WIEG)
	{
		cardfmt = FMT_MAG;
		write_ext_eeprom(MEM, CRD_FMT, FMT_MAG);
	}
	x = read_ext_eeprom(MEM, PIN_LEN);
	if ((x != 4) && (x != 6))
	{
		write_ext_eeprom(MEM, PIN_LEN, 4);
	}
	mode1 = read_ext_eeprom(MEM, DMODE1);	//door 1 mode
	if (mode1 > 5)
	{
		mode1 = MD_CARD;
		write_ext_eeprom(MEM, DMODE1, MD_CARD);
	}
	mode2 = read_ext_eeprom(MEM, DMODE2);	//door 2 mode
	if (mode2 > 5)
	{
		mode2 = MD_CARD;
		write_ext_eeprom(MEM, DMODE2, MD_CARD);
	}
	apb = read_ext_eeprom(MEM, APBMODE);		//apb mode?
    powerup_check();
	drmon1 = read_ext_eeprom(MEM, DOOR1_TM);	//door1 open time
	drmon2 = read_ext_eeprom(MEM, DOOR2_TM);	//door2 open time
	reset_apb();					//reset anti-passback
	reset_atb();	//reset anti-timeback
	clrcrd = 0;		//cards not cleared
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------------
void reset_atb(void)
{
long *addr;
char n;

	addr = atbst1;
	for (n = 0; n < sizeof(atbst1); n++) *addr++ = 0x00;
	addr = atbst2;
	for (n = 0; n < sizeof(atbst2); n++) *addr++ = 0x00;
	atbput1 = 0;
	atbput2 = 0;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
char read_rtc_val(char addr)
{
char x, val;

	x = read_ext_eeprom(RTC, RTCCON);
	bit_set(x, 0);
	write_ext_eeprom(RTC, RTCCON, x);
    val = read_ext_eeprom(RTC, addr);
	bit_clear(x, 0);
	write_ext_eeprom(RTC, RTCCON, x);
	return val;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------------
void setup_readers(void)
{
	disable_interrupts(GLOBAL);
	ext_int_edge(0, H_TO_L);		//-ve edge
	ext_int_edge(1, H_TO_L);		//-ve edge
	ext_int_edge(2, H_TO_L);
	ext_int_edge(3, H_TO_L);		//-ve edge

	cardfmt = read_ext_eeprom(MEM, CRD_FMT);
	if (cardfmt > FMT_WIEG)
	{
		cardfmt = FMT_MAG;
		write_ext_eeprom(MEM, CRD_FMT, FMT_MAG);
	}  
	if (cardfmt == FMT_MAG)		//mag card
	{
		disable_interrupts(INT_EXT);	//rdr2 data
		enable_interrupts(INT_EXT1);	//rdr2 clk
		disable_interrupts(INT_EXT2);	//rdr1 data
		enable_interrupts(INT_EXT3);	//rdr2 clk
	}
	else						//wiegand format
	{
		enable_interrupts(INT_EXT);		//rdr2 D1
		enable_interrupts(INT_EXT1);	//rdr1 D0
		enable_interrupts(INT_EXT2);	//rdr2 D1
		enable_interrupts(INT_EXT3);	//rdr2 D0
	}
    enable_interrupts(GLOBAL);
}
//---------------------------------------------------------------------------
//CCS strncpy does not always supply a terminating '\0'
//---------------------------------------------------------------------
void strcpyn(char *s1, char *s2, char n)
{
char x;

	for (x = 0; x < n; x++)
	{
		if (*s2 == 0x00) break;
		*s1++ = *s2++;
	}
	*s1 = 0x00;
}
//---------------------------------------------------------------------------
//Find a random card in memory. Uses global variables crdaddr, crd32, crddata
//Returns 1 if found. crddata = card data, crdaddr = address of card data
//---------------------------------------------------------------------------
char findcard(char sz)
{
long n;
char c1, c2, c3, c4;
char bf[10];

	c1 = evt.card.b[3];
	c2 = evt.card.b[2];
	c3 = evt.card.b[1];
	c4 = evt.card.b[0];
	crdst.addr = 0;
	for (n = 0; n < 8000; n++)
	{
		bf[0] = sz;
		readb_spi_eeprom(crdst.addr, bf);
		if (bf[1] == c1)
		{
			if (bf[2] == c2)
			{
				if (bf[3] == c3)
				{
					if (bf[4] == c4)	//card found?
					{
						crdst.data = bf[5];
						crdst.index = n;
						if (sz == 8)
						{
							crdst.pin.w = make32(0, bf[6], bf[7], bf[8]);
						}
						else crdst.pin.w = 0x00000000;
						return 1;
					}
				}
			}
		}
		restart_wdt();
		crdst.addr += sz;
	}
	return 0;
}
//---------------------------------------------------------------------------
//Find 1st card space. (card data = 0)
//On entry crdaddr points to 1st space to start with
//On exit crdaddr points to available space
//Returns true if space found
//---------------------------------------------------------------------------
short findspace(void)
{
char c;
long n;

	for (n = 0; n < MAXCARDR; n++)
	{
		c = read_ext_eeprom(MEM, n);
		if (c == 0) 
		{
			crdst.addr = n * CRDSIZER;
			crdst.index = n;
			return 1;				//found
		}
		restart_wdt();
	}
	return 0;						//not found
}
//---------------------------------------------------------------------
//Write card to memory
//On entry card = evt.card, data = val
//---------------------------------------------------------------------
short writecard(void)
{
char db;
long addr;

	db = read_ext_eeprom(MEM, CARD_DB);		//card database
	if (db == DB_PIN)						//20000 seq cards + PIN
	{
		addr = make16(rxdat[5], rxdat[6]);
		if (addr > 16000) return false;
		addr *= (long)4;
		rxdat[8] = 4;						//4 bytes per card
		rxdat[9] = rxdat[7];
		writeb_spi_eeprom(addr, &rxdat[8]);
		return true;
	}
	else 
	if (db == DB_RAN)						//if random
	{
		addr = make16(rxdat[8], rxdat[9]);	//card index
		if (addr > 8000) return false;
		addr *= (long)5;					//5 bytes per card
		rxdat[2] = 5;
		if (rxdat[7] == 0x00)				//if delete card
		{									//write card as zero
			rxdat[3] = 0x00;
			rxdat[4] = 0x00;
			rxdat[5] = 0x00;
			rxdat[6] = 0x00;
		}
		writeb_spi_eeprom(addr, &rxdat[2]);
		return true;
	}
	else 
	if (db == DB_PRAN)						//pin + random
	{
		addr = make16(rxdat[8], rxdat[9]);
		if (addr > 8000) return false;
		addr *= (long)8;					//8 bytes per card
		rxdat[2] = 8;
		rxdat[8] = rxdat[10];
		rxdat[9] = rxdat[11];
		rxdat[10] = rxdat[12];
		if (rxdat[7] == 0x00)				//if delete card
		{									//write card as zero
			rxdat[3] = 0x00;
			rxdat[4] = 0x00;
			rxdat[5] = 0x00;
			rxdat[6] = 0x00;
		}
		writeb_spi_eeprom(addr, &rxdat[2]);
		return true;
	}
	else									//65536 sequential cards, no PIN
	{
		addr = make16(rxdat[5], rxdat[6]);
		write_spi_eeprom(addr, rxdat[7]);
		return true;
	}
}
//---------------------------------------------------------------------------
//writeblock()
//Write a block of up to 32 cards to card memory
//---------------------------------------------------------------------------
short writeblock(char *s)
{
//union join16 idx;
//char n, x, j, sf[130], *sp, ff[36], *fp;

/*	idx.b[1] = *s++;		//index high byte
	idx.b[0] = *s++;		//idex low byte
	if (input(_RAND))			//if not random
	{
		writeb_ext_eeprom(idx.w, s);
	}
	else
	{
		n = *s++;			//number of cards
		while (n > 0)
		{
			j = n % 32;
			if (j == 0) j = 32;
			sp = sf;
			*sp++ = j * 4;
			fp = ff;
			*fp++ = j;
			for (x = 0; x < j; x++)
			{
				*sp++ = *s++;
				*sp++ = *s++;
				*sp++ = *s++;
				*sp++ = *s++;
				*fp++ = *s++;
			}
			writeb_spi_eeprom(idx.w * 4, sf);		
			writeb_ext_eeprom(idx.w, ff);
			n -= j;
		}
	}*/
	return 1;	
}
//---------------------------------------------------------------------------
//check data in bf[] for arming on/off
//x = state of input (0 or 1) 
//---------------------------------------------------------------------------
void armcheck(short i)
{
	if (bf[2] == 1)			//rdr1
	{
		if (i)
		{
			if (bf[3] == 0) arm1 = 0;
			else arm1 = 1;
		}
		else
		{
			if (bf[3] != 0) arm1 = 0;
			else arm1 = 1;
		}
	}
	else 
	if (bf[2] == 2)			//rdr2
	{
		if (i)
		{
			if (bf[3] == 0) arm2 = 0;
			else arm2 = 1;
		}
		else
		{
			if (bf[3] != 0) arm2 = 0;
			else arm2 = 1;
		}
	}
}
//---------------------------------------------------------------------------
//check data in bf[] for tamper on/off
//x = state of input (0 or 1) 
//---------------------------------------------------------------------------
void tpcheck(short i)
{
	if (bf[2] == 0)	output_low(auxo1);
	else output_high(auxo1);
}
//---------------------------------------------------------------------------
//check aux input  
//i - low nibble = input 1 - 8, bit 7 = state  
//---------------------------------------------------------------------------
void iocheck(char i)
{
short st;

	if (bit_test(i, 7)) st = 1; else st = 0;
	i &= 0x0f;		//i = state of input
	bf[0] = 8;
	if (i == 1)	readb_ext_eeprom(INP1, bf);
	else if (i == 2) readb_ext_eeprom(INP2, bf);
	else if (i == 3) readb_ext_eeprom(INP3, bf);
	else if (i == 4) readb_ext_eeprom(INP4, bf);
	else if (i == 5) readb_ext_eeprom(INP5, bf);
	else if (i == 6) readb_ext_eeprom(INP6, bf);
	else if (i == 7) readb_ext_eeprom(INP7, bf);
	else if (i == 8) readb_ext_eeprom(INP8, bf);
	else return;
	if (bf[1] == INP_TAMP)		//tamper input 
	{
		if (bf[2] == 0)	output_low(auxo1);
		else output_high(auxo1);
	}
	else 
	if (bf[1] == INP_ARM)	//arming input 
	{
		if (bf[2] == 1)			//rdr1
		{
			if (st)
			{
				if (bf[3] == 0) arm1 = 0;
				else arm1 = 1;
			}
			else
			{
				if (bf[3] != 0) arm1 = 0;
				else arm1 = 1;
			}
		}
		else 
		if (bf[2] == 2)			//rdr2
		{
			if (st)
			{
				if (bf[3] == 0) arm2 = 0;
				else arm2 = 1;
			}
			else
			{
				if (bf[3] != 0) arm2 = 0;
				else arm2 = 1;
			}
		}
	}
	else 
	if (bf[1] == INP_SW)
	{
		if (bit_test(bf[2], 0))			//aux out 1
		{
			if (st)
			{
				if (bf[3] == 0) output_low(auxo1);
				else output_high(auxo1);
			}
			else
			{
				if (bf[3] != 0) output_low(auxo1);
				else output_high(auxo1);
			}
		}
		else 
		if (bit_test(bf[2], 1))			//aux out 2
		{
			if (st)
			{
				if (bf[3] == 0) output_low(auxo2);
				else output_high(auxo2);
			}
			else
			{
				if (bf[3] != 0) output_low(auxo2);
				else output_high(auxo2);
			}
		}
		else 
		if (bit_test(bf[2], 2))			//aux out 3
		{
			if (st)
			{
				if (bf[3] == 0) output_low(auxo3);
				else output_high(auxo3);
			}
			else
			{
				if (bf[3] != 0) output_low(auxo3);
				else output_high(auxo3);
			}
		}
		else 
		if (bit_test(bf[2], 3))			//aux out 4
		{
			if (st)
			{
				if (bf[3] == 0) output_low(auxo4);
				else output_high(auxo2);
			}
			else
			{
				if (bf[3] != 0) output_low(auxo4);
				else output_high(auxo2);
			}
		}
	}
}
//---------------------------------------------------------------------------
//ioinit()  
//Initialize inputs
//---------------------------------------------------------------------------
void ioinit(void)
{

	arm1 = 1;			//arm reader 1
	arm2 = 1;			//arm reader 2
	instat = 0x00;
	if (!input(auxi1)) instat |= 0x01;
	if (!input(auxi2)) instat |= 0x02;
	if (!input(auxi3)) instat |= 0x04;
	if (!input(auxi4)) instat |= 0x08;
	if (!input(auxi5)) instat |= 0x10;
	if (!input(auxi6)) instat |= 0x20;
	if (!input(auxi7)) instat |= 0x40;
	if (!input(auxi8)) instat |= 0x80;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
short card_block(void)
{
long addr;

	addr = make16(rxdat[3], rxdat[4]);
	if (addr > MAXCARD) return false;
/*	if (input(_RAND))			//if not random
	{
		rxdat[4] = 200;
		writeb_ext_eeprom(addr, &rxdat[4]);
	}
	else
	{
		rxdat[4] = 210;
		addr *= (long)CRDSIZER;
		writeb_spi_eeprom(addr, &rxdat[4]);
	}*/
	return true;
}
//---------------------------------------------------------------------
//set_defaults()
//Set the CRC220 to a known default state
//---------------------------------------------------------------------
void set_defaults(short cf)
{
char n;

	tbuf[1] = 3;		//relay1 3secs
	tbuf[2] = 3;		//relay2 3secs
	tbuf[3] = 0;		//door mon
	tbuf[4] = 0;		//door1 0secs
	tbuf[5] = 0;		//door1 0secs
	tbuf[6] = 0;		//door1 tz0
	tbuf[7] = 0;		//door1 tz0
	tbuf[8] = 1;		//door1 mode
	tbuf[9] = 1;		//door1 mode
	tbuf[10] = 0;		//
//Card Format, Mag or Wiegand
	if (!cf)
	{
		tbuf[11] = 0;		//mag format
		tbuf[12] = 10;		//10 digits
		tbuf[13] = 13;		//even parity
		tbuf[14] = 13;		//odd parity
		tbuf[15] = 4;		//site digits
		tbuf[16] = 2;		//site location
		tbuf[17] = 4;		//card digits
		tbuf[18] = 6;		//card location
		tbuf[19] = 0;		//site high byte
		tbuf[20] = 1;		//site low byte
	}
	else
	{
		tbuf[11] = 1;		//wieg format
		tbuf[12] = 26;		//26 bits
		tbuf[13] = 13;		//even parity
		tbuf[14] = 13;		//odd parity
		tbuf[15] = 8;		//site bits
		tbuf[16] = 1;		//site location
		tbuf[17] = 16;		//card bits
		tbuf[18] = 9;		//card location
		tbuf[19] = 0;		//site high byte
		tbuf[20] = 1;		//site low byte
	}
	tbuf[21] = 0;		//apb mode (off)
	tbuf[22] = 0;		//rdr1 atb
	tbuf[23] = 0;		//rdr2 atb
	tbuf[24] = 0;		//node
	tbuf[25] = 0;		//op mode
	tbuf[26] = 0xb8;	//password high byte
	tbuf[27] = 0x22;	//password low byte
	tbuf[28] = 0;		//Use password (no)
	for (n = 29; n < 180; n++) tbuf[n] = 0;
	tbuf[0] = 180;
	writeb_ext_eeprom(CBASE, tbuf);

//clear all programmable timezones
	zero_eeprom(TZ2, (TZQTY * TZSIZE));
//clear all access levels
	zero_eeprom(ACC0, (ACCQTY * ACCSIZE));
//set access level 1 for both readers/always
	write_ext_eeprom(MEM, (ACC1 + 1), 0x01);
	write_ext_eeprom(MEM, (ACC1 + 3), 0x01);
	zero_eeprom(HOLS, 0x00);
	clear_events(false);
}
//---------------------------------------------------------------------------
//dayofyear()
//Reads rtc and returns day of the year
//---------------------------------------------------------------------------
void dayofyear(void)
{
char x;
//long drtc;
			
	read_rtc();
	x = bcd2hex(evt.date);
	drtc = (long)x;
	if (evt.mon == 1) return;	//if january 
	drtc += 31;		//jan = 31 days
	if (evt.mon == 2) return;	//if february
	drtc += 28;
	if (evt.mon == 3)	//if March
	{
		if ((evt.year & 0x03) == 0) ++drtc;	//leap year
		return;
	}
	drtc += 31;
	if (evt.mon == 4) return;	//if April
	drtc += 30;
	if (evt.mon == 5) return;	//if May
	drtc += 31;
	if (evt.mon == 6) return;	//if June
	drtc += 30;
	if (evt.mon == 7) return;	//if July
	drtc += 31;
	if (evt.mon == 8) return;	//if August
	drtc += 31;
	if (evt.mon == 9) return;	//if september
	drtc += 30;
	if (evt.mon == 0x10) return;	//if October
	drtc += 31;
	if (evt.mon == 0x11) return;	//if November
	drtc += 30;
	return;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------------
char chkpin(char *pdat)
{
char x;

	if (pinlen == 6)
	{
		x = (crdst.pin.b[2] >> 4) + 0x30;
		if (*pdat++ != x) return 0;
		x = (crdst.pin.b[2] & 0x0F) + 0x30;
		if (*pdat++ != x) return 0;
	}
	x = (crdst.pin.b[1] >> 4) + 0x30;
	if (*pdat++ != x) return 0;
	x = (crdst.pin.b[1] & 0x0F) + 0x30;
	if (*pdat++ != x) return 0;
	x = (crdst.pin.b[0] >> 4) + 0x30;
	if (*pdat++ != x) return 0;
	x = (crdst.pin.b[0] & 0x0F) + 0x30;
	if (*pdat != x) return 0;
	return 1;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
char check_apb(char rdr)
{
long addr;
char md, dt;

	if (crdst.data & 0x02) return 1;	//passback
	apb = read_ext_eeprom(MEM, APBMODE);	//apb mode?
	if (apb == 1)					//anti-passback?
	{
		addr = crdst.index >> 2;
		md = crdst.index % 4;
		dt = read_ext_eeprom(MEM, addr);
		if (rdr == 1)				//reader 1
		{
			if (md == 0)
			{
				if (dt & 0x01)		//rdr1 last?
				{
					return 0;
				}
				else
				{
					dt |= 0x01;		//disable rdr1
					dt &= 0xFD;		//enable rdr2
				}
			}
			else 
			if (md == 1)
			{
				if (dt & 0x04)		//rdr1 last?
				{
					return 0;
				}
				else				//rdr2 last
				{
					dt |= 0x04;		//disable rdr1
					dt &= 0xF7;		//enable rdr2
				}
			}
			else 
			if (md == 2)
			{
				if (dt & 0x10)		//rdr1 last?
				{
					return 0;
				}
				else				//rdr2 last
				{
					dt |= 0x10;		//disable rdr1
					dt &= 0xDF;		//enable rdr2
				}
			}
			else 
			if (md == 3)
			{
				if (dt & 0x40)		//rdr1 last?
				{
					return 0;
				}
				else				//rdr2 last
				{
					dt |= 0x40;		//disable rdr1
					dt &= 0x7F;		//enable rdr2
				}
			}
		}
		else						//reader 2
		{
			if (md == 0)
			{
				if (dt & 0x02)		//rdr2 last?
				{
					return 0;
				}
				else
				{
					dt |= 0x02;		//disable rdr2
					dt &= 0xFE;		//enable rdr1
				}
			}
			else 
			if (md == 1)
			{
				if (dt & 0x08)		//rdr2 last?
				{
					return 0;
				}
				else				//rdr1 last
				{
					dt |= 0x08;		//disable rdr2
					dt &= 0xFB;		//enable rdr1
				}
			}
			else 
			if (md == 2)
			{
				if (dt & 0x20)		//rdr2 last?
				{
					return 0;
				}
				else				//rdr1 last
				{
					dt |= 0x20;		//disable rdr2
					dt &= 0xEF;		//enable rdr1
				}
			}
			else 
			if (md == 3)
			{
				if (dt & 0x80)		//rdr2 last?
				{
					return 0;
				}
				else				//rdr1 last
				{
					dt |= 0x80;		//disable rdr2
					dt &= 0xBF;		//enable rdr1
				}
			}
		}
		write_ext_eeprom(MEM, addr, dt);
	}
	return 1;		//OK
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
char buddycheck(char rdr)
{
int32 tsec;

	tsec = (int32)bcd2hex(evt.hour) * (int32)3600;
	tsec += (int32)bcd2hex(evt.min) * (int32)60;
	tsec += (int32)bcd2hex(evt.sec);
	if (rdr == 1)
	{
		if (lastcard1.card == 0x00000000)
		{
			lastcard1.card = evt.card.w;
			lastcard1.sec = tsec;
			return 0;
		}
		else 
		if (lastcard1.card == evt.card.w)
		{
			lastcard1.sec = tsec;
			return 0;
		}
		else 
		if ((tsec - lastcard1.sec) < buddytime)
		{
			tsec = evt.card.w;		//save current card
			evt.card.w = lastcard1.card;
			evt.evt = E1_ACC;
			store_event();
			evt.card.w = tsec;
			lastcard1.card = 0x00000000;
			return 1;
		}
		else
		{
			lastcard1.card = 0x00000000;
			lastcard1.sec = tsec;
			return 0;
		}
	}
	else 
	if (rdr == 2)
	{
		if (lastcard2.card == 0x00000000)
		{
			lastcard2.card = evt.card.w;
			lastcard2.sec = tsec;
			return 0;
		}
		else 
		if (lastcard2.card == evt.card.w)
		{
			lastcard2.sec = tsec;
			return 0;
		}
		else 
		if ((tsec - lastcard2.sec) < buddytime)
		{
			tsec = evt.card.w;		//save current card
			evt.card.w = lastcard2.card;
			evt.evt = E2_ACC;
			store_event();
			evt.card.w = tsec;
			lastcard2.card = 0x00000000;
			return 1;
		}
		else
		{
			lastcard2.card = 0x00000000;
			lastcard2.sec = tsec;
			return 0;
		}
	}
	return 0;				
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void loadfw(void)
{
unsigned char x, bf[4];

	x = read_ext_eeprom(MEM, PASSH);
	if (x == rxdat[3])
	{
		x = read_ext_eeprom(MEM, PASSL);
		if (x == rxdat[4])
		{
			output_high(txon);
			delay_us(50);
			putchar(0x06);
			while (!TRMT) restart_wdt(); 
			delay_us(50);
			output_low(txon);
			bf[0] = 0x00;
			bf[1] = node;
			write_program_memory(0x200000, bf, 2);
			reset_cpu();
		}
	}
	output_high(txon);
	delay_us(50);
	putchar(0x15);
	while (!TRMT) restart_wdt(); 
	delay_us(50);
	output_low(txon);

}							
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void ToggleDoor(unsigned char dr)
{
	LEDcnt = 1;						//begin LEDflash with LED on
	LEDstop = LEDcnt + 5;			//run through LEDflash so 2 flashes
	if (dr == 1)					//if door 1
	{
		LEDflash = 1;
		if (dr1stat == DOOR_SOP)
		{
			output_low(relay1);		//relay1 off
			LEDcnt = 2;				//begin LEDflash with LED off
			dr1stat = DOOR_NO;		//set normal
			evt.evt = E1_LATC;
		}
		else 
		if (dr1stat != DOOR_TZOP)
		{
			dr1stat = DOOR_SOP;		//set open
			output_high(relay1);	//relay1 on
			evt.evt = E1_LATO;
		}
	}
	else							//assume door 2
	{
		LEDflash = 2;
		if (dr2stat == DOOR_SOP)
		{
			output_low(relay2);		//relay2 off
			LEDcnt = 2;				//begin LEDflash with LED off
			dr2stat = DOOR_NO;		//set normal
			evt.evt = E2_LATC;
		}
		else 
		if (dr2stat != DOOR_TZOP)
		{
			dr2stat = DOOR_SOP;		//set open
			output_high(relay2);	//relay2 on
			evt.evt = E2_LATO;
		}
	}
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void rcount(void)					//Send relay counts to host
{
	int32 rnum;
	char cs, x, cv, rc;
	char bf[12];

	for (rc=0; rc<2; rc++)
	{
		if (rc == 0) rnum = r1count;
		else rnum = r2count;

		bf[0] = (0x80  |node);
		bf[1] = 0x0a;
		bf[2] = 0xf2;
		bf[3] = rc;
		for (x=4; x<12; x++)
		{
			cv = ((rnum >> (44 - (x*4))) & 0x0f);
			if (cv < 0x0A) bf[x] = (0x30 + cv);
			else bf[x] = (0x37 + cv);
		}
		delay_ms(1);
		output_high(txon);				//RS485 = TX
		delay_ms(TXDEL);
		cs = 0;
		for (x=0; x<12; x++)
		{
			putchar(bf[x]);
			cs += bf[x];
		}
		putchar(cs);
		while (!TRMT) restart_wdt();
		output_low(txon);				//RS485 = RX
	}
}
//---------------------------------------------------------------------
void rcountsave (char relay)
{
	char x, rc1;
	int32 rcmem;
	int16 ract;
	if (relay == 0)	for (x = 0; x < 6; x++)	write_ext_eeprom(MEM, (R1ACT + x), 0x00);	//clear stored counts
	else 
	{
		rcmem = r1count;				//Allocate relay 1 values
		ract = R1ACT;
		if (relay == 2)					//change values if relay 2
		{
			rcmem = r2count;
			ract = R2ACT;
		}
		for (x = 0; x < 3; x++)			//write counts to mem
		{
			rc1 = rcmem & 0xff;
			write_ext_eeprom(MEM, (ract + x), rc1);
			rcmem >>= 8;
		}
	}	
}
//--------------------------------------------------------------------- 
//check_pin() 
//Checks 4 digit pin against up to 10 stored pins 
//--------------------------------------------------------------------- 
//Process pin number 
short check_pin(char *ps, char rdr) 
{ 
	long xpin, pin_al, addr, tz;
	char n, y, pl, ph, pinvalid, *pn;
	read_rtc();									//get current time
	evt.card.w = atol(ps);
	if (evt.card.w == 0) return 0;
	xpin = PIN1_L;
	pinvalid = 0;
	for (n = 0; n < 10; n++)
	{
		pn = &evt.card.w;
		y = read_ext_eeprom(MEM, xpin++);
		if (y == *pn++)
		{
			y = read_ext_eeprom(MEM, xpin++);
			pinvalid = 2;
			if (y == *pn)
			{
				pin_al = read_ext_eeprom(MEM, (xpin + (18 - n)));
				if (rdr == 1) addr = ACC0 + (ACCSIZE * pin_al); 				//locate accesslevel 
				else if (rdr == 2) addr = ACC0 + (ACCSIZE * pin_al) + 2; 
				ph = read_ext_eeprom(MEM, addr++);								//high byte 
				pl = read_ext_eeprom(MEM, addr);								//low byte 
				if ((pl == 0) && (ph == 0)) return pinvalid;					//no timezone 
				if (bit_test(pl, 0)) pinvalid = 1;								//tz always = bit 0 
				if (pinvalid == 2)												//tz is 2 or greater
				{
					tz = TZ2;													//point to tz2 
					for (n = 1; n < 8; n++)										//1st 7 user timezones 
					{ 
						if ((bit_test(pl, n)) && (tz_check(tz))) pinvalid = 1;	//timezone valid 
						tz += TZSIZE;											//point to next tz 
					} 
					for (n = 0; n < 8; n++)										//next 8 timezones 
					{ 
						if ((bit_test(ph, n)) && (tz_check(tz))) pinvalid = 1;	//timezone valid 
						tz += TZSIZE;											//point to next tz 
					} 
				}
			}
		}
		else ++xpin;
	}
	return pinvalid;
} 
