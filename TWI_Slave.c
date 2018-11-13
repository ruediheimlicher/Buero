//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "twislave.c"
#include "lcd.c"

#include "adc.c"

//***********************************
//Buero							*
#define SLAVE_ADRESSE 0x62 //		*
//									*
//***********************************

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4 // PORT C
#define SCLPIN		5

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4

#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define WDTBIT			7


#define BUEROPORT	PORTD		// Ausgang fuer BUERO
#define UHRPIN 0

#define SERVOPORT	PORTD		// Ausgang fuer Servo
#define SERVODDR  DDRD
#define SERVOPIN0 7				// Impuls für Servo
#define SERVOPIN1 6				// Enable fuer Servo, Active H
#define SERVOCONTROLPIN 5 // LED

// Definitionen fuer mySlave PORTD
//#define UHREIN 4
//#define UHRAUS 5

// Definitionen Slave Buero
#define UHREIN 0
#define UHRAUS 1


#define LOOPLEDPORT		PORTD

// Define fuer Slave:
#define LOOPLED			4


// Define fuer mySlave PORTD:
//#define LOOPLED			2
//#define TWILED			7


#define TASTE1		38
#define TASTE2		46
#define TASTE3		54
#define TASTE4		72
#define TASTE5		95
#define TASTE6		115
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		225
#define TASTE0		235
#define TASTER		245
#define TASTATURPORT PORTC

#define TASTATURPIN		3
#define POTPIN			0
#define BUZZERPIN		0

#define INNEN			0	//	Byte fuer INNENtemperatur
#define AUSSEN			1	//	Byte fuer Aussentemperatur
#define STATUS			3	//	Byte fuer Status
#define BRENNERPIN		2	//	PIN 2 von PORT B als Eingang fuer Brennerstatus

uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);

volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

uint8_t buerostatus=0x00;

volatile uint16_t Servotakt=20;					//	Abstand der Impulspakete
volatile uint16_t Servopause=0x00;				//	Zaehler fuer Pause
volatile uint16_t Servoimpuls=0x00;				//	Zaehler fuer Impuls
volatile uint8_t Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
volatile uint8_t ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
volatile uint8_t ServoimpulsdauerSpeicher=0;	//	Speicher  fuer Servoimpulsdauer
volatile uint8_t Potwert=45;
volatile uint8_t TWI_Pause=1;
volatile uint8_t ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
uint8_t ServoimpulsNullpunkt=23;
uint8_t ServoimpulsSchrittweite=10;
uint8_t Servoposition[]={23,33,42,50,60};
volatile uint16_t ADCImpuls=0;

volatile uint8_t twicount=0;
volatile uint8_t readcount=0;
volatile uint8_t twir=0;
volatile uint8_t twiw=0;
uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset



uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void slaveinit(void)
{
 	DDRD |= (1<<DDD0);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<DDD1);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<DDD2);		//Pin 2 von PORT D als Ausgang fuer Buzzer
 	DDRD |= (1<<DDD3);		//Pin 3 von PORT D als Ausgang fuer LED TWI
	DDRD |= (1<<LOOPLED);		//Pin 4 von PORT D als Ausgang fuer loop-LED
	SERVODDR |= (1<<SERVOCONTROLPIN);		//Pin 5 von PORT D als Ausgang fuer LED Servo
 	SERVODDR |= (1<<SERVOPIN1);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	SERVODDR |= (1<<SERVOPIN0);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	
	
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang für Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang für Taste 2
	PORTB |= (1<<PB1);	//Pull-up
	

	//LCD
	DDRB |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	DDRB |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	DDRB |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang für SDA
	TWI_PORT |= (1<<SDAPIN); // HI
	
	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang für SCL
	TWI_PORT |= (1<<SCLPIN); // HI

	SlaveStatus=0;
	SlaveStatus |= (1<<TWI_WAIT_BIT);

	
	DDRC &= ~(1<<INNEN);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<AUSSEN);	//Pin 1 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up


	
	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//Timer fuer Servo	
	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
	
}

void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 

ISR(TIMER0_OVF_vect) 
{ 
	ADCImpuls++;
	Servopause++;
	//lcd_clr_line(1);

	//lcd_gotoxy(10,1);
	//lcd_puts("Tim\0");
	//delay_ms(400);
	//lcd_cls();
	//lcd_clr_line(0);
	//lcd_gotoxy(0,1);
	//lcd_puts("Stop Servo\0");
	//lcd_puts(" TP\0");
	//lcd_putint1(TWI_Pause);
	//	Intervall 64 us, Overflow nach 16.3 ms
	if (Servopause==3)	// Neues Impulspaket nach 48.9 ms
	{

		if (TWI_Pause)
		{
//			lcd_gotoxy(19,0);
//			lcd_putc(' ');
			timer2(Servoimpulsdauer);	 // setzt die Impulsdauer
			if (SERVOPORT &  (1<<SERVOPIN1)) // Servo ist ON
			{
				SERVOPORT |= (1<<SERVOPIN0); // Schaltet Impuls an SERVOPIN0 ein
			}
			SERVOPORT |= (1<<SERVOCONTROLPIN);// Kontrolle auf PIN D5 ON
		}
		Servopause=0;
	}
}


ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR2=0;
		SERVOPORT &= ~(1<<SERVOPIN0);//	SERVOPIN0 zuruecksetzen
		SERVOPORT &= ~(1<<SERVOCONTROLPIN);// Kontrolle auf PIN D5 OFF
		//delay_ms(800);
		//lcd_clr_line(1);
		
}




void main (void) 
{
   /*
    in Start-loop in while
    init_twi_slave (SLAVE_ADRESSE);
    sei();
    */
   
   wdt_disable();
   MCUSR &= ~(1<<WDRF);
   wdt_reset();
   WDTCR |= (1<<WDCE) | (1<<WDE);
   WDTCR = 0x00;
   
   slaveinit();
   //PORT2 |=(1<<PC4);
   //PORTC |=(1<<PC5);
   
   //uint16_t ADC_Wert= readKanal(0);
   
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   lcd_puts("BUERO 62\0");
   
   BUEROPORT &= ~(1<<UHREIN);//	UHREIN sicher low
   BUEROPORT &= ~(1<<UHRAUS);//	UHRAus sicher low
   BUEROPORT |= (1<<UHRAUS);
   delay_ms(10);
   BUEROPORT &= ~(1<<UHRAUS);
   
   uint8_t Tastenwert=0;
   uint8_t TastaturCount=0;
   uint8_t Servowert=0;
   uint8_t Servorichtung=1;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x01F;
   uint8_t Schalterposition=0;
   //timer0();
   
   //initADC(TASTATURPIN);
   //wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint16_t startdelay0=0x01FF;
   //uint16_t startdelay1=0;
   
   uint16_t twi_LO_count0=0;
   uint16_t twi_LO_count1=0;
   
   
   //uint8_t twierrcount=0;
   LOOPLEDPORT |=(1<<LOOPLED);
   
   delay_ms(800);
   //eeprom_write_byte(&WDT_ErrCount0,0);
   uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0);
   //	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
   uint16_t twi_HI_count0=0;
   
   if (eepromWDT_Count0==0xFF)
   {
      eepromWDT_Count0=0;
      
   }
   
   /*
    Bit 0: 1 wenn wdt ausgelöst wurde
    
    */
   while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0xFFFF)
      {
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         
      }
      
      
      
      /**	Beginn Startroutinen	***********************/
      // wenn Startbedingung vom Master:  TWI_slave initiieren
      if (SlaveStatus & (1<<TWI_WAIT_BIT))
      {
         if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
         {
            init_twi_slave (SLAVE_ADRESSE);
            sei();
            SlaveStatus &= ~(1<<TWI_WAIT_BIT);
            SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
            
            // StartDelayBit zuruecksetzen
            
         }
      }
      
      
      /**	Ende Startroutinen	***********************/
      
      
      /* **** rx_buffer abfragen **************** */
      //rxdata=0;
      
      
      //	Schaltuhr
      
      
      if ((SlaveStatus & (1<<TWI_OK_BIT)) && (rxdata))
      {
         twicount++;
         lcd_cls();
         lcd_gotoxy(0,0);
         lcd_puthex(twicount);
         
         /*
          lcd_gotoxy(4,0);
          lcd_puthex(twir);
         //twir=0;
         lcd_putc(' ');
         lcd_puthex(twiw);
         lcd_putc(' ');
         */
         //twiw=0;
         /*
          BueroTXdaten[1] = Zeit.stunde;
          BueroTXdaten[2] = Zeit.minute;
          BueroTXdaten[3] = Stundencode;
          BueroTXdaten[4] = Stundencode;
          */
         /*
         lcd_gotoxy(0,1);
         lcd_puthex(rxbuffer[0]);
         lcd_puthex(rxbuffer[1]);
         lcd_puthex(rxbuffer[2]);
*/
         lcd_gotoxy(6,1);
         //lcd_puthex(rxbuffer[3]);
         //lcd_puthex(rxbuffer[4]);
         lcd_puthex(rxbuffer[5]);
         lcd_puthex(rxbuffer[6]);
         lcd_puthex(rxbuffer[7]);
         //lcd_puthex(readcount);
         //readcount=0;
          
          
         
         {
            //
            
            if (rxbuffer[3] < 6)
               
            {
               
               if (Servorichtung) // vorwärts
               {
                  Servowert++;
                  if (Servowert==4)
                  {
                     Servorichtung=0;
                  }
                  
               }
               else
               {
                  
                  Servowert--;
                  if (Servowert==0)
                  {
                     Servorichtung=1;
                  }
                  
                  
                  
                  
               }
               /*
                lcd_gotoxy(1,10);
                lcd_puts("R:\0");
                lcd_putint2(Servorichtung);
                lcd_puts(" W:\0");
                lcd_putint2(Servowert);
               */
               
               
               Servowert=rxbuffer[3];
               
               ServoimpulsdauerPuffer=Servoposition[Servowert];
               
            }
            
         }
         
         //RingD2(2);
         //delay_ms(20);
         
         buerostatus=rxbuffer[0];
 //        lcd_gotoxy(0,1);
         lcd_gotoxy(0,1);
         //cli();
         lcd_puts("St");
         lcd_puthex(buerostatus);
         //sei();
         //delay_ms(1000);
         if ( buerostatus  & (1<<UHRPIN)) // 0
         {
            //delay_ms(1000);
            //Schaltuhr ein
            //cli();
            lcd_gotoxy(19,1);
            lcd_putc('1');
            //sei();
            BUEROPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
            BUEROPORT &= ~(1<<UHREIN);//	UHREIN sicher low
            BUEROPORT |= (1<<UHREIN);
            delay_ms(20);
            BUEROPORT &= ~(1<<UHREIN);
         }
         else
         {
            //delay_ms(1000);
            //Schaltuhr aus
            //cli();
            lcd_gotoxy(19,1);
            lcd_putc('0');
            //sei();
            BUEROPORT &= ~(1<<UHREIN);//	UHREIN sicher low
            BUEROPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
            BUEROPORT |= (1<<UHRAUS);
            delay_ms(20);
            BUEROPORT &= ~(1<<UHRAUS);
         }
         
         
         // tx_buffer laden
         
         // Temperatur lesen
         initADC(AUSSEN);
         uint16_t temperaturBuffer=(readKanal(AUSSEN));
         lcd_gotoxy(10,0);
         lcd_puts("A");
         //lcd_putint12(temperaturBuffer);
         
         // neues Thermometer
         
         //lcd_puts("A0+\0");
         lcd_put_tempbis99(temperaturBuffer>>1);//Doppelte Auflösung
         txbuffer[AUSSEN]=(temperaturBuffer>>1);
         //txbuffer[AUSSEN]=0x37;
         //	initADC(RUECKLAUF);
         
         //	PIN B4 abfragen
         txbuffer[4]=(PINB & (1<< 4));
         txbuffer[5]=77;
         txbuffer[6]=87;
         txbuffer[7]=98;
         
         rxdata=0;
         
         
      }
      
      
      
      if (!(PINB & (1<<PB0))) // Taste 0
      {
         //lcd_gotoxy(12,1);
         //lcd_puts("P0 Down\0");
         
         if (! (TastenStatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
         {
            //RingD2(5);
            TastenStatus |= (1<<PB0);
            Tastencount=0;
            //lcd_gotoxy(0,1);
            //lcd_puts("P0 \0");
            //lcd_putint(TastenStatus);
            //delay_ms(800);
         }
         else
         {
            
            
            Tastencount ++;
            //lcd_gotoxy(7,1);
            //lcd_puts("TC \0");
            //lcd_putint(Tastencount);
            
            if (Tastencount >= Tastenprellen)
            {
               if (Servowert<4)
               {
                  Servowert++;
                  Servoimpulsdauer=Servoposition[Servowert];
                  
               }
               /*
                if (Servoimpulsdauer<61)
                {
                Servoimpulsdauer++;
                SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
                lcd_gotoxy(0,1);
                //lcd_puts("P0 down  \0");
                lcd_putint2(Servoimpulsdauer);
                
                }
                */
               Tastencount=0;
               TastenStatus &= ~(1<<PB0);
            }
         }//else
         
      }	// Taste 0
      
      
      if (!(PINB & (1<<PB1))) // Taste 1
      {
         //lcd_gotoxy(12,1);
         //lcd_puts("P1 Down\0");
         
         if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
         {
            TastenStatus |= (1<<PB1);
            Tastencount=0;
            //lcd_gotoxy(3,1);
            //lcd_puts("P1 \0");
            //lcd_putint(Servoimpulsdauer);
            //delay_ms(800);
            
         }
         else
         {
            //lcd_gotoxy(3,1);
            //lcd_puts("       \0");
            
            Tastencount ++;
            if (Tastencount >= Tastenprellen)
            {
               
               if (Servowert > 0)
               {
                  Servowert--;
                  Servoimpulsdauer=Servoposition[Servowert];
                  
               }
               
               
               if (Servoimpulsdauer>19)
               {
                  Servoimpulsdauer--;
                  SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
                  
                  lcd_gotoxy(0,1);
                  lcd_putint2(Servoimpulsdauer);
               }
               Tastencount=0;
               TastenStatus &= ~(1<<PB1);
            }
         }//	else
         
      } // Taste 1
      
      /* ******************** */
      //		initADC(TASTATURPIN);
      //		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      
      Tastenwert=0;
      
      //lcd_gotoxy(3,1);
      //lcd_putint(Tastenwert);
      
      if (Tastenwert>23)
      {
         /*
          0: 
          1: 
          2: 
          3: 
          4: 
          5: 
          6: 
          7: 
          8: 
          9: 
          */
         
         TastaturCount++;
         if (TastaturCount>=50)
         {
            
            //lcd_clr_line(1);
            //				lcd_gotoxy(8,1);
            //				lcd_puts("T:\0");
            //				lcd_putint(Tastenwert);
            
            uint8_t Taste=Tastenwahl(Tastenwert);
            
            lcd_gotoxy(18,1);
            lcd_putint2(Taste);
            //delay_ms(600);
            // lcd_clr_line(1);
            
            
            TastaturCount=0;
            Tastenwert=0x00;
            uint8_t i=0;
            //uint8_t pos=0;
            
            switch (Taste)
            {
               case 0://
               { 
                  
               }break;
                  
               case 1://
               { 
               }break;
                  
               case 2://
               { 
                  
               }break;
                  
               case 3://
               { 
                  
               }break;
                  
               case 4://
               { 
                  if (Schalterposition)
                  {
                     Schalterposition--;
                     Servoimpulsdauer=Servoposition[Schalterposition];
                  }
                  
               }break;
                  
               case 5://
               { 
                  
                  Schalterposition=0;
                  Servoimpulsdauer=Servoposition[Schalterposition];
                  
               }break;
                  
               case 6://
               { 
                  if (Schalterposition<4)
                  {
                     Schalterposition++;
                     Servoimpulsdauer=Servoposition[Schalterposition];
                  }
               }break;
                  
               case 7://
               { 
                  if (Servoimpulsdauer>Servoposition[0])
                  {
                     Servoimpulsdauer--;
                     lcd_gotoxy(0,16);
                     lcd_putint2(Servoimpulsdauer);
                  }
                  
               }break;
                  
               case 8://
               { 
                  
               }break;
                  
               case 9://
               { 
                  if (Servoimpulsdauer<Servoposition[4])
                  {
                     Servoimpulsdauer++;
                     lcd_gotoxy(0,2);
                     lcd_putint2(Servoimpulsdauer);
                  }
               }break;
                  
                  
            }//switch Tastatur
            SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
         }//if TastaturCount	
         
      }//	if Tastenwert
      
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while
   
   
   // return 0;
}
