//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 19.04.2020.
//  Copyright __MyCompanyName__ 2020. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

//#include "twislave.c"
#include "lcd.c"

#include "adc.c"
#include "defines.h"
#include "irmp.c"

//***********************************
//Buero							*
#define SLAVE_ADRESSE 0x62 //		*
//									*
//***********************************

extern IRMP_DATA   irmp_data;

#define toggleA PORTC ^= (1<<PC4)
#define toggleB PORTC ^= (1<<PC5)

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4 // PORT C
#define SCLPIN		5



#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define WDTBIT			7


#define BUEROPORT	PORTD		// Ausgang fuer BUERO
#define UHRPIN 0

#define SERVOPORT	PORTD		// Ausgang fuer Servo
#define SERVODDR  DDRD
#define SERVOPIN0 7				// Impuls fŸr Servo
#define SERVOPIN1 6				// Enable fuer Servo, Active H
#define SERVOCONTROLPIN 5 // LED


// Definitionen Slave Buero
#define UHREIN 0
#define UHRAUS 1


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD

// Define fuer Slave:
#define LOOPLED			7




#define TASTATURPIN		3
#define POTPIN			0
#define BUZZERPIN		0

#define INNEN			0	//	Byte fuer INNENtemperatur
#define AUSSEN			1	//	Byte fuer Aussentemperatur
#define STATUS			3	//	Byte fuer Status
#define BRENNERPIN		2	//	PIN 2 von PORT B als Eingang fuer Brennerstatus

#define US (1000000 / F_INTERRUPTS)

uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);


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
volatile uint8_t twi=0;
uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

// IRMP
//void timer1_init (void);
volatile uint16_t audio_remote_command = 0;

volatile uint8_t timer1count=0;

extern volatile uint8_t protokoll;
extern volatile uint8_t adresse;
extern volatile uint8_t code;
extern volatile uint16_t irmpcontrolD;

volatile uint8_t inputstatus = 0; // Eingaenge am ADC abfragen

uint16_t kanaldelayA = KANALDELAY;
uint16_t kanaldelayB = KANALDELAY;
uint16_t kanaldelayC= KANALDELAY;
uint16_t kanaldelayD = KANALDELAY;

uint16_t outputdelay = OUTPUTDELAY;

uint16_t timer1counter = 0;
uint16_t sekundencounter = 0;
/* evaluate an IR remote signal */
void audio_remote(uint8_t command)
{
   if (   irmp_data.protocol != IRMP_NEC_PROTOCOL
       || irmp_data.address  != MY_REMOTE) {
      return;
   }
   uint16_t cmd = irmp_data.command;
   //audio_remote_command = cmd;
   switch (command)
   {
         
   }
   switch (irmp_data.command) 
   {
         
      case KEY_OPERATE: /* on/off */
      {
         //remotecommand = KEY_OPERATE;
         break;
      }
         
      case KEY_1:
      {
         break;
      }
         
      case KEY_2:
      {
         break;
      }
         
      case KEY_FWD:
      {
          
         break;
      }
         
      case KEY_REV:
      {
         break;
      }
         
      default:
      {
         break;
      }
   }
}



static void
timer1_init (void)
{
#if defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__)                // ATtiny45 / ATtiny85:
   
#if F_CPU >= 16000000L
   OCR1C   =  (F_CPU / F_INTERRUPTS / 8) - 1;                              // compare value: 1/15000 of CPU frequency, presc = 8
   TCCR1   = (1 << CTC1) | (1 << CS12);                                    // switch CTC Mode on, set prescaler to 8
#else
   OCR1C   =  (F_CPU / F_INTERRUPTS / 4) - 1;                              // compare value: 1/15000 of CPU frequency, presc = 4
   TCCR1   = (1 << CTC1) | (1 << CS11) | (1 << CS10);                      // switch CTC Mode on, set prescaler to 4
#endif
   
#else                                                                       // ATmegaXX:
   OCR1A   =  (F_CPU / F_INTERRUPTS) - 1;                                  // compare value: 1/15000 of CPU frequency
   TCCR1B  = (1 << WGM12) | (1 << CS10);                                   // switch CTC Mode on, set prescaler to 1
#endif
   
#ifdef TIMSK1
   TIMSK1  = 1 << OCIE1A;                                                  // OCIE1A: Interrupt by timer compare
#else
   TIMSK   = 1 << OCIE1A;                                                  // OCIE1A: Interrupt by timer compare
#endif
}

ISR(TIMER1_COMPA_vect)                                                             // Timer1 output compare A interrupt service routine, called every 1/15000 sec
{
   //PORTC ^= (1<<PC5); 
   (void) irmp_ISR();
   //audio_remote();                                                        // call irmp ISR
   // call other timer interrupt routines...
   inputstatus = PINC & 0x0F;
   
   timer1counter++;
   if (timer1counter >= F_INTERRUPTS)
   {
      timer1counter = 0;
      sekundencounter++;
      toggleA;
   }
 
}

void slaveinit(void)
{
	LOOPLEDDDR |= (1<<LOOPLED);		//Pin z von PORT D als Ausgang fuer loop-LED
//	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr IR
//	PORTB |= (1<<PB1);	//Pull-up

   DDRC |= (1<<PC4);   //Ausgang fuer control A
   PORTC |= (1<<PC4);   //Pull-up

   DDRC |= (1<<PC5);   //Ausgang fuer control B
   PORTC |= (1<<PC5);   //Pull-up

   DDRB |= (1<<PB0);   //Ausgang fuer Verstaerker
   PORTB &= ~(1<<PB0);   //LO

   DDRC &= ~(1<<AUDIO_A);  // Eingang von Audioquellen
   PORTC &= ~(1<<AUDIO_A); // LO
   DDRC &= ~(1<<AUDIO_B);
   PORTC &= ~(1<<AUDIO_B); // LO
   DDRC &= ~(1<<AUDIO_C);
   PORTC &= ~(1<<AUDIO_C); // LO
   DDRC &= ~(1<<AUDIO_D);
   PORTC &= ~(1<<AUDIO_D); // LO
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

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



void main (void) 
{
   /*
    in Start-loop in while
    init_twi_slave (SLAVE_ADRESSE);
    sei();
    */
   // IRMP
   IRMP_DATA   irmp_data;
   
   irmp_init();                                                            // initialize IRMP
   timer1_init();                                                          // initialize timer1

   
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
   lcd_puts("AVR_Audio");
   
   
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
   
   sei ();                                                                 // enable interrupts
   
    while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0xFFFF)
      {
         
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         TastaturCount++;
         //lcd_gotoxy(13,1);
         //lcd_putint(TastaturCount);
         //lcd_gotoxy(17,1);
         //lcd_putint(timer1count);
         lcd_gotoxy(0,3);
         lcd_putint(protokoll);
         protokoll=0;
         lcd_putc(' ');
         lcd_putint(adresse);
         adresse=0;
         lcd_putc(' ');
         lcd_putint(code);
         code=0;
         lcd_putc(' ');
         lcd_putint12(irmpcontrolD);
         irmpcontrolD=0;
         lcd_putc(' ');
         lcd_gotoxy(0,2);
         lcd_puthex(inputstatus);
         lcd_putc(' ');
         lcd_putint12(outputdelay);
         if (inputstatus > 0)
         {
            outputdelay = OUTPUTDELAY;
            //PORTB |= (1<<PB0);
         }
         else
         {
            if (outputdelay)
            {
               outputdelay--;
            }
            
            //PORTB &= ~(1<<PB0);
         }
         
         if (outputdelay)
         {
            PORTB |= (1<<PB0); // ON
         }
         else
         {
            PORTB &= ~(1<<PB0); // OFF
         }
         /*
          */
      }
      
      PORTB &= ~(1<<PB1); 
      
      // IRMP
      if (irmp_get_data (&irmp_data))
      {
         /*
         if (   irmp_data.protocol != IRMP_NEC_PROTOCOL
             || irmp_data.address  != MY_REMOTE) 
         {
            
            return;
         }
          */
         // got an IR message, do something
         _delay_ms(25);
         
    //     PORTB |= (1<<PB0);
         protokoll = irmp_data.protocol;
         adresse = irmp_data.address;
         code = irmp_data.command;
         _delay_ms(25);
 
 
      }
       
        
        
      
      
   }     
         
      
      
      
 
   // return 0;
}
