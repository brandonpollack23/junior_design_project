/*
 * FinalProject.c
 *
 * Created: 3/31/2014 6:55:12 PM
 *  Author: Brandon
 */ 
#define F_CPU 8e6
#define Vref 1.1

#define LCDLINE1 0x00
#define LCDLINE2 0x40

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "my_def.h"
#include "boot_sound.h"

//note on set up order
//0 SPI and TC0 for boot sound
//when that is done do this stuff here
//1 LCD
//2 ADC
//3 TC1 for IC, be sure to set ICIE1 in TIMSK1
//4 Comp


//TODO check stuff, make sure SPI xck is 4 Mhz


int main(void)
{
	CLKPR = 1<<CLKPCE;
	CLKPR &= 1<<CLKPCE; //switch to 8 Mhz
	
	IO_init();
	SPI_init();
	TC_DAC_init();
	
	sei();
	
	while(doneBooting != 1) asm("nop"); //play the boot sound
	
	cli();
	
	TCCR0B = 0; //turn off timer 0
	
	LCD_init();
	ADC_init();
	TC_IC_init();
	COMP_init();
	
	ADCSRA |= (1 << ADSC); //start an ADC conversion
	
	sei();
	
	//static char currentNote[3] = { 0, 0 ,0 };
	
    while(1)
    {
        //static char sharp;
		//sharp = findNote(freq,currentNote);
		
		static char buffer[16];
		
		static double period;
		period = (ICval - lastCapture)*pow(1e6,-1); //each clock is 1/8 * 10e-6 seconds
		
		lastCapture = ICval;
		
		freq = (freq + (pow(period,-1)*100))/2; //average of last 2 frequencies, hopefully this smooths things, for some reason i have 10 hz error, fix that
		
		//snprintf(buffer, 16, "%"PRIu16"e-2Hz %s %c", freq, currentNote, sharp); //0 means flat, 1 sharp, 2 is in tune
		snprintf(buffer, 16, "%"PRIu16"e-2Hz", freq);
		
		/*if(sharp == '2')
		{
			IOPORT |= (1 << PIND7);
		}
		else
		{
			IOPORT = 0;
		}*/
		
		//ADCSRA ^= (1 << ADIE); //changes address so we have to stop it
		
		lcd_cmd(0x80); //go back to the first address
		printLCD(buffer); //print the frequency		
		lcd_cmd(0xC0); //set ddram address to second line		
		printLCD(line2buffer); //print ADC value
		//ADCSRA ^= (1 << ADIE); //turn it back on
    }
}

ISR(ADC_vect) //calculate battery level, output it to second line of LCD
{
	cli();
	/*uint16_t ADCval = getADCVal();
	
	double ADCin = ADCval*Vref/1023; //voltage input after divider
	
	double Vcc = ADCin*8.1; //VCC is my battery level, output that to lcd
	
	uint16_t temp = Vcc;
	
	Vcc = Vcc - temp;
	Vcc *= 100;
	uint16_t Vcc_dec = (uint16_t) Vcc;	*/
	
	uint16_t ADCval = getADCVal();
	
	double Vin = ((double)ADCval*9)/1023;
	
	uint16_t whole = Vin;
	
	double dec = Vin - whole;
	
	uint16_t deci = trunc(dec*1000);
	
	sprintf(line2buffer,"Battery: %"PRIu16".%"PRIu16" V",whole,deci); //format vcc to a string with 3 decimal places
	
	sei();
}

ISR(TIMER1_CAPT_vect) //subtract this with prevous cap value and use it to find period, inverse for freq, output to display, look up closest note and output that too and direction to be more in tune
{  //NOTE THIS MAY HAVE TO BE COMP INT VECTOR, WHICH MEANS THAT NEEDS TO BE ENABLED IN INIT AS WELL
	cli();
	//remember this clock is prescaled by 8, so each clock is 1e-6 seconds
	lastCapture = ICval;
	
	ICval = ICR1;
	
	sei();
}

ISR(TIMER0_COMPA_vect) //send current dac value of counter, if at last value, turn off TC0 and finish set up of other stuff
{
	IOPORT ^= (1 << PIND7); //50 percent duty cycle led during boot
	uint16_t word = pgm_read_word(&boot_sound[sample_count++]);
	if(sample_count >= ARRAYLENGTH) doneBooting = 1;
	send_DAC_value(word);
}

ISR(TIMER1_OVF_vect)
{
	static uint16_t ADC_count = 0;
	ADC_count = (ADC_count + 1)%1000;  //every  seconds get a new ADC value
	if(ADC_count == 0) ADCSRA |= (1 << ADSC); //start an ADC conversion
}

uint8_t findNote(uint16_t freq, char currentNote[])
{
	//look up tables for stupid notes
	if(freq >= C3 - (C3 - 12347)/2 && freq < C3 + (D3 - C3)/2)
	{
		strcpy(currentNote,"C3");
		if(freq < C3 - .025*C3) return 'f'; //not within 10%
		else if(freq > C3 + .025*C3) return '#';
		else return '2'; //in tune
	}
	else if(freq >= D3 - (D3 - C3)/2 && freq < D3 + (E3 - D3)/2)
	{
		strcpy(currentNote,"D3");
		if(freq < D3 - .025 * D3) return 'f';
		else if(freq > D3 + .025 * D3) return '#';
		else return '2';
	}
	else if(freq >= E3 - (E3 - D3)/2 && freq < E3 + (F3 - E3)/2)
	{
		strcpy(currentNote,"E3");
		if(freq < E3 - .025 * E3) return 'f';
		else if(freq > E3 + .025 * E3) return '#';
		else return '2';
	}
	else if(freq >= F3 - (F3 - E3)/2 && freq < F3 + (G3 - F3)/2)
	{
		strcpy(currentNote,"F3");
		if(freq < F3 - .025 * F3) return 'f';
		else if(freq > F3 + .025 * F3) return '#';
		else return '2';
	}
	else if(freq >= G3 - (G3 - F3)/2 && freq < G3 + (A3 - G3)/2)
	{
		strcpy(currentNote,"G3");
		if(freq < G3 - .025 * G3) return 'f';
		else if(freq > G3 + .025 * G3) return '#';
		else return '2';
	}
	else if(freq >= A3 - (A3 - G3)/2 && freq < A3 + (B3 - A3)/2)
	{
		strcpy(currentNote,"A3");
		if(freq < A3 - .025 * A3) return 'f';
		else if(freq > A3 + .025 * A3) return '#';
		else return '2';
	}
	else if(freq >= B3 - (B3 - A3)/2 && freq < B3 + (C4 - B3)/2)
	{
		strcpy(currentNote,"B3");
		if(freq < B3 - .025 * B3) return 'f';
		else if(freq > B3 + .025 * B3) return '#';
		else return '2';
	}
	else if(freq >= C4 - (C4 - B3)/2 && freq < C4 + (D4 - C4)/2)
	{
		strcpy(currentNote,"C4");
		if(freq < C4 - .025 * C4) return 'f';
		else if(freq > C4 + .025 * C4) return '#';
		else return '2';
	}
	else if(freq >= D4 - (D4 - C4)/2 && freq < D4 + (E4 - D4)/2)
	{
		strcpy(currentNote,"D4");
		if(freq < D4 - .025 * D4) return 'f';
		else if(freq > D4 + .025 * D4) return '#';
		else return '2';
	}
	else if(freq >= E4 - (E4 - D4)/2 && freq < E4 + (F4 - E4)/2)
	{
		strcpy(currentNote,"E4");
		if(freq < E4 - .025 * E4) return 'f';
		else if(freq > E4 + .025 * E4) return '#';
		else return '2';
	}
	else if(freq >= F4 - (F4 - E4)/2 && freq < F4 + (G4 - F4)/2)
	{
		strcpy(currentNote,"F4");
		if(freq < F4 - .025 * F4) return 'f';
		else if(freq > F4 + .025 * F4) return '#';
		else return '2';
	}
	else if(freq >= G4 - (G4 - F4)/2 && freq < G4 + (A4 - G4)/2)
	{
		strcpy(currentNote,"G4");
		if(freq < G4 - .025 * G4) return 'f';
		else if(freq > G4 + .025 * G4) return '#';
		else return '2';
	}
	else if(freq >= A4 - (A4 - G4)/2 && freq < A4 + (B4 - A4)/2)
	{
		strcpy(currentNote,"A4");
		if(freq < A4 - .025 * A4) return 'f';
		else if(freq > A4 + .025 * A4) return '#';
		else return '2';
	}
	else if(freq >= B4 - (B4 - A4)/2 && freq < B4 + (C4 - B4)/2)
	{
		strcpy(currentNote,"B4");
		if(freq < B4 - .025 * B4) return 'f';
		else if(freq > B4 + .025 * B4) return '#';
		else return '2';
	}
	else if(freq >= C5 - (C5 - B4)/2 && freq < C5 + (C5 - 58733)/2)
	{
		strcpy(currentNote,"C5");
		if(freq < C5 - .025 * C5) return 'f';
		else if(freq > B4 + .025 * C5) return '#';
		else return '2';
	}
	else
	{
		strcpy(currentNote,"00"); //out of range
		return '0';
	}
}

void ADC_init()
{
	ADMUX |= (1 << REFS1);//internal 1.1v ref, right adjusted, ADC0 input
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //enable adc, enable interrupts slowest prescale (divide by 128)
}

void COMP_init()
{
	ACSR &= ~(1 << ACD);  //clear this disable bit (wtf?)
	ACSR |= (1 << ACIC); //alow analog comparator input capture, trigger on rising edge
	DIDR1 |= (1 << AIN1D) | (1 << AIN0D); //manual said to disable these digital inputs since I'm using it as analog
}

void SPI_init()
{
	DDRB |= (1<<MOSI_BIT | 1<<SCK_BIT | 1<<PORTB4); // /ss, sck, and mosi all outputs	
	PORTB |= 1<<PORTB4; //make /ss high
	SPSR0 |= 1 << SPI2X0;
	SPCR0 |= (1<<SPE0 | 1<<MSTR0); //enable in master mode, fosc/2 frequency ie 4 mhz
}

void LCD_init()
{
	//LCD port format
	//<Enable><NOTHING><RS><RW><DB7:4> and DB 3 : 0 are not connected
	
	//_delay_ms(100); //need to delay at least 40ms
	
	LCDPORT_DIR = 0xFF; //using LSBits of porta
	//see manual for init details
	
	lcd_cmd(0x33);
	lcd_cmd(0x32);
	lcd_cmd(0x2C);
	lcd_cmd(0x0C);
	lcd_cmd(0x01);
}

void print_char_lcd(char data)
{
	LCDPORT_DIR = 0xFF;
	char upper = (data & 0xF0) >> 4 | 0xA0; //upper data
	char lower = (data & 0x0F) | 0xA0; //lower data
	
	//notice the A instead of the 8, this makes RS true, selecting Data Reg
	
	LCDPORT = upper; //send cmd upper
	_delay_ms(2);
	LCDPORT ^= (0x80); //toggle enable
	_delay_ms(2);
	
	LCDPORT = lower;
	_delay_ms(2);
	LCDPORT ^= (0x80); //same thing but for lower
	_delay_ms(2);
}

void printLCD(char* a)
{
	for(int i = 0; a[i] != 0; i++)
	{
		print_char_lcd(a[i]);
	}
}

void lcd_cmd(uint8_t CMD)
{
	LCDPORT_DIR = 0xFF;
	uint8_t upper = ((CMD & 0xF0) >> 4); //upper command
	uint8_t lower = (CMD & 0x0F); //lower command
	
	LCDPORT = upper | 1<<7; //send cmd upper
	_delay_ms(2);
	LCDPORT ^= (0x80); //toggle enable
	_delay_ms(2);
	
	LCDPORT = lower | 1<<7;
	_delay_ms(2);
	LCDPORT ^= (0x80); //same thing but for lower
	_delay_ms(2);
}

void IO_init()
{
	IOPORT_DIR |= (1 << PIND7);
}

void TC_IC_init()
{
	TCCR1B |= /*(1 << ICNC1) |*/ (1 << ICES1) | (1 << CS11) ; //FILTER ON (maybe turn off), rising edge trigger, divide clock by 1
	TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);  //use interrupts and this also allows COMP to trigger an IC
}

void TC_DAC_init()
{
	TCCR0A |= (1 << WGM01); //OCRA to measure period
	TCCR0B |= (1 << CS01) | (1 << CS00); //64 prescale
	OCR0A = DACPERIOD;
	TIMSK0 |= (1 << OCIE0A); //interrupt when we hit dacperiod
}

uint16_t getADCVal()
{
	/*uint16_t ADC_Low = 0;
	uint16_t ADC_High = 0;
	uint16_t retval = 0;
	
	ADC_Low = ADCL;
	ADC_High = ADCH << 8; //shift by 8 bits for the sum
	
	retval = ADC_Low + ADC_High;
	
	ADCSRA |= 1<<ADIF; //clear the conversion done bit*/
	
	return ADC;
}

void send_DAC_value(uint16_t val)
{
	PORTB &= ~(1<<PORTB4); //turn off /ss
	
	//code to update outputs and load to DACA is 1001
	val = val << 2;
	val |= (0b1001 << 12);
	
	SPDR0 = val >> 8; //upper byte first
	
	while(!(SPSR0 & 1<<SPIF0)); //wait until SPIF is set
	
	SPDR0 = (0xFF & val); //send lower byte
	
	while(!(SPSR0 & 1<<SPIF0)); //wait until lower byte is done
	
	PORTB |= 1<<PORTB4; //toggle /ss to finish
}