//Brandon Pollack
//Definitions and Declarations
#define LCDPORT PORTC //Port C is the IO port
#define LCDPORT_DIR DDRC

#define IOPORT PORTD
#define IOPORT_DIR DDRD

#define C3 13081 //low c
#define D3 14683
#define E3 16481
#define F3 17461
#define G3 19600
#define A3 22000
#define B3 24694
#define C4 26163 //middle c
#define D4 29367
#define E4 32963
#define F4 34923
#define G4 39200
#define A4 44000
#define B4 49388
#define C5 52325 //high C

#define DACPERIOD 125 //125 clocks when prescale is 64 and we re running at 8 mhz for 1000 samples/sec audio

void inline ADC_init(); //gets new value whenever I tell it to
void inline COMP_init(); //comparator that events to the TC
void inline SPI_init(); //sends data to play boot sound
void inline LCD_init();
void inline IO_init(); //makes LED output somewhere so I can light up when in tune
void inline TC_DAC_init(); //1000 Hz interrupt to send next value of boot sound
void inline TC_IC_init(); //gets event from comparator to measure period, if we don't have a rising edge for more than lowest note (tbd) then stop counting

uint16_t getADCVal(); //get the value in the result register

void print_char_lcd(char data);
void lcd_cmd(uint8_t CMD);
void printLCD(char* a);

void send_DAC_value(uint16_t val); //send a 10 bit DAC value, with the 4 bit write to A command appended

uint8_t findNote(uint16_t freq, char currentNote[]); //returns 1 if you are sharp, 0 if flat

static uint16_t freq;

static char line2buffer[16];

static uint16_t lastCapture; //static variables init to 0

static uint16_t ICval;