#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

void xmitString ( char * string );

/*********************************************************\
    Initialize Board functions
\*********************************************************/
void initializeBoard(void)
{
    // set up general configuration bits
    // disable peripherals

    PRR =   
            _BV(PRSPI) |        // SPI
            _BV(PRTIM2) |        // TImer/Counter 2
            _BV(PRTIM1) |        // Timer/Counter 1
            _BV(PRTWI) |        // Two-wire interface
            _BV(PRADC);            // Analog/digital converter
    ACSR &= ~_BV(ACD);            // Analog comparator
    
    MCUCR &= ~_BV(PUD); // enable pull ups
    
    DDRB = 0b00000000; // 0 outputs
    PORTB =0b11111111; // enable pullup resistors
    
    DDRC = 0b00000000; // 0 outputs
    PORTC =0b11111111; // enable pullups

    DDRD = 0b00001000; // 1 outputs    
    PORTD =0b11110111; // enable pull ups on resistors
    
    //set up UART
    // UCSR0A &= ~_BV(U2X0) & ~_BV(MPCM0);
    // UCSR0B = 0b10011000;
    // UCSR0C = 0b00101111;
    // UBRR0H = 0;
   // UBRR0L = 103; // baud rate 4800
    // UBRR0L = 1; // baud rate 250,000
    
    // set up timer interrupts
    TCCR0A = _BV(WGM01); // CTC on OCR0A
    TCCR0B = _BV(CS02) | _BV(CS00); // prescale by 1024 (1 count ~8kHz)
    OCR0A = 100; // divide by another 200 (~40Hz)
    TIMSK0 = _BV(OCIE0A); // generate interrupt on match with A
    
    // set button interrupt
//    PCMSK0 = button_bit;
//    PCICR = _BV(PCIE0);

}

uint8_t pixels[105];

uint8_t spiral_init[] PROGMEM = {
120,0,255,
130,0,255,
250,0,255,
110,0,255,
100,0,255,
240,0,255,
255,0,170,
255,0,250,
140,0,255,
0,0,255,
150,0,255,
255,0,240,
255,0,180,
230,0,255,
90,0,255,
80,0,255,
220,0,255,
255,0,190,
255,0,200,
255,0,210,
255,0,220,
255,0,230,
160,0,255,
10,0,255,
20,0,255,
170,0,255,
180,0,255,
190,0,255,
200,0,255,
210,0,255,
70,0,255,
60,0,255,
50,0,255,
40,0,255,
30,0,255
}; // 470 seems to be the max size of this array? 468 = 156 pixels * 3 bytes


void dumpColor(uint16_t numBytes)
{

    uint8_t pinMask = 8;
    volatile uint16_t
    i = numBytes; // Loop counter
    volatile uint8_t
        *ptr = pixels,   // Pointer to next byte
        b    = *ptr++,   // Current byte value
        hi,             // PORT w/output bit set high
        lo;             // PORT w/output bit set low

    // Hand-tuned assembly code issues data to the LED drivers at a specific
    // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
    // for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
    // datastream timing for the LED drivers allows a little wiggle room each
    // way (listed in the datasheets), so the conditions for compiling each
    // case are set up for a range of frequencies rather than just the exact
    // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
    // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
    // on the datasheet figures and have not been extensively tested outside

    // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
    // close to the extremes (or possibly they could be pushed further).
    // Keep in mind only one CPU speed case actually gets compiled; the
    // resulting program isn't as massive as it might look from source here.

    volatile uint8_t n1, n2 = 0;    // First, next bits out

    // Squeezing an 800 KHz stream out of an 8 MHz chip requires code
    // specific to each PORT register.  At present this is only written
    // to work with pins on PORTD or PORTB, the most likely use case --
    // this covers all the pins on the Adafruit Flora and the bulk of
    // digital pins on the Arduino Pro 8 MHz (keep in mind, this code
    // doesn't even get compiled for 16 MHz boards like the Uno, Mega,
    // Leonardo, etc., so don't bother extending this out of hand).
    // Additional PORTs could be added if you really need them, just
    // duplicate the else and loop and change the PORT.  Each add'l
    // PORT will require about 150(ish) bytes of program space.

    // 10 instruction clocks per bit: HHxxxxxLLL
    // OUT instructions:              ^ ^    ^   (T=0,2,7)

    hi = PORTD |  pinMask;
    lo = PORTD & ~pinMask;
    n1 = lo;
    if(b & 0x80) n1 = hi;

    // Dirty trick: RJMPs proceeding to the next instruction are used
    // to delay two clock cycles in one instruction word (rather than
    // using two NOPs).  This was necessary in order to squeeze the
    // loop down to exactly 64 words -- the maximum possible for a
    // relative branch.

    asm volatile(
     "headD:"                   "\n\t" // Clk  Pseudocode
    // Bit 7:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
    "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 6"        "\n\t" // 1-2  if(b & 0x40)
     "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 6:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
    "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 5"        "\n\t" // 1-2  if(b & 0x20)
     "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 5:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
    "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 4"        "\n\t" // 1-2  if(b & 0x10)
     "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 4:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
    "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 3"        "\n\t" // 1-2  if(b & 0x08)
     "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 3:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
    "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 2"        "\n\t" // 1-2  if(b & 0x04)
     "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 2:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
    "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 1"        "\n\t" // 1-2  if(b & 0x02)
     "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "rjmp .+0"                "\n\t" // 2    nop nop
    // Bit 1:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
    "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
    "rjmp .+0"                "\n\t" // 2    nop nop
    "sbrc %[byte] , 0"        "\n\t" // 1-2  if(b & 0x01)
     "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "sbiw %[count], 1"        "\n\t" // 2    i-- (don't act on Z flag yet)
    // Bit 0:
    "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
    "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
    "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
    "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++
    "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80)
     "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
    "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
    "brne headD"              "\n"   // 2    while(i) (Z flag set above)
    : [byte]    "+r" (b),
    [n1]    "+r" (n1),
    [n2]    "+r" (n2),
    [count]    "+w" (i)
    : [port]    "I" (_SFR_IO_ADDR(PORTD)),
    [ptr]    "e" (ptr),
    [hi]    "r" (hi),
    [lo]    "r" (lo));
}

int main(void)
{
    // initialize board
    initializeBoard();
    sei(); // enable global interrupts
    while(1);
}

void changeColor(uint16_t numBytes)
{
    uint8_t i, j;
    
    for( i = 0; i < numBytes; i+= 3)
    {
        for( j = 0; j < 3; j++)
        {
            if( pixels[i + j] > 0 && pixels[i + (j+ 2)%3] == 0)
            {
                if( pixels[i + (j+1)%3] >= 255 )
                {
                    pixels[ i + j] -= 1;
                }
                else
                {
                    pixels[ i + (j+1)%3 ] += 1;
                }
                break;
            }
            else if( pixels[i + j] == 0 && pixels[i + (j+1)%3] == 0 && pixels[i + (j+2)%3] == 0)
            {
                // all zeros
                pixels[i+j] += 1;
            }
        }
    }

}

void set_new_brightness( uint8_t* inner, uint8_t* mid, uint8_t* outer,
                         uint8_t direction )
{
    static uint8_t counter = 0;


    if(direction)
    {
        if(counter < 255)
            counter += 5;
    }
    else
    {
        if(counter > 0)
            counter -= 5;
    }

    if( counter < 68 )
        *inner = 3*counter + 50;
    else
        *inner = 255;

    if( counter < 64 )
        *mid = 0;
    else if( counter < 128 )
        *mid = (uint16_t)(counter)*4 - 256;
    else
        *mid = 255;

    if( counter < 127 )
        *outer = 0;
    else if(counter < 192 )
        *outer = (uint16_t)(counter)*4 - 512;
    else
        *outer = 255;
}

void set_pixel(uint8_t i, uint8_t G, uint8_t R, uint8_t B, uint8_t brightness )
{
    pixels[i*3] = ((uint16_t)(G*brightness))/255;
    pixels[i*3 + 1] = ((uint16_t)(R*brightness))/255;
    pixels[i*3 + 2] = ((uint16_t)(B*brightness))/255;
}

ISR(TIMER0_COMPA_vect)
{
    static uint16_t counter = 0;
    static uint8_t state = 0;
    uint8_t i;
    const uint16_t time_table[] = {40, 4000};
    
    uint8_t inners[] = {6, 12, 17, 18, 19, 20, 21, 11},
            mids[] = {2, 5, 13, 16, 29, 28, 27, 26, 25, 22, 10, 7},
            outers[] = {0, 3, 4, 14, 15, 30, 31, 32, 33, 34, 24, 23, 9, 8, 1};
            
    uint8_t inner_brightness,
            mid_brightness,
            outer_brightness;
    
    switch (state)
    {
        case 0:
        // colors spiraling outward
            if( counter == 0 )
            {
                for (i=0; i< sizeof(pixels); i++)
                {
                    pixels[i] = pgm_read_byte(&(spiral_init[i]));
                }
            }
            counter++;
            if( counter == time_table[state] )
            {
                counter = 0;
                state++;
            }
            changeColor(105);
            break;
        case 1:
    // Pulsing
            set_new_brightness(&inner_brightness, &mid_brightness, &outer_brightness,
                               (counter % 120) / 80);
            
            for(i=0; i< sizeof(inners); i++)
                set_pixel(inners[i], 0, 255, 50, inner_brightness);

            for(i=0; i< sizeof(mids); i++)
                set_pixel(mids[i], 0, 255, 255, mid_brightness);

            for(i=0; i< sizeof(outers); i++)
                set_pixel(outers[i], 160, 70, 230, outer_brightness);
            
            counter++;

            if( counter == time_table[state] )
            {
                counter = 0;
                state = 0;
            }
            break;
    }
    dumpColor(105);
}

