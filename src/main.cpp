#include <Arduino.h>

#define PIN_CLOCK 2
#define PIN_VSYNC 10
#define PIN_HSYNC 3

static uint16_t line = 0;
static uint16_t prescale_compensator = 0;


/**
 * The idea is to generate 3 different signals.
 * PIXELCLOCK: The clock we're gonna use to generate pixel signals and patterns, all the other signals are derived from this for stability (image running away etc).
 * HSYNC: Fires when we're done drawing a line on the X-axis.
 * VSYNC: Fires when all the drawing is done and tells the monitor to update the screen.
 * 
 * The idea is to have insanely fast Timer running to generate our pixel clock.
 * Using an interrupt linked to that Timer we can increase our HSYNC which will in term increase the VSYNC.
 * We should also look into reducing the overall frequency, the current freq of the HSYNC is 31khz which works but we would need a 18Mhz clock to generate 600 pixels on the X-axis, so thats not gonna happen.
 */


void vga_begin()
{
    // disable TIMER0 interrupt
    TIMSK0 = 0;
    TCCR0A = 0;
    TCCR0B = (1 << CS00);
    OCR0A = 0;
    OCR0B = 0;
    TCNT0 = 0;

    pinMode(PIN_VSYNC, OUTPUT);

    // attachInterrupt(digitalPinToInterrupt(PIN_CLOCK), func, FALLING);

    pinMode(13, OUTPUT);


    // TIMER2 - Generates the HSYNC pulses and has an interrupt that fires.
    pinMode(PIN_HSYNC, OUTPUT);
    TCCR2A = bit(WGM20) | bit(WGM21) | bit(COM2B1); // pin3=COM2B1
    TCCR2B = bit(WGM22) | bit(CS20);                // 8 prescaler
    OCR2A = 63;                                     // 32 / 0.5 uS=64 (minus one)
    OCR2B = 7;                                      // 4 / 0.5 uS=8 (minus one)
    // OCR2A = 2;                                   // 32 / 0.5 uS=64 (minus one)
    // OCR2B = 1;                                   // 4 / 0.5 uS=8 (minus one)
    TIFR2 = bit(OCF2B);                             // clear Compare Match B flag
    TIMSK2 = bit(OCIE2B);                           // enable Compare Match B interrupt

    sei();
}

// Timer2 COMPB ISR - Generates the VSYNC pulses
ISR(TIMER2_COMPB_vect)
{
    prescale_compensator++;

    if (prescale_compensator >= 3)
        PORTB |= bit(5);
    if (prescale_compensator >= 8)
    {

        prescale_compensator = 0;
        line++;
        // VSYNC signal
        if (line > 522)
        {
            PORTB |= bit(2);
            if (line > 524)
            {
                PORTB &= ~bit(2);
                line = 0;
            }
        }
        PORTB &= ~bit(5);

    }
}

void setup(void)
{
    vga_begin();
}

void loop()
{
}
