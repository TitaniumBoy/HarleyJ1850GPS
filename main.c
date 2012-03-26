#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#define F_CPU		12000000
#include <util/delay.h>

/*-------------------------------------------------------------------------*/

#define TIMER_PRESCALER	8

/* timing formulas */
#define US(x)		((x) * ((F_CPU) / 1000000) / (TIMER_PRESCALER))
#define MS(x)		((x) * ((F_CPU) / 1000) / (TIMER_PRESCALER))
#define SECS_TO_OVF(x)	((x) * (((F_CPU) / (TIMER_PRESCALER)) / 65536))

#define VPW_SHORT_PULSE_MIN	 35     /* us */
#define VPW_LONG_PULSE_MIN	 97     /* us */
#define VPW_SOF_MIN		164     /* us */
#define VPW_EOF_MIN		240     /* us */
#define VPW_SLEEP_MIN		 20     /* s */

#undef INVERSE_POLARITY

#define PPS

static unsigned char ticnt[5] = "000\r\n";
static unsigned calclk = MS(1);

/*-------------------------------------------------------------------------*/
#define OUTSIZE 64
static unsigned char outbuf[OUTSIZE + (OUTSIZE >> 1)], outhead, outtail, midbuf[OUTSIZE], midlen;

// add buffered data from J1850 or PPS or whereever to output buffer
static void sendmid()
{
    unsigned char *mp = midbuf;
    while (midlen) {
        outbuf[outhead++] = *mp++;
        if (outhead >= OUTSIZE)
            outhead = 0;
        midlen--;
    }
}

// time in TOVFs to make the UART RX to TX transparent (dump other traffic).
static unsigned transptime = 0;
static unsigned char innmea = 0;
// Received character from GPS - put to transmit buffer, but also handle other cases
ISR(USART_RX_vect)
{
    unsigned char c = UDR;

    if (!c || c == 0xa0)
        transptime = SECS_TO_OVF(10);
    if (c == '$' && !transptime) {
	innmea = 1;
	// prepend any already queued lines - maybe redundant
	if (midlen)
	    sendmid();
	// prepend ms timestamp
        unsigned char t;
        for (t = 0; t < 5; t++) {
            outbuf[outhead++] = ticnt[t];
            if (outhead >= OUTSIZE)
                outhead = 0;
        }
    }

    outbuf[outhead++] = c;
    if (outhead >= OUTSIZE)
        outhead = 0;
    if (midlen && (!c || c == '\n')) {
	innmea = 0;
	// append any queued lines
        if (transptime)
            midlen = 0;         // dump during config
        else
            sendmid();
    }
    // enable send
    UCSRB |= _BV(UDRIE);
}

// Send next character from transmit buffer
ISR(USART_UDRE_vect)
{
    // disable interrupt when buffer empty
    if (outhead == outtail) {
        UCSRB &= ~_BV(UDRIE);
        return;
    }
    UDR = outbuf[outtail++];
    if (outtail >= OUTSIZE)
        outtail = 0;
}

// PPS on the UTC second mark
ISR(INT1_vect)
{
    OCR1B = TCNT1 + calclk; // resync mS counter

    // can adjust calclk here based on ticnt not being 999 or just rollover to 000

    memcpy(&midbuf[midlen], ticnt, 5);
    midlen += 5;
    midbuf[midlen++] = '=';
    midbuf[midlen++] = '\r';
    midbuf[midlen++] = '\n';
    ticnt[0] = ticnt[1] = ticnt[2] = '0';
    if (!innmea) {
	sendmid();
	UCSRB |= _BV(UDRIE);
    }
}

/*-------------------------------------------------------------------------*/
// set up UART port and configuration, out of reset and deep sleep
static void uart_init()
{

    outhead = outtail = midlen = innmea = 0;
    /* turn on bluetooth */
    DDRB |= _BV(PB1);
    PORTB |= _BV(PB1);

    /* setup serial port */
#define BAUD 115200
#include <util/setbaud.h>
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X);
#else
    UCSRA &= ~(1 << U2X);
#endif
    UCSRC = _BV(UCSZ0) | _BV(UCSZ1);    /* 8N1 */
    UCSRB = _BV(RXEN) | _BV(TXEN);      /* enable rx/tx */
}

// send string from flash to local buffer and push it out
static void uart_puts_P(const char *p)
{
    char c = pgm_read_byte(p);
    while (c) {
        midbuf[midlen++] = c;
        c = pgm_read_byte(++p);
    }
    sendmid();
    UCSRB |= _BV(UDRIE);
}

/*-------------------------------------------------------------------------*/
// 1mS tick - increment millisecond tick string
ISR(TIMER1_COMPB_vect)
{
    OCR1B += calclk;
    if (ticnt[2] < '9') {
        ticnt[2]++;
        return;
    }
    ticnt[2] = '0';
    if (ticnt[1] < '9') {
        ticnt[1]++;
        return;
    }
    ticnt[1] = '0';
    if (ticnt[1] < '9')
        ticnt[0]++;
    else
        ticnt[0] = '0';
}

/*-------------------------------------------------------------------------*/
static volatile unsigned int lastedge;  /* = 0 */
static unsigned char polarity;  /* = 0 */
static unsigned char jbitaccum, jbitcnt, jmsgbuf[40], jmsglen;

// General setup of timing and J1850 - for out of reset and out of deep sleep
void receiver_init(void)
{
    lastedge = 0;
#ifndef INVERSE_POLARITY
    polarity = 0;
#else
    polarity = 1;
#endif
    jmsglen = 0;
    jbitaccum = 0;
    jbitcnt = 0;
    /* j1850 input - PD6 as input without pullup */
    DDRD &= ~_BV(PD6);
    PORTD &= ~_BV(PD6);

    GTCCR = _BV(PSR10);         /* reset prescaler */
    TCNT1 = 0;                  /* reset counter value */
    /* activate noise canceller, */
    /* trigger on rising edge, clk/8 */
    TCCR1B = _BV(ICES1) | _BV(ICNC1) | _BV(CS11);

    /* clear and enable Overflow and Input Capture interrupt */
    TIFR |= _BV(TOV1) | _BV(ICF1);
    TIMSK |= _BV(TOIE1) | _BV(ICIE1);

    OCR1A = US(VPW_EOF_MIN);    /* timeout - EOD */

#ifdef PPS
    MCUCR |= 0xC;
    GIMSK |= 0x80;
    TIFR |= _BV(OCF1B);         /* clear compare match interrupt */
    TIMSK |= _BV(OCIE1B);       /* enable compare match interrupt */
#endif

}

/*-------------------------------------------------------------------------*/
// Special timing requires assembly to disable brownout detect.
#define sleep_bod_disable() \
do { \
	uint8_t tempreg; \
	__asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
			     "ori %[tempreg], %[bods_bodse]" "\n\t" \
			     "out %[mcucr], %[tempreg]" "\n\t" \
			     "andi %[tempreg], %[not_bodse]" "\n\t" \
			     "out %[mcucr], %[tempreg]" \
			     : [tempreg] "=&d" (tempreg) \
			     : [mcucr] "I" _SFR_IO_ADDR(BODCR), \
			       [bods_bodse] "i" (_BV(BPDS) | _BV(BPDSE)), \
			       [not_bodse] "i" (~_BV(BPDSE))); \
} while (0)

// Wake-up on J1850 edge after sleep
ISR(PCINT_D_vect)
{
#define PCIE2 4
    GIMSK &= ~_BV(PCIE2);
}

// Shut things down, deep-sleep until j1850 interrupt, then restart everything
void deepsleep(void)
{
    /* disable timers */
    TIMSK = 0;

    UCSRB &= ~_BV(RXCIE);       // disable receives

    outhead = outtail = midlen = 0;
    uart_puts_P(PSTR("\r\n=Sleep\r\n"));
    sei();
    while (UCSRB & _BV(UDRIE));
    cli();
    _delay_ms(20);

    /* turn off bluetooth */
    PORTB &= ~_BV(PB1);

    /* disable UART */
    UCSRB = 0;
    _delay_ms(1);

    /* set RX/TX as inputs in HiZ to prevent leakage */
    DDRD &= ~(_BV(PD0) | _BV(PD1));
    PORTD &= ~(_BV(PD0) | _BV(PD1));

    /* clear and enable bit change interrupt for wakeup */
    GIMSK = _BV(PCIE2);
    PCMSK2 = _BV(PCINT17);
#define PCIF2 4
    EIFR = _BV(PCIF2);

    /* power down with BOD disabled, INT will awake */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();

    receiver_init();
    /* setup uart */
    uart_init();
    _delay_ms(1);

    uart_puts_P(PSTR("=WakeUp\r\n"));

    UCSRB |= _BV(RXCIE);
}

/*-------------------------------------------------------------------------*/

volatile unsigned inactime;     /* = 0 */
// overflow - used as a coarse counter for long timeouts
ISR(TIMER1_OVF_vect)
{
    // downcount transparency counter if active
    if (transptime) {
        transptime--;
        return;
    }
    // downcount inactivity timer
    if (inactime > SECS_TO_OVF(VPW_SLEEP_MIN)) {
        if (PIND & _BV(PD4))
            return;             // don't powerdown while BT active
        inactime = 0;
        deepsleep();
    }
    else
        ++inactime;
}

// J1850 EOD/EOF after 300 or so microseconds.
ISR(TIMER1_COMPA_vect)
{
    jbitcnt = jbitaccum = 0;
    /* disable compare match interrupt */
    TIMSK &= ~_BV(OCIE1A);

    lastedge = TCNT1;

    /* timeout - J1850 EOD/EOF */
    jmsgbuf[jmsglen++] = '\r';
    jmsgbuf[jmsglen++] = '\n';
    memcpy(&midbuf[midlen], jmsgbuf, jmsglen);
    midlen += jmsglen;
    jmsglen = 0;
    if (!innmea) {
	sendmid();
	UCSRB |= _BV(UDRIE);
    }
}

// Process edge on J1850
ISR(TIMER1_CAPT_vect)
{
    unsigned int now, width;

    /* toggle interrupt on rising/falling edge */
    TCCR1B ^= _BV(ICES1);

    now = ICR1;
    width = now - lastedge;
    lastedge = now;
    polarity ^= 1;

    /* bad width or outside, will happen at EOD */
    if (width >= US(VPW_EOF_MIN) || width < US(VPW_SHORT_PULSE_MIN))
        return;

    OCR1A = now + US(VPW_EOF_MIN);      /* timeout - EOD */
    TIFR |= _BV(OCF1A);         /* clear compare match interrupt */
    TIMSK |= _BV(OCIE1A);       /* enable compare match interrupt */

    inactime = 0;

    /* doesn't quite do IFRs - normalization bit, crc? */
    if (!polarity && width >= US(VPW_SOF_MIN)) {
        memcpy(jmsgbuf, ticnt, 5);
        jmsglen = 5;
        jmsgbuf[jmsglen++] = 'J';
        jbitcnt = jbitaccum = 0;
        return;
    }

    jbitaccum <<= 1;
    if ((width < US(VPW_LONG_PULSE_MIN)) ^ polarity)
        jbitaccum++;
    if (++jbitcnt < 4)
        return;
    jmsgbuf[jmsglen++] = jbitaccum > 9 ? 'A' - 10 + jbitaccum : '0' + jbitaccum;
    jbitcnt = jbitaccum = 0;
}

/*-------------------------------------------------------------------------*/

int main(void)
{
    /* power savings */
    PRR = _BV(PRUSI);           /* shut down USI */
    DIDR = _BV(AIN0D) | _BV(AIN1D);     /* disable digital input on analog */
    ACSR = _BV(ACD);            /* disable analog comparator */

    /* setup uart */
    uart_init();
    receiver_init();
    sei();
    uart_puts_P(PSTR("=Harley J1850-GPS\r\n"));

    UCSRB |= _BV(RXCIE);
    for (;;) {
        /* idle sleep, INT from J1850 or serial will awake */
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
}
