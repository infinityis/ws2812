#include <msp430.h>
#include "ws2812/ws2812.h"

#define NUMBEROFLEDS	15
#define ENCODING 		3		// possible values 3 and 4

//For MSP430G2553, this works for up to 32 LEDs with encoding of 4, or 39 LEDs with encoding of 3. Beyond that it seems to use too much memory.

/*
 * In places with preprocessor directives like: #if defined(UCA0CTLW0)
 * the code is checking to see what capabilities exist for the target microcontroller.
 * If the value in question is defined, it means that a certain capabilities exist or
 * register names differ, and code appropriate to the device will be used.
 */

void configClock(void);
void configSPI(void);
void sendBuffer(uint8_t* buffer, ledcount_t ledCount);
void sendBufferDma(uint8_t* buffer, ledcount_t ledCount);
void shiftLed(ledcolor_t* leds, ledcount_t ledCount);

int main(void) {

	// buffer to store encoded transport data
	uint8_t frameBuffer[(ENCODING * sizeof(ledcolor_t) * NUMBEROFLEDS)] = { 0, };

	// array with 15 rbg colors
	ledcolor_t leds[NUMBEROFLEDS] = {
/*
			// rainbow colors
			{ 0xFF, 0x00, 0x00 },
			{ 0xFF, 0x3F, 0x00 },
			{ 0xFF, 0x7F, 0x00 },
			{ 0xFF, 0xFF, 0x00 },
			{ 0x00, 0xFF, 0x00 },
			{ 0x00, 0xFF, 0x3F },
			{ 0x00, 0xFF, 0x7F },
			{ 0x00, 0xFF, 0xFF },
			{ 0x00, 0x00, 0xFF },
			{ 0x3F, 0x00, 0xFF },
			{ 0x7F, 0x00, 0xFF },
			{ 0xFF, 0x00, 0xFF },
			{ 0xFF, 0x00, 0x7F },
			{ 0xFF, 0x00, 0x3F },
			{ 0xFF, 0x00, 0x0F },
*/
			// rainbow colors
			{ 0xFF, 0x00, 0x00 },
			{ 0xFF, 0x3F, 0x00 },
			{ 0xFF, 0x7F, 0x00 },
			{ 0xFF, 0xFF, 0x00 },
			{ 0x00, 0xFF, 0x00 },
			{ 0x00, 0xFF, 0x3F },
			{ 0x00, 0xFF, 0x7F },
			{ 0x00, 0xFF, 0xFF },
			{ 0x00, 0x00, 0xFF },
			{ 0x3F, 0x00, 0xFF },
			{ 0x7F, 0x00, 0xFF },
			{ 0xFF, 0x00, 0xFF },
			{ 0xFF, 0x00, 0x7F },
			{ 0xFF, 0x00, 0x3F },
			{ 0xFF, 0x00, 0x0F },
	};

	uint8_t update;
	ledcolor_t blankLed = {0x00, 0x00, 0x00};
	uint8_t colorIdx;
	ledcolor_t led;

	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	configClock();
	configSPI();

	while(1) {
		// blank all LEDs
		fillFrameBufferSingleColor(&blankLed, NUMBEROFLEDS, frameBuffer, ENCODING);
		sendBuffer(frameBuffer, NUMBEROFLEDS);
		__delay_cycles(0x100000);

		// Animation - Part1
		// set one LED after an other (one more with each round) with the colors from the LEDs array
		fillFrameBuffer(leds, NUMBEROFLEDS, frameBuffer, ENCODING);
		for(update=1; update <= NUMBEROFLEDS; update++) {
			sendBuffer(frameBuffer, update);
			__delay_cycles(0xFFFFF);
		}
		__delay_cycles(0xFFFFFF);

		// Animation - Part2
		// shift previous LED pattern
		for(update=0; update < 15*8; update++) {
			shiftLed(leds, NUMBEROFLEDS);
			fillFrameBuffer(leds, NUMBEROFLEDS, frameBuffer, ENCODING);
			sendBuffer(frameBuffer, NUMBEROFLEDS);
			__delay_cycles(0x7FFFF);
		}

		// Animation - Part3
		led = blankLed;
		// set all LEDs with the same color and simulate a sunrise
		for(colorIdx=0; colorIdx < 0xFF; colorIdx++) {
			led.red = colorIdx + 1;
			fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
			sendBuffer(frameBuffer, NUMBEROFLEDS);
			__delay_cycles(0x1FFFF);
		}
		for(colorIdx=0; colorIdx < 0xD0; colorIdx++) {
			led.green = colorIdx;
			fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
			sendBuffer(frameBuffer, NUMBEROFLEDS);
			__delay_cycles(0x2FFFF);
		}
		for(colorIdx=0; colorIdx < 0x50; colorIdx++) {
			led.blue = colorIdx;
			fillFrameBufferSingleColor(&led, NUMBEROFLEDS, frameBuffer, ENCODING);
			sendBuffer(frameBuffer, NUMBEROFLEDS);
			__delay_cycles(0x3FFFF);
		}
		__delay_cycles(0xFFFFF);
	}
	return 0;
}

void shiftLed(ledcolor_t* leds, ledcount_t ledCount) {
	ledcolor_t tmpLed;
	ledcount_t ledIdx;

	tmpLed = leds[ledCount-1];
	for(ledIdx=(ledCount-1); ledIdx > 0; ledIdx--) {
		leds[ledIdx] = leds[ledIdx-1];
	}
	leds[0] = tmpLed;
}

// copy bytes from the buffer to SPI transmit register
void sendBuffer(uint8_t* buffer, ledcount_t ledCount) {
#if defined(DMA0CTL_)                       //used for MSP430FR5739 (and similar)
	//If DMA exists for the microcontroller, this code will make use of it
	DMA0SA = (__SFR_FARPTR) buffer;		    // source address
	DMA0DA = (__SFR_FARPTR) &UCA0TXBUF_L;	// destination address
	// single transfer, source increment, source and destination byte access, interrupt enable
	DMACTL0 = DMA0TSEL__UCA0TXIFG;		    // DMA0 trigger input
	DMA0SZ = (ENCODING * sizeof(ledcolor_t) * ledCount);
	DMA0CTL = DMADT_0 | DMASRCINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAEN;

	//start DMA
	UCA0IFG ^= UCTXIFG;
	UCA0IFG ^= UCTXIFG;
#else
	uint16_t bufferIdx;
	for (bufferIdx=0; bufferIdx < (ENCODING * sizeof(ledcolor_t) * ledCount); bufferIdx++) {
		                          // wait for TX buffer to be ready
  #if defined(UCA0TXIFG)          //used for MSP430G2553 (and similar)
		while (!(IFG2 & UCA0TXIFG));
  #elif defined(UCA0IFG)          //used for MSP430FR5739 (and similar)
		while (!(UCA0IFG & UCTXIFG));
  #endif

  #if defined(UCA0TXBUF_)         //used for MSP430FR5739 (and similar) where the USCI TX buffer is two bytes in size
		UCA0TXBUF_L = buffer[bufferIdx];
  #else   //used for MSP430G2553 and similar where the USCI TX buffer is one byte in size
		UCA0TXBUF = buffer[bufferIdx];
  #endif

	}
	__delay_cycles(300);
#endif
}





void configClock(void) {
#if defined(CSCTL0_)       //used for MSP430FR5739 (and similar)
	// configure MCLK and SMCLK to be sourced by DCO with a frequency of
	//   8Mhz (3-bit encoding)
	// 6.7MHz (4-bit encoding)
	CSCTL0_H = 0xA5;
  #if ENCODING == 3
	CSCTL1 = DCOFSEL_3;       // DCO frequency setting = 8MHz
  #else
	CSCTL1 = DCOFSEL_2;       // DCO frequency setting = 6.7MHz
  #endif
	CSCTL2 = SELS__DCOCLK + SELM__DCOCLK;
	CSCTL3 = DIVS__1 + DIVM__1;
#else                      //used for MSP430G2553 (and similar)
	// configure MCLK and SMCLK to be sourced by DCO with a frequency of 16Mhz
	BCSCTL1 = CALBC1_16MHZ;   //DCO frequency setting 16MHz
	DCOCTL = CALDCO_16MHZ;

	BCSCTL2 = DIVS_0 + DIVM_0 + SELM_0;  //set MCLK and SMCLK to the DCO clock of 16MHz
#endif
}

void configSPI(void) {

#if defined(UCA0CTLW0)     //used for MSP430FR5739 (and similar)
	UCA0CTLW0 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTLW0 |= UCMST + UCSYNC + UCCKPL + UCMSB;	// 3-pin, 8-bit SPI master
													// Clock polarity high, MSB
	UCA0CTLW0 |= UCSSEL__SMCLK;                     // SMCLK

  #if ENCODING == 3
	UCA0BR0 = 3;		   // SPICLK 8MHz/3 = 2.66MHz
	UCA0BR1 = 0;
  #else
	UCA0BR0 = 2;		   // SPICLK 6.7MHz/2 = 3.35MHz
	UCA0BR1 = 0;
  #endif
	UCA0MCTLW = 0;                            // No modulation
	UCA0CTLW0 &= ~UCSWRST;
#else                      //used for MSP430G2553 (and similar)
	UCA0CTL1 |= UCSWRST;   //hold USCI A0 in reset

    UCA0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;
    UCA0CTL1 = UCSSEL_2 | UCSWRST;   // Clock from SMCLK; hold in reset
  #if ENCODING == 3
	UCA0BR0 = 6;           // SPICLK 16MHz/6 = 2.66MHz
	UCA0BR1 = 0;
  #else
	UCA0BR0 = 5;           // SPICLK 16MHz/5 = 3.2MHz
	UCA0BR1 = 0;           // 3.2MHz provides 0.31us per bit, so 4 bit operation results in
	                       // 0.31us (within 0.15us of both 0.40us and 0.45us) and and 0.93us (within 0.15us of 0.80us and 0.85us)
  #endif

	P1SEL = BIT1 + BIT2;   //need to set P1SEL to assign pins to the USCI peripheral
	P1SEL2 = BIT1 + BIT2;  //need to set P1SEL to assign pins to the USCI peripheral

	UCA0CTL1 &= ~UCSWRST;  //release USCI A0 to operate normally
#endif

}
