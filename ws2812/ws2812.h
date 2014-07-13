#include <stdint.h>

//removed the BIT0, BIT1, etc. #defines as it conflicted with those in the microcontroller-specific header files

typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} ledcolor_t;

typedef uint16_t ledcount_t;

void fillFrameBuffer(ledcolor_t* leds, ledcount_t ledCount, uint8_t* buffer, uint8_t encoding);
void fillFrameBufferSingleColor(ledcolor_t* led, ledcount_t ledCount, uint8_t* buffer, uint8_t encoding);
void encodeData3bit(ledcolor_t* led, uint8_t* output);
void encodeData4bit(ledcolor_t* led, uint8_t* output);
