#ifndef RADIO_DECODER_H
#define RADIO_DECODER_H

#include "Arduino.h"

#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

struct ChPin
{
	uint8_t analogPin;
	int status; // ?

	uint16_t rc_values;
	uint32_t rc_start;
	volatile uint16_t rc_shared;

	uint16_t minIn, maxIn;
	float minOut, maxOut;

	float output;
};

class RadioDecoder
{
public:
	RadioDecoder(const uint8_t *pins, const uint8_t numPins, const uint8_t mode);
	RadioDecoder();
	~RadioDecoder();
	void Initialize(const float *minOuts, const float *maxOuts, bool CALIBRATE);
	void rc_read_values();
	inline uint16_t GetChannel(uint8_t chIndex) { return channel[chIndex].rc_values; }
	float CtrInput(int ch);
	void PrintIn();
	void PrintOut();

protected:
	void PWM_Input(uint8_t channel, uint8_t input_pin);
	void PPM_Input();
	void Calibrate();

private:
	static void CallbackGlue();
	void Callback();

	static RadioDecoder *isr;
	uint8_t numChannels, ChCounter, firstPin, type, startCh;
	ChPin *channel;

	enum {
		PWM,
		PPM,
	};
};

#endif // RADIO_DECODER_H
