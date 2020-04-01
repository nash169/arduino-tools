#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "Arduino.h"

#include <math.h>
#include <Servo.h>

struct CtrPin
{
	uint8_t digitalPin;
	float input;
	int output;

	Servo *ctr;
	float minIn, maxIn;
	int minOut, maxOut;
};

class Transmitter
{
public:
	Transmitter(const uint8_t *pins, const uint8_t numPins);
	Transmitter();
	~Transmitter();

	void Initialize(const float *minIns, const float *maxIns, const int *minOuts, const int *maxOuts);
	inline int GetCtr(uint8_t ctr) { return dPin[ctr].output; }
	void SetServo(float inputCtr, uint8_t ch);
	void PrintIn();
	void PrintOut();

protected:

private:
	uint8_t numCtrs;
	CtrPin *dPin;
};

#endif
