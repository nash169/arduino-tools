#include "transmitter.h"

Transmitter::Transmitter(const uint8_t *pins, const uint8_t numPins)
{
	numCtrs = numPins;
	dPin = new CtrPin[numCtrs];

	for (int i = 0; i < numCtrs; ++i)
		dPin[i].digitalPin = pins[i];
}

Transmitter::Transmitter() { }

Transmitter::~Transmitter() { }

void Transmitter::Initialize(const float *minIns, const float *maxIns, const int *minOuts, const int *maxOuts)
{
	for (int i = 0; i < numCtrs; ++i)
	{
		dPin[i].ctr = new Servo;
		dPin[i].ctr->attach(dPin[i].digitalPin);
		dPin[i].minIn = minIns[i];
		dPin[i].maxIn = maxIns[i];
		dPin[i].minOut = minOuts[i];
		dPin[i].maxOut = maxOuts[i];
		dPin[i].output = 0;
		SetServo(dPin[i].output, i);
	}
}

void Transmitter::SetServo(float inputCtr, uint8_t ch)
{
	if (inputCtr < dPin[ch].minIn)
      inputCtr = dPin[ch].minIn;
    else if (inputCtr > dPin[ch].maxIn)
      inputCtr = dPin[ch].maxIn;

  	dPin[ch].input = inputCtr;

    float Q = abs(dPin[ch].maxIn-dPin[ch].minIn)/abs(dPin[ch].maxOut-dPin[ch].minOut);

  	dPin[ch].output = round(abs(dPin[ch].input-dPin[ch].minIn)/Q + dPin[ch].minOut);

	dPin[ch].ctr->writeMicroseconds(dPin[ch].output);
}

void Transmitter::PrintIn() {
  for (int i = 0; i < numCtrs-1; ++i)
  {
    Serial.print("CH"); Serial.print(i); Serial.print(": ");
    Serial.print(dPin[i].input); Serial.print("\t");
  }
  Serial.print("CH"); Serial.print(numCtrs-1); Serial.print(": ");
  Serial.println(dPin[numCtrs-1].input);
}

void Transmitter::PrintOut() {
  for (int i = 0; i < numCtrs-1; ++i)
  {
    Serial.print("CH"); Serial.print(i); Serial.print(": ");
    Serial.print(dPin[i].output); Serial.print("\t");
  }
  Serial.print("CH"); Serial.print(numCtrs-1); Serial.print(": ");
  Serial.println(dPin[numCtrs-1].output);
}
