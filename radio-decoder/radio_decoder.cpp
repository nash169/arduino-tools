#include "radio_decoder.h"

RadioDecoder *RadioDecoder::isr;

RadioDecoder::RadioDecoder(const uint8_t *pins, const uint8_t numPins, const uint8_t mode) {
  numChannels = numPins;
  ChCounter = 0;
  channel = new ChPin[numChannels];

  type = mode;
  firstPin = pins[0];
  startCh = 0;

  switch (type) {
    case PWM:
      for (int i = 0; i < numChannels; ++i)
        channel[i].analogPin = pins[i];
      break;

    case PPM:
      for (int i = 0; i < numChannels; ++i)
        channel[i].analogPin = firstPin;
      break;

    default:
      break;
  }
}

RadioDecoder::~RadioDecoder() {  }

void RadioDecoder::Initialize(const float *minOuts, const float *maxOuts, bool CALIBRATE)
{
  isr = this;

  switch (type) {
    case PWM:
      for (int i = 0; i < numChannels; ++i)
      {
        channel[i].status = digitalRead(channel[i].analogPin);
        enableInterrupt(channel[i].analogPin, CallbackGlue, CHANGE);
        // isr = this;
        pinMode(channel[i].analogPin, INPUT);
        channel[i].minOut = minOuts[i];
        channel[i].maxOut = maxOuts[i];
        if (!CALIBRATE) {
          channel[i].maxIn = 2000;
          channel[i].minIn = 1000;
        }
      }
    break;

    case PPM:
      enableInterrupt(firstPin, CallbackGlue, CHANGE);
      pinMode(firstPin, INPUT);
      for (int i = 0; i < numChannels; ++i)
      {
          channel[i].minOut = minOuts[i];
          channel[i].maxOut = maxOuts[i];
          if (!CALIBRATE) {
            channel[i].maxIn = 2000;
            channel[i].minIn = 1000;
          }
      }
    break;

    default:
    break;
  };

  if (CALIBRATE) {
    for (int i = 0; i < numChannels; ++i) {
      channel[i].minIn = 1500;
      channel[i].maxIn = 1500;
    }
    Calibrate();
  }
}

void RadioDecoder::PrintIn() {
  for (int i = 0; i < numChannels-1; ++i)
  {
    Serial.print("CH"); Serial.print(i); Serial.print(": ");
    Serial.print(channel[i].rc_values); Serial.print("\t");
  }
  Serial.print("CH"); Serial.print(numChannels-1); Serial.print(": ");
  Serial.println(channel[numChannels-1].rc_values);
}

void RadioDecoder::PrintOut() {
  for (int i = 0; i < numChannels-1; ++i)
  {
    Serial.print("CH"); Serial.print(i); Serial.print(": ");
    Serial.print(channel[i].output); Serial.print("\t");
  }
  Serial.print("CH"); Serial.print(numChannels-1); Serial.print(": ");
  Serial.println(channel[numChannels-1].output);
}

float RadioDecoder::CtrInput(int ch) {
    if (channel[ch].rc_values < channel[ch].minIn)
      channel[ch].rc_values = channel[ch].minIn;
    else if (channel[ch].rc_values > channel[ch].maxIn)
      channel[ch].rc_values = channel[ch].maxIn;

    float Q = abs(channel[ch].maxIn-channel[ch].minIn)/abs(channel[ch].maxOut-channel[ch].minOut);
    channel[ch].output = abs(channel[ch].rc_values-channel[ch].minIn)/Q + channel[ch].minOut;

    return channel[ch].output;
}

void RadioDecoder::rc_read_values() {
  noInterrupts();
  for (int i = 0; i < numChannels; ++i)
    channel[i].rc_values = channel[i].rc_shared;
  interrupts();
}

void RadioDecoder::PWM_Input(uint8_t chIndex, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    channel[chIndex].rc_start = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - channel[chIndex].rc_start);
    channel[chIndex].rc_shared = rc_compare;
  }
}

void RadioDecoder::PPM_Input() {
  if (digitalRead(firstPin) == HIGH) {
    if (!startCh) {
      channel[ChCounter].rc_start = micros();
      startCh = 1;
    }
    else if (startCh) {
      channel[ChCounter+1].rc_start = micros();
      uint16_t rc_compare = channel[ChCounter+1].rc_start - channel[ChCounter].rc_start;
      channel[ChCounter].rc_shared = rc_compare;

      if ((ChCounter == numChannels-1) || (channel[ChCounter].rc_shared > 3000)) {
        channel[0].rc_start = channel[ChCounter+1].rc_start;
        ChCounter = 0;
      }
      else
        ChCounter++;
    }
  }
}

void RadioDecoder::CallbackGlue()
{
  isr->Callback();
}

void RadioDecoder::Callback()
{
  switch (type) {
    case PWM:
      for (int i = 0; i < numChannels; ++i) {
        if (channel[i].status != digitalRead(channel[i].analogPin)) {
          PWM_Input(i, channel[i].analogPin);
          channel[i].status = digitalRead(channel[i].analogPin);
        }
      }
    break;

    case PPM:
      PPM_Input();
    break;

    default:
    break;
  }
}

// Start Calibrate method
void RadioDecoder::Calibrate()
{
  Serial.println(F("\nSend any character to begin calibration of radio deoceder: "));

  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  while(millis() < 50000) {
    rc_read_values();
    for (int i = 0; i < numChannels-1; ++i)
    {
      Serial.print("CH"); Serial.print(i); Serial.print(": ");
      Serial.print(channel[i].rc_values); Serial.print("\t");
    }
    Serial.print("CH"); Serial.print(numChannels-1); Serial.print(": ");
    Serial.println(channel[numChannels-1].rc_values);

    for (int i = 0; i < numChannels; ++i)
    {
      if (channel[i].rc_values > channel[i].maxIn)
        channel[i].maxIn = channel[i].rc_values;
      else if (channel[i].rc_values < channel[i].minIn)
        channel[i].minIn = channel[i].rc_values;
    }
  }

  for (int i = 0; i < numChannels; ++i) {
    Serial.print("Max CH"); Serial.print(i); Serial.print(": "); Serial.print("\t");
    Serial.print(channel[i].maxIn); Serial.print("\t");
    Serial.print("Min CH"); Serial.print(i); Serial.print(": "); Serial.print("\t");
    Serial.println(channel[i].maxIn);
  }

  Serial.println(F("\nSend any character to continue: "));

  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
} // End Calibrate method
