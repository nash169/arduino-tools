#include "imu.h"

// #define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

IMU *IMU::isr;

// Constructor
IMU::IMU(uint8_t mode) {
  modeIMU = mode;

  switch (modeIMU) {
    case DMP:
      mpu = new MPU6050_6;
      dmpState = new dmp_IMU;

      LED_PIN = 13;         // (Arduino is 13, Teensy is 11, Teensy++ is 6)
      blinkState = false;

      dmpReady = false;     // set true if DMP init was successful
      mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
      INTERRUPT_PIN = 2;
    break;

    case RAW:
      mpu_address = 0x68; // I2C address of the MPU-6050
      rawState = new raw_IMU;
    break;

    default:
    break;
  }
}

// Default Constructor
IMU::IMU() : IMU(DMP) { }

// Destructor
IMU::~IMU() {
  delete mpu;
}

void IMU::Init() {
  switch (modeIMU) {
    case DMP:
      isr = this;

      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
      #endif

      // initialize device
      mpu->initialize();
      pinMode(INTERRUPT_PIN, INPUT);
      // verify connection
      Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      // load and configure the DMP
      devStatus = mpu->dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu->setXGyroOffset(220);
      mpu->setYGyroOffset(76);
      mpu->setZGyroOffset(-85);
      mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu->setDMPEnabled(true);
        // enable Arduino interrupt detection
        enableInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), CallbackGlue, RISING);
        mpuIntStatus = mpu->getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu->dmpGetFIFOPacketSize();
      } else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      }
      // configure LED for output
      pinMode(LED_PIN, OUTPUT);
    break;

    case RAW:
      // Wake up MPU
      Wire.begin();
      Wire.beginTransmission(mpu_address);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);

      delay(100);
    break;

    default:
    break;
  }
}

void IMU::Run() {
  switch (modeIMU) {
    case DMP:
      // if programming failed, don't try to do anything
      if (!dmpReady) return;
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu->getIntStatus();
      // get current FIFO count
      fifoCount = mpu->getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu->resetFIFO();
        Serial.println(F("FIFO overflow!"));
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu->getFIFOCount();
        // read a packet from FIFO
        mpu->getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get quaternions
        mpu->dmpGetQuaternion(&dmpState->q, fifoBuffer);
        // Get gravity
        mpu->dmpGetGravity(&dmpState->gravity, &dmpState->q);
        // Get linear accelerations
        mpu->dmpGetAccel(&dmpState->aa, fifoBuffer);
        // Get Euler angles
        mpu->dmpGetEuler(dmpState->euler, &dmpState->q);
        // Get yaw, pitch and roll
        mpu->dmpGetYawPitchRoll(dmpState->ypr, &dmpState->q, &dmpState->gravity);
        // Get real accelerations in Body
        mpu->dmpGetLinearAccel(&dmpState->aaReal, &dmpState->aa, &dmpState->gravity);
        // Get real accelerations in World
        mpu->dmpGetLinearAccelInWorld(&dmpState->aaWorld, &dmpState->aaReal, &dmpState->q);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }
    break;

    case RAW:
      Wire.beginTransmission(mpu_address);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(mpu_address,14,true);  // request a total of 14 registers
      rawState->linearAcc[0] = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      rawState->linearAcc[1] = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      rawState->linearAcc[2] = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      rawState->temp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      rawState->angularVel[0] = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      rawState->angularVel[1] = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      rawState->angularVel[2] = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    break;

    default:
    break;
  }

}

float IMU::SetGyroSens(uint8_t config) {
  // Set Gyro sensitivity
  Wire.beginTransmission(mpu_address);
  Wire.write(0x1B);
  Wire.write(B0000000); // +/- 500°/sec -> 65.5
  Wire.endTransmission(true);

  delay(100);

  gyro_sensitivity = 65.536;

  return gyro_sensitivity;
}

float IMU::SetAccSens(uint8_t config) {
  // Set Acc sensitivity
  Wire.beginTransmission(mpu_address);
  Wire.write(0x1C);
  Wire.write(B00001000); // +/- 4g -> 8192
  Wire.endTransmission(true);

  delay(100);

  acc_sensitivity = 8192.0;

  return acc_sensitivity;
}

void IMU::CallbackGlue() {
  isr->dmpDataReady();
}

void IMU::dmpDataReady() {
  mpuInterrupt = true;
}
