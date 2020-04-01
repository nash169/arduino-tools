#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#include <Wire.h>
#include "MPU6050_6Axis.h"

struct dmp_IMU {
  Quaternion q;           // [w, x, y, z]         quaternion container

  VectorInt16 aa,         // [x, y, z]            accel sensor measurements
              aaReal,     // [x, y, z]            gravity-free accel sensor measurements
              aaWorld;    // [x, y, z]            world-frame accel sensor measurements

  VectorFloat gravity;    // [x, y, z]            gravity vector

  float euler[3],         // [psi, theta, phi]    Euler angle container
        ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

};

struct raw_IMU {
  uint16_t linearAcc[3],   // Raw linear accelerations from IMU
           temp,        // Raw temperature from IMU
           angularVel[3];  // Raw angular velocities from IMU
};

class IMU
{
public:
  IMU(uint8_t mode);
  IMU();
  virtual ~IMU();

  void Init();

  inline dmp_IMU* GetDMP () { return dmpState; }
  inline raw_IMU* GetRAW () { return rawState; }

  void Run();

  float SetGyroSens(uint8_t config);
  float SetAccSens(uint8_t config);

protected:

private:
  uint8_t modeIMU;

  enum {
    DMP,
    RAW
  };

  /*==RAW IMU==*/
  static void CallbackGlue();
  void dmpDataReady();
  static IMU *isr;

  int mpu_address;

  raw_IMU *rawState;

  float acc_sensitivity,
        gyro_sensitivity;

  /*==DMP IMU==*/
  MPU6050_6 *mpu;

  bool blinkState,
       dmpReady;

  volatile bool mpuInterrupt;

  uint8_t LED_PIN,
          INTERRUPT_PIN,
          mpuIntStatus,   // holds actual interrupt status byte from MPU
          devStatus,      // return status after each device operation (0 = success, !0 = error)
          fifoBuffer[64]; // FIFO storage buffer

  uint16_t packetSize,    // expected DMP packet size (default is 42 bytes)
           fifoCount;     // count of all bytes currently in FIFO

  dmp_IMU *dmpState;
};

#endif // IMU_H
