# Arduino Tools
Repository containing some of the tools I wrote for arduino.

## IMU Sensor
Arduino library to get either RAW or DMP processed data from the MPU6050 sensor.

This library the standard library "Wire" to get unprocessed (raw) data from the MPU6050. It takes advantage of the modified MPU6050 library (https://github.com/nash169/MPU6050) to give back the DMP processed IMU sensos data.

## Radio Decoder
Arduino library for decoding RX radio signals

This library is made to control flying model with Arduino using the following connection chain: 
Radio(TX) -> Receiver(RX) -> Arduino -> Motors(ESC/Servo).
With this package Arduino can decode the pilot signals from the RX and either using them directly to control the model or combining the with an automatic control signal. It also includes the possibility of mapping values from the original signal to a user defined space.

Supported standard:
- PWM
- PPM

## Motor Control
Arduino library to handle output signal towards actuation system in flying model (ESC/Servos)

This library is made to control with Arduino to control multiple Servos/ESC. It also includes the possibility of mapping data to a control space suitable for a given actuation system.

## MPU6050
This a modified version of @jrowberg MPU6050 library (https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050).

This MPU6050 library version allows to avoid "multiple definition" errors during the Arduino project's compilation when you include this library within another one. Both "MPU6050_6Axis_MotionApps20.h" and "MPU6050_9Axis_MotionApps20.h" have been transformed in two classes that are sons of "MPU6050.h/MPU6050.cpp" class.