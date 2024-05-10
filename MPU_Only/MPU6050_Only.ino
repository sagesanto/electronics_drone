// 3/15/2024 by Sage Santomenna
// this program reads values from an attached MPU6050 and outputs them over serial using a version of the MIN protocol

// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net> https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Added crc32 checksum implementation by Christopher Baker (https://github.com/bakercp/CRC32)
#include <CRC32.h>

//these repositories are licensed under the MIT license, also included in /licenses/
/* ============================================
MIT license
Copyright (c) 2012 Jeff Rowberg
Copyright (c) 2017 Christopher Baker https://christopherbaker.net

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <SPI.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)

#define LED_PIN LED_BUILTIN
bool blinkState = false;
int blinkTimer = 0;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         quaternion container

void readFifoBuffer_() {
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

// Sage's awesome MIN implementation
// 0x01 and 0x7F are reserved characters
// instead of doing 0x55 as an exit character, we'll use 0x7F. we won't do byte stuffing
// | 0x01 | 0x01 | 0x01 | ID | len |     Content    | C1 | C2 | C3 | C4 | 0x7F |
// |      Start (3)     | 1  |  1  |  (len : 0-255) |     Checksum (4)  |  End |
// checksum algorithm is CRC32 (ISO HDLC)

void minify(int id, String msg, uint8_t* buffer, size_t buf_size) {
  int START_CHAR = 0x01;
  int END_CHAR = 0x7F;
  buffer[0] = START_CHAR;
  buffer[1] = START_CHAR;
  buffer[2] = START_CHAR;
  buffer[3] = id;
  buffer[4] = msg.length();
  msg.getBytes(buffer + 5, buf_size - 5);
  int j = 5 + msg.length();
  uint32_t checksum = CRC32::calculate(buffer, j);
  buffer[j++] = (checksum >> 24) & 0xFF;
  buffer[j++] = (checksum >> 16) & 0xFF;
  buffer[j++] = (checksum >> 8) & 0xFF;
  buffer[j++] = checksum & 0xFF;
  buffer[j++] = END_CHAR;
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(57600);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own offsets here
  // can be obtained from MPU6050 calibration by programs like https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino
  mpu.setXGyroOffset(117);
  mpu.setYGyroOffset(53);
  mpu.setZGyroOffset(19);
  mpu.setZAccelOffset(1788); 

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(15);
    mpu.CalibrateGyro(15);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void min_write(String msg) {
  int buf_size = msg.length() + 10;
  uint8_t buffer[buf_size];
  minify(0, msg, buffer, buf_size);
  Serial.write(buffer, buf_size);
  Serial.println();
}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // blink LED to indicate activity
  if (blinkTimer / 5) {
    blinkTimer = 0;
    digitalWrite(LED_PIN, blinkState);
    blinkState = !blinkState;
  }
  blinkTimer++;

  readFifoBuffer_();

  String msg = "";

  // display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  msg = String(q.w) + " " + String(q.x) + " " + String(q.y) + " " + String(q.z);

  min_write(msg);
}
