// ======== MPU =============
#include <SPI.h>

// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net> https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
// Added crc32 checksum implementation by Christopher Baker (https://github.com/bakercp/CRC32)
#include <CRC32.h>
// uses the ArduPID library by PB2 (https://github.com/PowerBroker2/ArduPID/tree/main)
#include <ArduPID.h>

//these repositories are licensed under the MIT license, also included in /licenses/
/* ============================================
Copyright (c) 2012 Jeff Rowberg
Copyright (c) 2017 Christopher Baker https://christopherbaker.net
Copyright (c) 2021 PB2

MIT License
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

// Lora Transmitter, port 14101
#include <SoftwareSerial.h>
int i = 0;
const int SELF_ADDR = 1;
const int OTHER_ADDR = 0;
SoftwareSerial lora(4,7); //TXD port,RXD port
//send with Lora
void send(int addr, String msg)
{
  lora.write(("AT+SEND="+String(addr)+","+String(msg.length())+","+msg+"\r\n").c_str());
  delay(100);
}

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define OUTPUT_OVER_SERIAL
// #define OUTPUT_OVER_RADIO
MPU6050 mpu;

#define OUTPUT_READABLE_QUATERNION
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
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container

// Sage
void readFifoBuffer_() {
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}
// Sage's awesome MIN implementation
// instead of doing 0x55 as an exit character, we'll use 0x7F. we won't do byte stuffing
// | 0x01 | 0x01 | 0x01 | ID | len |     Content    | C1 | C2 | C3 | C4 | 0x7F |
// |      Start (3)     | 1  |  1  |  (len : 0-255) |     Checksum (4)  |  End |
// checksum algorithm is CRC32 (ISO HDLC)
// Sage
void minify(int id, String msg, uint8_t* buffer, size_t buf_size) {
  int START_CHAR = 0x01;
  int END_CHAR = 0x7F;
  // content is null-terminated!
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
// FSR ============================================
/*
Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground
For more information see www.ladyada.net/learn/sensors/fsr.html */
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
double fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
double fsrConductance;
double fsrForce;       // Finally, the resistance converted to force
void setup(void) {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  // GYRO SETUP
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
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
  lora.begin(9600);
  delay(1000);
  lora.println("AT");
  delay(1000);
  lora.println("AT");
  String b=lora.readString();
  Serial.println(b);
  delay(1000);
}
// Sage
void min_write(String msg, int id) {
  int buf_size = msg.length() + 10;
  uint8_t buffer[buf_size];
  minify(id, msg, buffer, buf_size);
  Serial.write(buffer, buf_size);
  Serial.println();
}
int loop_count = 0;
void loop(void) {
  fsrReading = analogRead(fsrPin);
  // Serial.print("Analog reading = ");
  // Serial.println(fsrReading);
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  // Serial.print("Voltage reading in mV = ");
  // Serial.println(fsrVoltage);
  if (fsrVoltage == 0) {
    fsrForce = 0;
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    // Serial.print("FSR resistance in ohms = ");
    // Serial.println(fsrResistance);
    fsrConductance = 1000000;           // we measure in microohms so
    fsrConductance /= fsrResistance;
    // Serial.print("Conductance in microohms: ");
    // Serial.println(fsrConductance);
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      // Serial.print("Force in Newtons: ");
      // Serial.println(fsrForce);
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      // Serial.print("Force in Newtons: ");
      // Serial.println(fsrForce);
    }
  }
  // Serial.println("--------------------");
  String fsr_msg = String(fsrForce);
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // blink LED to indicate activity
  if (blinkTimer / 5) {
    blinkTimer = 0;
    //digitalWrite(LED_PIN, blinkState);
    blinkState = !blinkState;
  }
  blinkTimer++;
  readFifoBuffer_();
  String mpu_msg = "";
  #ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu_msg = String(q.w) + " " + String(q.x) + " " + String(q.y) + " " + String(q.z);
  #endif

  #ifdef OUTPUT_OVER_SERIAL
    min_write(mpu_msg, 0);
    min_write(fsr_msg, 1);
  #endif
  send(0, mpu_msg);
  send(0, fsr_msg);
 while(lora.available()>0){
    byte b=lora.read();
    Serial.write(b);
  }
}