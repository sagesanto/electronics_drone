/*
Warning: operation of a home-made drone can lead to serious injury or property damage. 
Software flaws may be the cause or a contributing factor in such incidents. The code provided
and detailed here does not cause the drone to operate in a completely safe or risk-free manner. 
The code provided here may also allow undesired/unanticipated behavior and not all edge cases havebeen thoroughly tested. 
USE AT YOUR OWN RISK.
Make sure you understand the code and this warning before operation.
*/

#include <SPI.h>
#include <Servo.h>

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


// ===================== erratta ============================= 
int UPPER_LIMIT = 180; // DANGER do not change this, it won't do what you think
int THROTTLE = 105; // this is the default val

int UPPER_PID_LIMIT = 115; // THIS is the max the motors are allowed to run at

int PID_RANGE = UPPER_PID_LIMIT - THROTTLE; // don't touch

// this factor is intended to compensate for the fact that the motors are not equidistant from the center of mass
// (shorter arm length/longer arm length) + (1-(shorter arm length/longer arm length))/2
float DIST_FACTOR = (8.0/14.0) + (1-(8.0/14.0))/2.0;

float P;
float I;
float D;

//drone controller ====================================

// this contains drone stuff and mpu stuff

// human-readable: print so humans can read
// #define HUMAN_READABLE

// machine-readable: print so graphing can read
#define MACHINE_READABLE

double x_target = 0.0;
double y_target = 0.0;

const double X_SENSITIVITY = 0.001;
const double Y_SENSITIVITY = 0.001;

// mpu ======================================

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define OUTPUT_OVER_SERIAL

MPU6050 mpu;

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

  Quaternion q;

// drone motors ====================================
Servo ESC_FCW;     // create servo object to control the ESC
Servo ESC_BCW;
Servo ESC_FCCW;
Servo ESC_BCCW;

// drone control setup
ArduPID PIDControl_FCW;
ArduPID PIDControl_BCW;
ArduPID PIDControl_FCCW;
ArduPID PIDControl_BCCW;

// =================== UTILITY =======================================

// drone control ======================
//PID-controlled motor class
class SmartMotor {
  private:
    Servo ESC;
    ArduPID& pid;
  private:
    double count{0};
    double prevCount{0};
    double target{0};
    bool hasTarget{false};
    bool usingPid{false};
    double pidInput;
    double pidOutput;
    double pidTarget;
    double kP = 200;
    double kI = 0;
    double kD = 0;
    double threshold = 0.001; // distance threshold for considering the target reached
  public:
    SmartMotor(ArduPID& pid, Servo ESC) : pid{pid}, ESC{ESC} {}
    bool refresh(double count);
    void reset();
    double getTarget() {
      return target;
    }
    double getCount() {
      return count;
    }
    void setTarget(double targ);
    void clearTarget();
    bool getHasTarget() { return hasTarget; }
    double dist() {
      return target - count;
    }
    double absDist() {
      return abs(target - count);
    }
    bool reachedTarget(double threshold) {
      return absDist() <= threshold;
    }
    int  computeMotorSpeed();
    bool isUsingPid() {
      return usingPid;
    }
    void SmartMotor::setP(double P)
    {
      kP = P;
    }

    void SmartMotor::setI(double I)
    {
      kI = I;
    }

    void SmartMotor::setD(double D)
    {
      kD = D;
    }

  private:
    int  fixMotorSpeedSign(int motorSpeed) {
      return (target < count) ? -motorSpeed : motorSpeed;
    }
    void enablePid();
    void disablePid();
};

bool SmartMotor::refresh(double newCount)
{
  count = newCount;
  if (count != prevCount) {
    prevCount = count;
    return true;
  }
  return false;
}

void SmartMotor::reset()
{
  count = 0;
  prevCount = 0;
  clearTarget();
  disablePid();
}


void SmartMotor::enablePid()
{
  if (!usingPid) {
    usingPid = true;
    pidInput  = count;
    pidTarget = target;
    pid.begin(&pidInput, &pidOutput, &pidTarget, kP, kI, kD);
    pid.setOutputLimits(-PID_RANGE, PID_RANGE);
    // pid.setWindUpLimits(-20, 20); // need to tune this
  }
}

void SmartMotor::disablePid()
{
  if (usingPid) {
    usingPid = false;
    pid.stop();
  }
}

void SmartMotor::setTarget(double targ)
{
  target = targ;
  hasTarget = true;
}

void SmartMotor::clearTarget()
{
  target = count;
  hasTarget = false;
}

int SmartMotor::computeMotorSpeed()
{
  if (reachedTarget(threshold)) {
    disablePid();
    return 0;
  }

  enablePid();
  pid.compute();
  
  return pidOutput;
}

// MPU and MIN utility =============================
// read buffer from MPU
void readFifoBuffer_() {
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

// Sage's awesome MIN implementation (see /license/MIN_license.txt for original version's license)

// instead of doing 0x55 as an exit character, we'll use 0x7F. we won't do byte stuffing
// | 0x01 | 0x01 | 0x01 | ID | len |     Content    | C1 | C2 | C3 | C4 | 0x7F |
// |      Start (3)     | 1  |  1  |  (len : 0-255) |     Checksum (4)  |  End |
// checksum algorithm is CRC32 (ISO HDLC)

// MINify packet
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

// Sage
void min_write(String msg) {
  int buf_size = msg.length() + 10;
  uint8_t buffer[buf_size];
  minify(0, msg, buffer, buf_size);
  Serial.write(buffer, buf_size);
  Serial.println();
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}


void setup() {
  //drone motors =============================

  ESC_FCW.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC_BCW.attach(3,1000,2000);
  ESC_FCCW.attach(5,1000,2000);
  ESC_BCCW.attach(6,1000,2000);

  #ifdef OUTPUT_OVER_SERIAL
    Serial.begin(38400);
  #endif
  // initialize the motors. at this point in the code, the battery should NOT be plugged in!
  Serial.println("Initializing speed controller...");
  ESC_FCW.write(UPPER_LIMIT);
  ESC_BCW.write(UPPER_LIMIT);
  ESC_FCCW.write(UPPER_LIMIT);
  ESC_BCCW.write(UPPER_LIMIT);

  Serial.println("Send any character over serial as soon as the battery is plugged in");
  while (Serial.available() <= 0) {}
  Serial.read();
  Serial.println("Holding high...");
  delay(3000);

  Serial.println("Holding low...");
  ESC_FCW.write(0);    // Send the signal to the ESC
  ESC_BCW.write(0);    
  ESC_FCCW.write(0);    
  ESC_BCCW.write(0);    
  delay(10000);

  Serial.println("Speed controller initialized.");
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif


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
  
  // pulse the motors as a 3-second warning 

  ESC_FCW.write(5);
  ESC_BCW.write(5);
  ESC_FCCW.write(5);
  ESC_BCCW.write(5);

  delay(500);

  ESC_FCW.write(0);
  ESC_BCW.write(0);
  ESC_FCCW.write(0);
  ESC_BCCW.write(0);

  delay(3000);


  int rampup = 0;
  while(rampup <= THROTTLE)
   {
    ESC_FCW.write(rampup);
    ESC_BCW.write(rampup);
    ESC_FCCW.write(rampup);
    ESC_BCCW.write(rampup);
    rampup += 5;
    delay(500);
  }
  
}

SmartMotor FCW(PIDControl_FCW,ESC_FCW);
SmartMotor BCW(PIDControl_BCW,ESC_BCW);
SmartMotor FCCW(PIDControl_FCCW,ESC_FCCW);
SmartMotor BCCW(PIDControl_BCCW,ESC_BCCW);

void updatey(double y)
{
  FCW.refresh(y);
  BCW.refresh(-y);
}
void updatex(double x)
{
  FCCW.refresh(-x);
  BCCW.refresh(x);
}
void setyTarget(double y)
{
  FCW.setTarget(y);
  BCW.setTarget(y);
}
void setxTarget(double x)
{
  FCCW.setTarget(x);
  BCCW.setTarget(x);
}

void loop() {

  if(Serial.available() > 0)
  {
    Serial.println("Stopping...");
    ESC_FCW.write(0);
    ESC_BCW.write(0);
    ESC_FCCW.write(0);
    ESC_BCCW.write(0);
    delay(1000);
    return;
  }

  // ======== measure orientation ============
  // if programming the mpu failed, don't try to do anything
  if (!dmpReady) return;

  // blink LED to indicate activity 
  if (blinkTimer / 5) {
    blinkTimer = 0;
    digitalWrite(LED_PIN, blinkState);
    blinkState = !blinkState;
  }
  blinkTimer++;

  readFifoBuffer_();  // Sage 3/24

  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // measure current pos 
  double current_x = q.x;
  double current_y = q.y;
  // tell the pid/motors about our updated pos
  updatex(current_x);
  updatey(current_y);

  double desired_x = 0.0; // eventually this should be taken from glove input

  if(abs(desired_x - x_target) > X_SENSITIVITY) {
    setxTarget(desired_x);
    x_target = desired_x;
  }  
  
  double desired_y = 0.0; // eventually this should be taken from glove input

  if(abs(desired_y - y_target) > Y_SENSITIVITY) {
    setyTarget(desired_y);
    y_target = desired_y;
  }

  int fcw_speed = FCW.computeMotorSpeed();
  int bcw_speed = int(DIST_FACTOR*BCW.computeMotorSpeed());
  int fccw_speed = FCCW.computeMotorSpeed();
  int bccw_speed = int(DIST_FACTOR*BCCW.computeMotorSpeed());

  fcw_speed = min(THROTTLE+fcw_speed,UPPER_PID_LIMIT);
  bcw_speed = min(THROTTLE+bcw_speed,UPPER_PID_LIMIT);
  fccw_speed = min(THROTTLE+fccw_speed,UPPER_PID_LIMIT);
  bccw_speed = min(THROTTLE+bccw_speed,UPPER_PID_LIMIT);

  # ifdef HUMAN_READABLE
    Serial.println("x: "+String(q.x)+", y: "+String(q.y));
    Serial.print("Speed: ");
    Serial.print(" FCW: ");
    Serial.print(String(fcw_speed));
    Serial.print(" BCW: ");
    Serial.print(String(bcw_speed));
    Serial.print(" FCCW: ");
    Serial.print(String(fccw_speed));
    Serial.print(" BCCW: ");
    Serial.println(String(bccw_speed));
  #endif

  #ifdef MACHINE_READABLE
    Serial.print("# ");
    Serial.print(String(desired_x)); Serial.print(" ");
    Serial.print(String(desired_y)); Serial.print(" ");
    Serial.print(String(q.x)); Serial.print(" ");
    Serial.print(String(q.y)); Serial.print(" ");
    Serial.print(String(fcw_speed)); Serial.print(" ");
    Serial.print(String(bcw_speed)); Serial.print(" ");
    Serial.print(String(fccw_speed)); Serial.print(" ");
    Serial.print(String(bccw_speed)); Serial.print(" ");
    Serial.println("%");
  #endif

  ESC_FCW.write(fcw_speed);
  ESC_BCW.write(bcw_speed);
  ESC_FCCW.write(fccw_speed);
  ESC_BCCW.write(bccw_speed);

}

