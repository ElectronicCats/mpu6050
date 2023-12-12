// This sample code shows how to use dmpSetFIFOPacketSize() so as not to waste time
// initializaing the MPU6050 when the MCU has been reset or powercycled but the MPU6050 has not.
// Instead we just need to set the FIFOPacketSize, since this is the only setting needed
// that was stored in the MCU RAM.

// This example code was written for the LGT8F328. In this example the LGT8F328 goes into
// a deep sleep for 1 second which erases its ram and thus causes the MCU to restart on wakeup.
// The code is based off a simplication of example code "MPU6050_DMP6".

// Code and comments related to demonstrating how to use dmpSetFIFOPacketSize() are preceded
// by a line of asterisks (***************************************************************).

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <lgt_LowPower.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[42];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint32_t startTime;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  Serial.println(F("Starting setup"));

  // ***************************************************************
  // if MotionDetectionDuration is not the default value (0) but is instead a value we have set previously (1)
  // then we know that the mpu6050 has not been reset or powercycled only the MCU has been, so we don't need
  // to waste time reininitializing the mpu6050. Instead we just have to set the MCU to know the packet size
  // since this value was only retained previously in the MCU RAM:
  if (mpu.getMotionDetectionDuration() == 1) {
    Serial.println("Skipping MPU6050 initialization");
    mpu.dmpSetFIFOPacketSize(42);
    return;
  }

  // ***************************************************************
  // if we got here then the MPU6050 has been power cycled or reset so we have to do a full initialization
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

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

  // ***************************************************************
  // set MotionDetectionDuration to a value other than the default (0).
  // Note that we could set any value other than default in the MPU6050
  // and test for that instead of MotionDetectionDuration. We chose
  // MotionDetectionDuration to set in this example only as an example
  // of how to test for reset/powerup on the MPU6050.
  mpu.setMotionDetectionDuration(1);

  startTime = millis();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if ((millis() - startTime) < 5000) { // show YPR values for 5 seconds
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
    }
  } else {
    // ***************************************************************
    // 5 seconds have passed
    // put the MCU into a deep sleep for 1 second. All RAM is lost
    // however the MPU6050 will continue to operate.
    Serial.println("going to sleep for 1 Sec.");
    delay(100); // give the MCU a chance to print the above message
    LowPower.deepSleep2(SLEEP_1S);
  }
}
