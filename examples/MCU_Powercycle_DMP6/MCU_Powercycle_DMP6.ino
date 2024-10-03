/*
  MCU Powercycle DMP6

  Understand the use of function dmpSetFIFOPacketSize() so the MPU6050 initiates faster
  when the MCU has been reset or powercycled but MPU6050 has not.

  This example is designed for the LGT8F328. The MCU will enter deep sleep for 1 second, erasing
  the RAM, causing the MCU to restart on wakeup. 

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

  created 19 Sep 2023
  by John Harrison
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <lgt_LowPower.h> //Library needed to enter sleep mode, use the one for your board

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*---MPU6050 Control/Status Variables---*/
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[42];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
uint32_t startTime;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  Serial.println(F("Starting setup"));

  /*
  If MotionDetectionDuration is not 0 (default value) and instead a value we defined (1), then is know 
  that the mpu6050 has not been reset or power-cycled only the MCU has been.
  We just have to set the MCU to know the packet size since this value was only retained previously in 
  the MCU RAM.
  */
  if (mpu.getMotionDetectionDuration() == 1) {
    Serial.println("Skipping MPU6050 initialization");
    mpu.dmpSetFIFOPacketSize(42);
    return;
  }

  /* Full initialization if the MPU6050 was powercycled */
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  /*
  Setting MotionDetectionDuration to a value other than the default (0).
  Note that we could set any value other than default and test for that 
  instead of MotionDetectionDuration. We chose MotionDetectionDuration 
  to set in this example only as an example of how to test for reset/powerup 
  on the MPU6050.
  */
  mpu.setMotionDetectionDuration(1);
  startTime = millis();
}

void loop() {
  if ((millis() - startTime) < 5000) { // Print YPR values for 5 seconds
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
    }
  }
  else {
    /*
    5 seconds have passed, put the MCU into a deep sleep for 1 second. 
    All RAM is erased however the MPU6050 will continue to operate.
    */
    Serial.println("Going to sleep for 1 Sec.");
    delay(100);         // Give the MCU a chance to print the above message
    LowPower.deepSleep2(SLEEP_1S); //Function from low power library to enter deep sleep
  }
}
