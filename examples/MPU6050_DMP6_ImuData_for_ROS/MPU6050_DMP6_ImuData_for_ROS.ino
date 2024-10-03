/*
  MPU6050 DMP6 for ROS

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes auto-calibration and offsets generator tasks. Different 
  output formats available.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
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

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
}

void loop() {
  if (!DMPReady) return;

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);

    mpu.dmpGetGravity(&gravity, &q);

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
    Serial.print("\t");
    Serial.print(aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
    Serial.print("\t");
    Serial.println(aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    Serial.print("ggWorld\t");
    Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
    Serial.print("\t");
    Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
    Serial.print("\t");
    Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);

    /* Display Euler angles in degrees */
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(ypr[1] * RAD_TO_DEG);
    Serial.print("\t");
    Serial.println(ypr[2] * RAD_TO_DEG);
    
    Serial.println();
    delay(100);
  }
}
