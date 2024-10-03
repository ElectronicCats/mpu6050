/* 
  MPU6050 DMP6 ESPWiFi

  The board reads quaternion data from the MPU6050 and sends Open Sound
  Control messages.

  This example requires the WiFiManager and OSCMEssage libraries available here:
  - https://github.com/tzapu/WiFiManager
  - https://github.com/CNMAT/OSC 

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes auto-calibration and offsets generator tasks. Different 
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the board's
  external interrupt #0 pin.
    
  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

#if defined(ESP8266) 
#include <ESP8266WiFi.h> // Include this library if using an ESP8266 based board
#else
#include <WiFi.h> //Otherwise use the included WiFi library
#endif

#include <DNSServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <WiFiManager.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming 
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if there is no magnetometer present.

-  Use "OUTPUT_TEAPOT_OSC" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_TEAPOT_OSC
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

#ifdef OUTPUT_READABLE_EULER
float euler[3];         // [psi, theta, phi]    Euler angle container
#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
#endif

#define INTERRUPT_PIN 15 //define the INT pin on your board

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t FIFOCount;     // Count of all bytes currently in FIFO
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector

const char DEVICE_NAME[] = "mpu6050";

WiFiUDP UDP;                                // A  instance to let us send and receive packets over 
const IPAddress outIp(192, 168, 1, 11);     // Remote IP to receive OSC
const unsigned int outPort = 9999;          // Remote port to receive OSC

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void ICACHE_RAM_ATTR DMPDataReady() {
  MPUInterrupt = true;
}

void mpu_setup(){
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize device
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

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}

void mpu_loop()
{
  if (!DMPReady) return; // Stop the program if DMP programming fails.
  if (!MPUInterrupt && FIFOCount < packetSize) return; // Wait for MPU interrupt or extra packet(s) available

  /*Reset interrupt flag and get INT_STATUS byte*/
  MPUInterrupt = false;
  MPUIntStatus = mpu.getIntStatus();

  FIFOCount = mpu.getFIFOCount();  // Get current FIFO count

  /*Check for overflow (this should never happen unless our code is too inefficient)*/
  if ((MPUIntStatus & 0x10) || FIFOCount == 1024) {
    mpu.resetFIFO();     //Reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
    //*Otherwise, check for DMP data ready interrupt (this should happen frequently)*/
  } 
  else if (MPUIntStatus & 0x02) {
    while (FIFOCount < packetSize) FIFOCount = mpu.getFIFOCount();    // Wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(FIFOBuffer, packetSize);    //Read a packet from FIFO
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    FIFOCount -= packetSize;
    #ifdef OUTPUT_READABLE_QUATERNION
      /* Display Quaternion values in easy matrix form: [w, x, y, z] */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    #endif

    #ifdef OUTPUT_TEAPOT_OSC
      #ifndef OUTPUT_READABLE_QUATERNION
        /* Display Quaternion values in easy matrix form: [w, x, y, z] */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
      #endif
      /*Send OSC message*/
      OSCMessage msg("/imuquat");
      msg.add((float)q.w);
      msg.add((float)q.x);
      msg.add((float)q.y);
      msg.add((float)q.z);

      UDP.beginPacket(outIp, outPort);
      msg.send(UDP);
      UDP.endPacket();
      msg.empty();
    #endif

    #ifdef OUTPUT_READABLE_EULER
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      /* Display real acceleration, adjusted to remove gravity */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      /* Display initial world-frame acceleration, adjusted to remove gravity
      and rotated based on known orientation from Quaternion */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
    #endif
  }
}

void setup(){
  Serial.begin(115200); //115200 is required for Teapot Demo output
  Serial.println(F("\nOrientation Sensor OSC output")); Serial.println();

  /*WiFiManager*/
  WiFiManager wifiManager; // Initializate WiFi Manager
  //wifiManager.resetSettings();    //Reset saved settings

  /*Fetches ssid and pass from eeprom and tries to connect
  if it does not connect it starts an access point with the specified name
  and goes into a blocking loop awaiting configuration*/
  wifiManager.autoConnect(DEVICE_NAME);

  Serial.print(F("WiFi connected! IP address: "));
  Serial.println(WiFi.localIP());

  mpu_setup();
}

void loop(){
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("*** Disconnected from AP, rebooting ***");
    Serial.println();
    #if defined(ESP8266)
    ESP.reset();
    #else
    ESP.restart();
    #endif
  }
  mpu_loop();
}
