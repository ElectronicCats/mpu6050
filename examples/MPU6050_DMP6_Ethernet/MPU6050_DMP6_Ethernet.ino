/* 
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  Send the data through an HTTP request using the Arduino Ethernet Shield

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin. 
    
  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/
#include <Ethernet.h>   // Arduino Ethernet Shield
#include "avr/wdt.h"    // Watchdog library

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*Ethernet Shield variables definition*/
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0x26, 0x82};  // MAC address from Ethernet shield sticker under board
IPAddress ip(192,168,1,50);                         // Assign an IP address
byte gateway[] = { 192, 168, 1, 1 };                // Router's gateway address
byte subnet[] = { 255, 255, 0, 0 };                 // Subnet
String HTTP_req;                                    // Stores the HTTP request

/*Initialize the Ethernet server library (port 80 is default for HTTP)*/
EthernetServer server(80);

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

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
uint16_t FIFOCount;     // Count of all bytes currently in FIFO

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup() {
  wdt_enable(WDTO_1S); //WDTO_1S sets the watchdog timer to 1 second. The time set here is approximate.

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //115200 is required for Teapot Demo output

  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  Serial.print("Server is at ");
  Serial.println(Ethernet.localIP());
  while (!Serial);

  /*Initialize device*/
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
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    /*Calibration routine*/
    //mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateGyro(6);
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
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.

  wdt_reset();           // Resets the watchdog timer. If the timer is not reset, and the timer expires, a watchdog-initiated device reset will occur.
  /*Wait for MPU interrupt or extra packet(s) available*/
  while (!MPUInterrupt && FIFOCount < packetSize) {
    if (MPUInterrupt && FIFOCount < packetSize) {
      FIFOCount = mpu.getFIFOCount();    // Try to get out of the infinite loop 
    }  
    /*  Write your code here        
      If you are really paranoid you can frequently test in between other
      stuff to see if MPUInterrupt is true, and if so, "break;" from the
      while() loop to immediately process the MPU data
      */
  }

  MPUInterrupt = false;           // Reset interrupt flag and get INT_STATUS byte
  MPUIntStatus = mpu.getIntStatus();
  FIFOCount = mpu.getFIFOCount(); // Get current FIFO count

  /*Check for overflow (this should never happen unless our code is too inefficient)*/
  if ((MPUIntStatus & (1  <<  MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || FIFOCount >= 1024) {
    mpu.resetFIFO();    // Reset so we can continue cleanly
    FIFOCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } 
  /*Otherwise, check for DMP data ready interrupt (this should happen frequently)*/
  else if (MPUIntStatus & (1  <<  MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (FIFOCount < packetSize) FIFOCount = mpu.getFIFOCount();  //Wait for the correct available data length, should be a VERY short wait
      mpu.getFIFOBytes(FIFOBuffer, packetSize);   // Read a packet from FIFO, then clear the buffer
      //mpu.resetFIFO();
      /*Track FIFO count here in case there is > 1 packet available
      (this lets us immediately read more without waiting for an interrupt)*/
      FIFOCount -= packetSize;

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
  
      #ifdef OUTPUT_TEAPOT
        /* Display quaternion values in InvenSense Teapot demo format */
        teapotPacket[2] = FIFOBuffer[0];
        teapotPacket[3] = FIFOBuffer[1];
        teapotPacket[4] = FIFOBuffer[4];
        teapotPacket[5] = FIFOBuffer[5];
        teapotPacket[6] = FIFOBuffer[8];
        teapotPacket[7] = FIFOBuffer[9];
        teapotPacket[8] = FIFOBuffer[12];
        teapotPacket[9] = FIFOBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // PacketCount, loops at 0xFF on purpose
      #endif
    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
}

void serversend(){
  EthernetClient client = server.available();  // Try to get client
  if (client) { 
    //boolean currentLineIsBlank = true;
    while (client.connected()){
      if (client.available()) {   // Client data available to read
        char c = client.read(); // Read 1 byte (character) from the client
        HTTP_req += c;          // Save the HTTP request 1 char at a time
        /*  
          Last line of client request is blank and ends with \n
          respond to client only after last line received
        */
        if (c == '\n') {
          client.println("HTTP/1.1 200 OK");  //  Send a standard http response header
          client.println("Content-Type: text/html");
          //client.println("Connection: keep-alive");
          client.println();
          // AJAX request for switch state
          if (HTTP_req.indexOf("ajax_switch") > -1) {
            GetAjaxData(client);    //  Read switch state and analog input
          }
          else {  // HTTP request for web page
            /*Send web page - contains JavaScript with AJAX calls*/
            client.println("<!DOCTYPE html>");
            client.println("<html>");
            client.println("<head>");
            client.println("<title>Arduino Web Page</title>");
            client.println("<script>");
            client.println("function GetAjaxData() {");
            client.println("nocache = \"&nocache=\" + Math.random() * 1000000;");
            client.println("var request = new XMLHttpRequest();");
            client.println("request.onreadystatechange = function() {");
            client.println("if (this.readyState == 4) {");
            client.println("if (this.status == 200) {");
            client.println("if (this.responseText != null) {");
            client.println("document.getElementById(\"sw_an_data\")\.innerHTML = this.responseText;");
            client.println("}}}}");
            client.println(
            "request.open(\"GET\", \"ajax_switch\" + nocache, true);");
            client.println("request.send(null);");
            client.println("setTimeout('GetAjaxData()', 10);");
            client.println("}");
            client.println("</script>");
            client.println("</head>");
            client.println("<body onload=\"GetAjaxData()\">");
            client.println("<h1>MPU6050 Output</h1>");
            client.println("<div id=\"sw_an_data\">");
            client.println("</div>");
            client.println("</body>");
            client.println("</html>");
          }
          Serial.print(HTTP_req); // Display received HTTP request on serial port
          HTTP_req = "";          // Finished with the request, empty string
          client.stop();          // Close the connection
          break;
        }              
      }
    }      
  } 
}

void GetAjaxData(EthernetClient cl){
  #ifdef OUTPUT_READABLE_QUATERNION
    // Display quaternion values in easy matrix form: w x y z
    cl.print("Quaternion Values:\t");
    cl.print("<p>w:");
    cl.print(q.w); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>x:");
    cl.print(q.x); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>y:");
    cl.print(q.y); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>z:");
    cl.print(q.z); 
    cl.print("\t");
    cl.println("</p>");
  #endif
  #ifdef OUTPUT_READABLE_EULER
    // Display Euler angles in degrees
    cl.print("Euler Angles:\t");
    cl.print("<p>Yaw:");
    cl.print(euler[0] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Pitch:");
    cl.print(euler[2] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Roll:");
    cl.print(euler[1] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
  #endif
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // Display Yaw/Pitch/Roll values in degrees
    cl.print("Yaw, Pitch, and Roll:\t");
    cl.print("<p>Yaw:");
    cl.print(ypr[0] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Pitch:");
    cl.print(ypr[2] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Roll:");
    cl.print(ypr[1] * 180/M_PI); 
    cl.print("\t");
    cl.println("</p>");
  #endif
  #ifdef OUTPUT_READABLE_REALACCEL
    // Display real acceleration, adjusted to remove gravity
    cl.print("Real Accel:\t");
    cl.print("<p>Yaw:");
    cl.print(aaReal.x); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Pitch:");
    cl.print(aaReal.z); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Roll:");
    cl.print(aaReal.y); 
    cl.print("\t");
    cl.println("</p>");
  #endif
  #ifdef OUTPUT_READABLE_WORLDACCEL
    /*Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from quaternion*/
    cl.print("World Accel:\t");
    cl.print("<p>Yaw:");
    cl.print(aaWorld.x); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Pitch:");
    cl.print(aaWorld.z); 
    cl.print("\t");
    cl.println("</p>");
    cl.print("<p>Roll:");
    cl.print(aaWorld.y); 
    cl.print("\t");
    cl.println("</p>");
  #endif
  #ifdef OUTPUT_TEAPOT
    cl.print("<p>teapotpacket:");
    cl.write(teapotPacket, 14); 
    cl.print("\t");
    cl.println("</p>");
  #endif
}
