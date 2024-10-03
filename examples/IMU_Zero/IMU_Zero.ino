/*
  IMU Zero

  Calibrates the MPU6050 to compensate for measurement deviations or errors that may
  occur due to factors such as gyroscope and accelerometer offset.

  When running this example, the code adjusts the MPU6050 sensor's acceleration (X,Y,Z)
  and rotation (gyroscope) axis offset values to ensure that the readings are as accurate as
  possible when the device is at rest.

  To get better results consider:
  1. The MPU6050 module is working fine.
  2. Put the MPU6050 on a flat and horizontal surface, and leave it operating for 
  5-10 minutes so its temperature gets stabilized.
  4. Is in a location where the pull of gravity is 1g.

  During the execution it will generate a dozen outputs, showing that for each of the 6 
  desired offsets, it is:
  - First, try to find two estimates, one too low and one too high.
  - Closing in until the bracket can't be made smaller.

  The line just above the "done" (it will take a few minutes to get there) describes the 
  optimum offsets for the X acceleration, Y acceleration, Z acceleration, X gyro, Y gyro, 
  and Z gyro, respectively.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/

#include "I2Cdev.h"
#include "MPU6050.h"

/* Create the class for MPU6050. Default I2C address is 0x68 */
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

const int usDelay = 3150; // Delay in ms to hold the sampling at 200Hz
const int NFast = 1000; // Number of quick readings for averaging, the higher the better
const int NSlow = 10000; // Number of slow readings for averaging, the higher the better
const int LinesBetweenHeaders = 5;

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;
int i;

void setup() {
  Initialize(); //Initializate and calibrate the sensor
  for (i = iAx; i <= iGz; i++) { 
    Target[i] = 0; // Fix for ZAccel 
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  } 
  Target[iAz] = 16384; // Set the taget for Z axes
  SetAveraging(NFast); // Fast averaging
  PullBracketsOut();
  PullBracketsIn();
  Serial.println("-------------- DONE --------------");
}

void loop() {
  //Write your code here
} 

/*Initializate function*/
void Initialize() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  // Init the module
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("MPU initializated");
  // Check module connection
  Serial.println("Testing device connections...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
  while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  Serial.println("\nPID tuning Each Dot = 100 readings");
  /*
  PID tuning (actually PI) works like this: changing the offset in the MPU6050 gives instant results, 
  allowing us to use the Proportional and Integral parts of the PID to find the ideal offsets. 
  The Integral uses the error from the set point (which is zero) and adds a fraction of this error to 
  the integral value. Each reading reduces the error towards the desired offset. The greater 
  the error, the more we adjust the integral value. 
  
  The Proportional part helps by filtering out noise from the integral calculation. The Derivative part is 
  not used due to noise and the sensor being stationary. With the noise removed, the integral value stabilizes 
  after about 600 readings. At the end of each set of 100 readings, the integral value is used for the actual 
  offsets, and the last proportional reading is ignored because it reacts to any noise.
  */
  Serial.println("\nXAccel\t\tYAccel\t\tZAccel\t\tXGyro\t\tYGyro\t\tZGyro");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("\n600 Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("700 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("800 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("900 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("1000 Total Readings");
  mpu.PrintActiveOffsets();
  Serial.println("\nAny of the above offsets will work nicely \n\nProving the PID with other method:");
}

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("\nAveraging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  Serial.println("Expanding:");
  ForceHeader();

  while (!Done) {
    Done = true;
    SetOffsets(LowOffset); //Set low offsets
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get low values
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } 
      else {
        NextLowOffset[i] = LowOffset[i];
      }
    }
    SetOffsets(HighOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } 
      else {
        NextHighOffset[i] = HighOffset[i];
      }
    } 
    ShowProgress();
    for (int i = 0; i <= 5; i++) {
      LowOffset[i] = NextLowOffset[i]; 
      HighOffset[i] = NextHighOffset[i];
    }
  }
}

void PullBracketsIn() {
  boolean AllBracketsNarrow;
  boolean StillWorking;
  int NewOffset[6];

  Serial.println("\nClosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } 
    else {
      AllBracketsNarrow = true;
    }
    for (int i = 0; i <= 5; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } 
      else { // Binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } 
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Closing in
      if (Smoothed[i] > Target[i]) { // Use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } 
      else { // Use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } 
    }
    ShowProgress();
  } 
} 

void ForceHeader() {
  LinesOut = 99;
}

/*Function to smooth the read values*/
void GetSmoothed() {
  int16_t RawValue[6];
  long Sums[6];
  for (i = 0; i <= 5; i++) {
    Sums[i] = 0;
  }
  
/* Get Sums*/
  for (i = 1; i <= N; i++) { 
    mpu.getMotion6( & RawValue[iAx], & RawValue[iAy], & RawValue[iAz], & RawValue[iGx], & RawValue[iGy], & RawValue[iGz]);
    delayMicroseconds(usDelay);
    for (int j = 0; j <= 5; j++){
      Sums[j] = Sums[j] + RawValue[j];
    }
  } 
  for (i = 0; i <= 5; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} 

/*Function for configure the oba=tained offsets*/
void SetOffsets(int TheOffsets[6]) {
  mpu.setXAccelOffset(TheOffsets[iAx]);
  mpu.setYAccelOffset(TheOffsets[iAy]);
  mpu.setZAccelOffset(TheOffsets[iAz]);
  mpu.setXGyroOffset(TheOffsets[iGx]);
  mpu.setYGyroOffset(TheOffsets[iGy]);
  mpu.setZGyroOffset(TheOffsets[iGz]);
}

/*Print the progress of the reading averages, add formatting for better visualization*/
void ShowProgress() {
 /*Header*/
  if (LinesOut >= LinesBetweenHeaders) { 
    Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  } 
  Serial.print(' ');
  for (i = 0; i <= 5; i++) {
    Serial.print('[');
    Serial.print(LowOffset[i]),
    Serial.print(',');
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(',');
    Serial.print(HighValue[i]);
    if (i == 5) {
      Serial.println("]");
    } 
    else {
      Serial.print("]\t");
    }
  }
  LinesOut++;
} 
