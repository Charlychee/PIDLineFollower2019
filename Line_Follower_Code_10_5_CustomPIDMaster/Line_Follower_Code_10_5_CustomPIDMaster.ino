/*#################################THIS BASELINE IS FOR SIMPLE LOOPS ONLY####################################################*/
/* ************************************************************************************************* */
// * UCSD ECE 5 Lab 4 Code: Line Following Robot with PID * //
/* ************************************************************************************************* */

//   This is code for your PID controlled line following robot.
//
//
//
//
// Code Table of Contents
//  1)  Declare Variables     - declares many variables as global variables so each variable can be accessed from every function
//  2)  Setup (Main)          - runs once at beginning when you press button on arduino or motor drive or when you open serial monitor
//  3)  Loop  (Main)          - loops forever calling on a series of functions
//  4)  Calibration           - makes white = 0 and black = 100 (a few seconds to prep, a few seconds on white, a few seconds to move to black, a few seconds of black)
//  5)  Read Potentiometers   - reads each potentiometer
//  6)  Run Motors            - runs motors
//  7)  Read Photoresistors   - reads each photoresistor
//  8)  Calculate Error       - calculate error from photoresistor readings
//  9)  PID Turn              - takes the error and implements PID control
//  10) Print                 - used for debugging but should comment out when not debugging because it slows down program

// ************************************************************************************************* //
#define _DEBUG_
#define PSCALAR 2 //To scale effect of P ASK
// Declare Variables

#define _WIFI_

// Variables and Libaries for Motor (This part is from Motor_Driver_Code 10_3)
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//Library from https://github.com/rlogiacco/CircularBuffer
#include <CircularBuffer.h> //Libaray to create a FIFO buffer ASK
#include <PrintEx.h>

#ifdef _WIFI_
PrintEx newSerial = Serial1; //Use serial1 to talk to ESP32, otherwise use normal serial port
#else
PrintEx newSerial = Serial;
#endif

CircularBuffer<int, 10> kiBuffer; //Create buffer for error sum

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);  //  Motors can be switched here (1) <--> (2) (if you see motors responding to error in the opposite way they should be)
Adafruit_DCMotor *Motor2 = AFMS.getMotor(3);  //  ******Adjust the same way if you did in 10_3******

int M1Sp = 20, M2Sp = 20;   // this is the nominal speed for the motors when not using potentiometer
int M1SpeedtoMotor, M2SpeedtoMotor;

// Variables for Potentiometer (This part is from Potentiometer_Code 10_1)
const int S_pin = A0; //proportional control
const int P_pin = A1; //proportional control
const int I_pin = A2; //integral control
const int D_pin = A3; //derivative control
int       SpRead = 0; int Sp; //Speed Increase
int       kPRead = 0; //proportional gain
int       kIRead = 0; //integral gain
int       kDRead = 0; //derivative gain

// Variables for Light Sensors (This part is from Calibration_Code 10_4)
int LDR_Pin[7] = {A8, A9, A10, A11, A12, A13, A14}; // store photoresistor pins
int LDR[7]; // store photoresistor readings

// Calibration Variables (This part is from Calibration_Code 10_4)
int   led_Pin = 12;      /// This is a led set up to indicate what part of the calibration you are on.
float Mn[7];             ///     You could also connect a larger/more visible LED to Pin 13 or other digital pin
float Mx[7];
float LDRf[7] = {0., 0., 0., 0., 0., 0., 0.};

// Error Calculation Variables (This part is from Calibration_Code 10_4)
int   MxRead;
int   MxIndex;
float AveRead;
int   CriteriaForMax;
float WeightedAve;
int   ii;
int   im0, im1, im2;

// For Motor/Control
int   Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;

float absSumError = 0; //Adds up absolute value of error for speed adjustment ASK
float kSR = 40; //Speed reduction constant for slowing down while correcting for error.  Especially useful for handling sharp turns ASK
int SR; //Calculated speed reduction ASK
unsigned long runTime, pidRunTime, calcRunTime, telementaryTime, telementaryTimeToPrint, fullRunTime, fullRunTimeToPrint, limitTelementary;

int onLineCounter = 0;
// ************************************************************************************************* //
// setup - runs once
// Look at the lab to know what functions to call in which order in setup and loop
void setup()
{

  Serial.begin(115200);        // For serial communication set up

#ifdef _WIFI_                //Only start if using WIFI telementary
  Serial1.begin(115200);
#endif

  AFMS.begin();              // For motor setup
  pinMode(led_Pin, OUTPUT);  // Note that all analog pins used are INPUTs by default so don't need pinMode

  Calibrate();
  ReadPotentiometers();
  RunMotors();
  limitTelementary = millis();

}  // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop()
{
  runTime = micros();
  fullRunTime = micros();
  ReadPotentiometers();
  ReadPhotoResistors();
  CalcError();
  if (abs(error) < 2.5)
  {
    onLineCounter++;
    if (onLineCounter > 40)
    {
      SpRead = Sp = 150;
    } else
    {
      SpRead = Sp = 130;
    }
  } else
  {
    SpRead = Sp = 130;
    onLineCounter = 0;
  }
  PID_Turn();
  RunMotors();
  runTime = micros() - runTime;
  telementaryTime = micros();
  Telementary();
  telementaryTimeToPrint = micros() - telementaryTime;
  fullRunTimeToPrint = micros() - fullRunTime;

#ifdef _DEBUG_
  //Print();
#endif

}  // end loop()

// ************************************************************************************************* //
// function to calibrate
void Calibrate()
{
  // wait to make sure in position
  for (int calii = 0; calii < 4; calii++)
  {
    digitalWrite(led_Pin, HIGH);   // turn the LED on
    delay(100);                    // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);    // turn the LED off
    delay(900);                    // wait for 0.9 seconds
  }

  // Calibration
  // White Calibration
#ifdef _DEBUG_
  //Serial.println("Go to white area");
  //delay(1500);
#endif
  int numMeas = 10;  // number of samples to read for calibration
  for (int calii = 0; calii < numMeas; calii++)
  {
    digitalWrite(led_Pin, HIGH);   // turn the LED on
    delay(100);                    // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);    // turn the LED off
    delay(100);                    // wait for 0.1 seconds

    for ( int ci = 0; ci < 7; ci++ )
    {
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]);
      delay(2);
    }
  }

  for ( int cm = 0; cm < 7; cm++ )
  {
    Mn[cm] = round(LDRf[cm] / (float)numMeas); // take average
    LDRf[cm] = 0.;
  }

  // Time to move from White to Black Surface
#ifdef _DEBUG_
  //Serial.println("Go to black area");
  //delay(2500);
#endif

  for (int calii = 0; calii < 10; calii++)
  {
    digitalWrite(led_Pin, HIGH);
    delay(100);
    digitalWrite(led_Pin, LOW);
    delay(900);
  }

  // Black Calibration
  for (int calii = 0; calii < numMeas; calii++)
  {
    digitalWrite(led_Pin, HIGH);
    delay(100);
    digitalWrite(led_Pin, LOW);
    delay(100);

    for ( int ci = 0; ci < 7; ci++ )
    {
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]);
      delay(2);
    }
  }
  for ( int cm = 0; cm < 7; cm++ )
  {
    Mx[cm] = round(LDRf[cm] / (float)numMeas); // take average
    LDRf[cm] = 0.;
  }

#ifdef _DEBUG_
  //Serial.println("Go to track");
  delay(5000);
#endif
}  // end Calibrate()

// ************************************************************************************************* //
// function to read and map values from potentiometers
void ReadPotentiometers()
{
  //SpRead = map(analogRead(S_pin), 0, 1023, 0, 150); Sp = SpRead;
  //SpRead = Sp = 150;
  kPRead = map(analogRead(P_pin), 0, 1023, 0, 40);
  kIRead = map(analogRead(I_pin), 0, 1023, 0, 5);
  kDRead = map(analogRead(D_pin), 0, 1023, 0, 10);
}    // end ReadPotentiometers()

// ************************************************************************************************* //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors()
{
  // ******Remember you can insert your custom control in here******
  // ******To control motor behaviour from error, etc******
  M1SpeedtoMotor = min(M1Sp + Sp + M1P, 255); // limits speed to 255
  M2SpeedtoMotor = min(M2Sp + Sp + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  //Custom Motor Speed Implementation ASK
  /*
    if (error < 0)
    {
    M1SpeedtoMotor = M1Sp + Sp - SR;
    M2SpeedtoMotor = M2Sp + Sp + M2P - SR;
    } else
    {
    M1SpeedtoMotor = M1Sp + Sp - M1P - SR;
    M2SpeedtoMotor = M2Sp + Sp - SR;
    }
  */

  Motor1->setSpeed(abs(M1SpeedtoMotor));
  Motor2->setSpeed(abs(M2SpeedtoMotor));

  if (M1SpeedtoMotor > 0)
  {
    Motor1->run(FORWARD);
  }
  else
  {
    Motor1->run(BACKWARD);
  }

  if (M2SpeedtoMotor > 0)
  {
    Motor2->run(FORWARD);
  }
  else
  {
    Motor2->run(BACKWARD);
  }

}  // end RunMotors()

// ************************************************************************************************* //
// function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MxIndex)
void ReadPhotoResistors()
{
  for (int Li = 0; Li < 7; Li++)
  {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    //delay(2);
  }
}   // end ReadPhotoResistors()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
void CalcError()
{
  calcRunTime = micros();
  int sum = 0; //Added to do more efficient average calculation ASK
  MxRead = -99;
  AveRead = 0.0;
  // Step 1)  Iterate through photoresistor mapped values, find darkest/max (MxRead), it's index (im1) and weighted index (MxIndex)
  //          Weighted Index: from left to right: 3 - 2 - 1 - 0 (center) - 1 - 2 - 3
  for (int ii = 0; ii < 7; ii++)
  {
    if (MxRead < LDR[ii])
    {
      MxRead = LDR[ii];
      MxIndex = -1 * (ii - 3);
      im1 = ii; //Used to be cast as a float, not sure why.  Changed to not cast as float ASK
    }
    sum += LDR[ii];
    //AveRead = AveRead + (float)LDR[ii] / 7.; //Rather than do floating point math 7 times per cycle, do it afterwards ASK
  }
  AveRead = sum / 7.0;

  // Step 2)  Calculate error from weighted average, based off the readings around the maximum value
  CriteriaForMax = 2;  // max should be at least twice as big as the other values
  if (MxRead > CriteriaForMax * AveRead)  // only when the max is big enough
  {
    if (im1 != 0 && im1 != 6)  // max not on either ends
    {
      im0 = im1 - 1;  // index for left
      im2 = im1 + 1;  // index for right
      if (LDR[im0] + LDR[im1] + LDR[im2] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1] * im1 + LDR[im2] * im2)) / ((float)(LDR[im0] + LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 0)  // max on left end
    {
      im2 = im1 + 1;
      if (LDR[im1] + LDR[im2] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(LDR[im1] * im1 + LDR[im2] * im2)) / ((float)(LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3) * 1.2; //Increase p for when far off line ASK
    }
    else if (im1 == 6)  // max on right end
    {
      im0 = im1 - 1;
      if (LDR[im0] + LDR[im1] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1] * im1)) / ((float)(LDR[im0] + LDR[im1]));
      error = -1 * (WeightedAve - 3) * 1.2; //Increase p for when far off line ASK
    }
  }
  calcRunTime = micros() - calcRunTime;
}  // end CalcError()


// ************************************************************************************************* //
// function to make a turn ( a basic P controller)
void PID_Turn()
{
  pidRunTime = micros();
  float poppedKiVal = 0;
  // *Read values are between 0 and 100, scale to become PID Constants
  kP = (float)kPRead / 1.;    // each of these scaling factors can change depending on how influential you want them to be
  kI = (float)kIRead / 1000.; // the potentiometers will also scale them
  kD = (float)kDRead / 100.;
  // error holds values from -3 to 3

  //kP *= PSCALAR; //Added to scale effect of kP ASK

  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!

  //Old method of integral error ASK
  //sumerror = sumerror + error;

  //We want to only sum up the last 10 error points to account for line curvature changes rather than sum every error in the past ASK
  sumerror += error;
  absSumError += abs(error);
  if (kiBuffer.size() > 10) //Check if there are too many items on buffer (At max should only reach 10 elements)
  {
    poppedKiVal = kiBuffer.pop(); //Pull item off buffer
    sumerror -= poppedKiVal;  //subtract from sumError and absSumError to keep that limited to only the last 10 errors
    absSumError -= abs(poppedKiVal);
  }
  kiBuffer.push(error); //Add error to our FIFO buffer.  We do this after the check to prevent memory overwrite

  SR = kSR * absSumError;

  if (sumerror > 5) {
    sumerror = 5; // prevents integrator wind-up
  }
  else if (sumerror < -5) {
    sumerror = -5;
  }

  if (absSumError > 10) {
    absSumError = 10;
  }

  lasterror = error;

  // Set the motor speed
  M1P = Turn;
  M2P = -Turn;
  pidRunTime = micros() - pidRunTime;
}  // end PID_Turn()


// ************************************************************************************************* //
// function to print values of interest
void Print()
{
  Serial.print(SpRead); Serial.print(" ");  // Initial Speed addition from potentiometer
  Serial.print(kP, 4); Serial.print(" ");      // PID values from potentiometers after scaling
  Serial.print(kI, 4); Serial.print(" ");
  Serial.print(kD, 4); Serial.print("    ");

  Serial.print(LDR[0]); Serial.print(" ");  // Each photo resistor value is shown
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print("    ");

  Serial.print(MxRead); Serial.print(" ");    // the maximum value from the photo resistors is shown again
  Serial.print(MxIndex); Serial.print("    "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.print(error); Serial.print("    ");  // this will show the calculated error (-3 through 3)

  Serial.print(M1SpeedtoMotor); Serial.print(" "); // This prints the arduino output to each motor so you can see what the values (0-255)
  Serial.println(M2SpeedtoMotor);                  //  that are sent to the motors would be without actually needing to power/run the motors

  // delay(100); //just here to slow down the output for easier reading if wanted
  // ensure delay is commented when actually running your robot or this will slow down sampling too much
}  // end Print()

void Telementary()
{
  if (millis() - limitTelementary > 75) //We have to rate limit to prevent overloading the serial buffer which causes data corruption and significant slowdown at runtime
  {
    newSerial.printf("DATA,%d,%1.4f,%1.4f,%1.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%1.4f,%d,%d,%1.4f,%1.4f,%lu,%lu,%lu,%lu,%lu,%lu,%d\n", SpRead, kP, kI, kD, LDR[0], LDR[1], LDR[2], LDR[3], LDR[4], LDR[5], LDR[6], MxRead, MxIndex, error, M1SpeedtoMotor, M2SpeedtoMotor, absSumError, sumerror, runTime, pidRunTime, calcRunTime, telementaryTimeToPrint, fullRunTimeToPrint, micros(), 0); //the constant 0 is to make the gui happy ASK
    limitTelementary = millis();
  }
}
