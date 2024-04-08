#include <PID_v1.h>

#define WATER 10
#define SALT 7
#define CONCENTRATION_INLET 0
#define CONCENTRATION_OUTLET 0

#define RUN 1
#define CALIBRATE 2
#define FLUSH 3
#define CURVELOAD 4
#define STOP 5
#define SALT_MEASURE 9

#define CALIBRATION_TIME 10000
#define FLUSH_TIME 10000
#define MAX_ROWS 300
#define RAMP_TIME 20000

#define FULL_SPEED 255
#define OFF 0
#define WATER_SPEED 190
#define SALT_SPEED 120
#define WATER_MIN 50
#define SALT_MIN 80

#define SETPOINT_THRESHOLD 40
#define WINDOW_SIZE 100

#define SALTKP 1.0
#define SALTKI 0.0
#define SALTKD 0.00
#define WATERKP 0.0
#define WATERKI 0.0
#define WATERKD 0.0

double Setpoint, Input, Output;

double Kp = SALTKP, Ki = SALTKI, Kd = SALTKD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


float conc = 0;
float saltConc = 0;
float waterConc = 0;

double initialActivity = 0.0;
double upperBoundTracer;
double lowerBoundTracer;

int state = 5;
int numRows = 0;
double curve[MAX_ROWS][2];// = { { 0.0, 140.0 }, { 60.0, 0.0 } };
int curveIndex = 0;

unsigned long runStart;
unsigned long flushStart;
unsigned long calibrationStart;

unsigned long currentTest;
unsigned long currentFlush;
unsigned long currentCal;

double testTime;

String inChar;
String receivedString;

int waterCommand;

double kidneyGains[2][3] = { { 0.4, 0.5, 0.01 }, { 0.2, 0.15, 0.01 } };
double sphereGains[2][3] = { { 0.05, 0.18, 0.01 }, { 0.05, 0.05, 0.01 } };

void setup() {
  pinMode(WATER, OUTPUT);
  pinMode(SALT, OUTPUT);

  Input = analogRead(CONCENTRATION_OUTLET);
  numRows = 2;  // TEMPORARY LINE


  Serial.begin(115200);

  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(8);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(SALT_MIN, 255);


  state = STOP;  // change this to be in the Stopped State once GUI is fully done
}

void loop() {


  for (int i = 0; i < WINDOW_SIZE; i++) {
    conc += analogRead(CONCENTRATION_OUTLET);
  }
  conc = conc / WINDOW_SIZE;
  conc=linearInterpolation(conc,saltConc,waterConc,initialActivity,0);

  if (state == RUN) {

    // if(currentTime-runStart<=RAMP_TIME){
    //   Setpoint = curve[0][1];
    //   currentTime = millis();
    // }
    currentTest=millis();
  
    if (currentTest - runStart >= (int)curve[curveIndex + 1][0] * 1000) {
      curveIndex++;
    }
    Setpoint = curve[curveIndex][1];

    //  unsigned long setpointTime = millis();
    //  Setpoint = 140.0 * exp(-0.005*(double)(setpointTime-currentTime)/1000.0);


    if (curveIndex >= numRows) {
      state = STOP;  // MAYBE HAVE A MESSAGE THAT PRINTS OUT THE PROFILE HAS BEEN COMPLETED
      Serial.println("Finished... Stopping...");
      curveIndex = 0;
    }

    Input = conc;
    //myPID.SetSampleTime(8);
    waterCommand = (int)linearInterpolation(Setpoint, 40, 140, 250, 80);
    analogWrite(WATER, waterCommand);

    Kp = linearInterpolation(Setpoint, 140, 40, kidneyGains[0][0], kidneyGains[1][0]);
    Ki = linearInterpolation(Setpoint, 140, 40, kidneyGains[0][1], kidneyGains[1][1]);
    Kd = linearInterpolation(Setpoint, 140, 40, kidneyGains[0][2], kidneyGains[1][2]);
    // Kp = linearInterpolation(Setpoint, 140, 40, sphereGains[0][0], sphereGains[1][0]);
    // Ki = linearInterpolation(Setpoint, 140, 40, sphereGains[0][1], sphereGains[1][1]);
    // Kd = linearInterpolation(Setpoint, 140, 40, sphereGains[0][2], sphereGains[1][2]);
    // //Serial.print(0);
    myPID.SetTunings(Kp, Ki, Kd);

    myPID.Compute();
    analogWrite(SALT, (int)Output);

    testTime = (double(currentTest)-double(runStart))/1000.0;

    Serial.print(testTime);
    Serial.print(" ");
    Serial.println(conc);
    // Serial.print(" ");
    // Serial.print(Setpoint);
    // Serial.print(" ");
    // Serial.print(0);
    // Serial.print(" ");
    // Serial.print(waterCommand);
    // Serial.print(" ");
    // Serial.print((int)Output);
    // Serial.print(" ");
    // Serial.print(Ki, 3);
    // Serial.print(" ");
    // Serial.print(Setpoint * 1.1);
    // Serial.print(" ");
    // Serial.println(Setpoint * 0.9);
  }

  else if (state == FLUSH) {
    Serial.println("flush");


    analogWrite(WATER, FULL_SPEED);
    analogWrite(SALT, FULL_SPEED);

    while (currentFlush - flushStart <= 10000) {
      Serial.print("flushing");
      Serial.print(" ");
      Serial.println(analogRead(CONCENTRATION_OUTLET));
      currentFlush = millis();
    }
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
    Serial.println("Done Flush");
    state = STOP;
  }
  else if (state == CALIBRATE) {
    Serial.println("calibrate");

    analogWrite(WATER, FULL_SPEED);
    analogWrite(SALT, OFF);
   
   
    delay(CALIBRATION_TIME);

    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);

    delay(1000);

    for (int i = 0; i < 50; i++) {
      waterConc += analogRead(CONCENTRATION_OUTLET);
    }
    waterConc = waterConc / 50;

    
    analogWrite(WATER, OFF);
    analogWrite(SALT, FULL_SPEED);
    
    delay(CALIBRATION_TIME);
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);

    delay(1000);

    for (int i = 0; i < 50; i++) {
      saltConc += analogRead(CONCENTRATION_OUTLET);
    }
    saltConc = saltConc / 50;

    Serial.println(saltConc);
    Serial.println(waterConc);

    state = STOP;
  }
  if (state == CURVELOAD) {
    Serial.println("curveload");
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
    while (Serial.available()==0) {
    }
    if (Serial.available() > 0) {
      // Serial.readBytes((char*)&numRows, numRowsSize); // Read numRows as an int
      receivedString = Serial.readStringUntil('\n');
      Serial.println("Received numRows: " + receivedString);
      numRows = receivedString.toInt();

      // Define the 2D array to store received data
      // Convert bytes back into 2D list of doubles
      for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < 2; j++) {
          while (!Serial.available()) {
          }
          receivedString = Serial.readStringUntil('\n');
          //Serial.println(receivedString);
          //Serial.println(receivedString);
          curve[i][j] = receivedString.toFloat();
          //Serial.println(curve[i][j]);
        }
      }
      Serial.println("Finished Loading Curve");
      for (int i = 0; i < numRows; i++) {
        Serial.print("Row ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(curve[i][0]);
        Serial.print(", ");
        Serial.println(curve[i][1]);
      }
    }

    state = STOP;
  }

  else if (state == STOP) {

    myPID.SetMode(MANUAL);
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
  }
  else if (state == SALT_MEASURE) {
    // analogWrite(WATER, 120);
    // analogWrite(SALT, 120);
    Serial.println(conc);
  }



  if (Serial.available() > 0) {


    // read incoming serial data:

    inChar = Serial.readStringUntil('\n');
    //Serial.println(inChar);

    if (inChar.equals("1")) {
      state = RUN;
      //Serial.println("About to start...");
      myPID.SetMode(AUTOMATIC);
      runStart = millis();
      currentTest = millis();
      curveIndex = 0;
      while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        
        initialActivity = receivedString.toDouble();
        lowerBoundTracer = linearInterpolation(40.0,saltConc,waterConc,initialActivity, 0);
      }
       
    } else if (inChar.equals("2")) {
      Serial.println(CALIBRATE);
      calibrationStart=millis();
      currentCal=millis();
      state = CALIBRATE;
    } else if (inChar.equals("3")) {
      flushStart=millis();
      currentFlush=millis();
      state = FLUSH;

    } else if (inChar.equals("4")) {
      // Serial.println(CURVELOAD);
      state = CURVELOAD;
    } else if (inChar.equals("5")) {
      Serial.println("Stopping");
      state = STOP;
    }

    else if (inChar.equals("6")) {
      Serial.println(myPID.GetKp(), 3);
      Serial.println(myPID.GetKi(), 3);
      Serial.println(myPID.GetKd(), 3);
    }

    else if (inChar.equals("7")) {
      Serial.println("Changing Gains");
      double tempKp, tempKi, tempKd;
      while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        Serial.println(receivedString);
        tempKp = receivedString.toDouble();
      }
      while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        Serial.println(receivedString);
        tempKi = receivedString.toDouble();
      }
      while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        Serial.println(receivedString);
        tempKd = receivedString.toDouble();
      }
      Kp = tempKp;
      Ki = tempKi;
      Kd = tempKd;
      myPID.SetTunings(Kp, Ki, Kd);

      Serial.println("Done Changing Gains");
    }

    else if (inChar.equals("8")) {
      Serial.println("Changing Setpoint...");
      receivedString;
      double tempSet;
      while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        Serial.println(receivedString);
        tempSet = receivedString.toDouble();
      }
      curve[0][1] = tempSet;
      Serial.println(curve[0][1]);
      Serial.println("Done Changing Setpoint..");
    } 
    
    else if (inChar.equals("9")) {
      Serial.println("Measuring Salt...");
      delay(10);
      state = SALT_MEASURE;
    }
  }
}

int map_range(float value, float old_min, float old_max, float new_min, float new_max) {
  // Scale the value from the old range to a value in the range [0, 1]
  float normalized_value = (value - old_min) / (old_max - old_min);
  // Map this normalized value to the new range
  int new_value = new_min + normalized_value * (new_max - new_min);
  return new_value;
}

double linearInterpolation(float x, float x1, float x0, float y1, float y0) {
  float normalized_Value = (y1 - y0) / (x1 - x0) * (x - x0) + y0;
  return (double)normalized_Value;
}