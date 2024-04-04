#include <PID_v1.h>

#define WATER 3
#define SALT 8
#define CONCENTRATION 1

#define RUN 1
#define CALIBRATE 2
#define FLUSH 3
#define CURVELOAD 4
#define STOP 5

#define CALIBRATION_TIME 10000
#define FLUSH_TIME 10000
#define MAX_ROWS 300
#define RAMP_TIME 0

#define FULL_SPEED 255
#define OFF 0
#define WATER_SPEED 190
#define SALT_SPEED 120
#define WATER_MIN 50
#define SALT_MIN 75

#define SETPOINT_THRESHOLD 40
#define WINDOW 5

#define SALTKP 0.09
#define SALTKI 0.18
#define SALTKD 0.04
#define WATERKP 0.0
#define WATERKI 0.0
#define WATERKD 0.0




double Setpoint, Input, Output;

double Kp = SALTKP, Ki = SALTKI, Kd = SALTKD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


int lowSpeed = 0;
int highSpeed = 0;

int prevSig = 0;
int newSig = 0;
double freq = 0.0;
double flow = 0;
double time = 0;

float conc = 0;
float salt_conc = 0;
float water_conc = 0;

unsigned long runStart;
unsigned long flushStart;
unsigned long begin=0;
unsigned long elapsedTime;
int state = 5;
int pidState = 0;
bool flowState = false;
int numRows = 0;
float curve[MAX_ROWS][2] ;
int curveIndex = 0;
int i=0;
const byte numRowsSize = sizeof(int);
const byte dataPointSize = sizeof(double);
const int BUFFER_SIZE = 5; // 4 bytes for the string + 1 for null terminator
char buffer[BUFFER_SIZE]; // Buffer to store the received bytes

void setup() {
  pinMode(WATER, OUTPUT);
  pinMode(SALT, OUTPUT);
  Input = analogRead(CONCENTRATION);
  //numRows = 2;    // TEMPORARY LINE


  Serial.begin(115200);

  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(8);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(SALT_MIN, 255);

  
  state = STOP;      // change this to be in the Stopped State once GUI is fully done
  
  
}

void loop() {


  // for (int i = 0; i < 50; i++) {
  //   conc += analogRead(CONCENTRATION);
  // }
  // conc = conc / 50;

    

  if (state == RUN) {
    
    conc=random(40);
    if (i==0){
      runStart=millis();
    }

    if (millis() - runStart > (unsigned long)curve[curveIndex + 1][0] * 1000){
    curveIndex++;
    

    }
    
    Setpoint = curve[curveIndex][1];
  

    if (curveIndex >= numRows) {
      state = STOP;  // MAYBE HAVE A MESSAGE THAT PRINTS OUT THE PROFILE HAS BEEN COMPLETED
      Serial.println("Finished... Stopping...");
      curveIndex = 0;
    }

    Input = conc;
    myPID.SetSampleTime(8);
    
    myPID.Compute();
    //Output  = map_range(Output, 0, 255, SALT_MIN, 255);
    int waterCommand = linearInterpolation(Setpoint,180,20,70,240);
    
    analogWrite(WATER, waterCommand);
    analogWrite(SALT, (int)Output);

    Serial.print(float((millis() - runStart))/1000.0);
    Serial.print(" ");
    // Serial.print(Setpoint);
    // Serial.print(" ");
    Serial.print(conc);
    Serial.print(" ");
    Serial.println(conc);
    // Serial.print(" ");
    // Serial.print(waterCommand);
    // Serial.print(" ");
    // Serial.print((int)Output);
    // Serial.print(" ");
    // Serial.print(Setpoint*1.1);
    // Serial.print(" ");
    // Serial.println(Setpoint*0.9);
    
 
    


    // Serial.print(0);
    // Serial.print(" ");
    // Serial.print(25);
    // Serial.print(" ");
    
    //delay(5);
    i+=1;
    
  }
  if (state == FLUSH) {
    Serial.println("flush");
    flushStart=millis();

    analogWrite(WATER, FULL_SPEED);
    analogWrite(SALT, FULL_SPEED);
    

    while (millis() - flushStart <= 10000) {
      Serial.print(millis()-flushStart);
      Serial.print("flushing");
      Serial.print(" ");
      Serial.println(analogRead(CONCENTRATION));
      
    }
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
    Serial.println("Done Flush");
    state = STOP;
  }
  if (state == CALIBRATE){
      Serial.println("calibrate");
      int calibrationStart = millis();

      analogWrite(WATER, FULL_SPEED);
      analogWrite(SALT, OFF);

      while (millis() - calibrationStart <= CALIBRATION_TIME) {
      }

      analogWrite(WATER, OFF);
      analogWrite(SALT, OFF);

      delay(1000);
      for (int i = 0; i < 50; i++) {
        water_conc += analogRead(0);
      }
      water_conc = water_conc / 50;

      calibrationStart = millis();
      analogWrite(WATER, OFF);
      analogWrite(SALT, FULL_SPEED);

      while (millis() - calibrationStart <= CALIBRATION_TIME) {
      }

      analogWrite(WATER, OFF);
      analogWrite(SALT, OFF);

      delay(1000);
      for (int i = 0; i < 50; i++) {
        salt_conc += analogRead(0);
      }
      salt_conc = salt_conc / 50;

      state = STOP;
  }
  if (state == CURVELOAD) {
    Serial.println("curveload");
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
    while (!Serial.available()) {
    }
    String receivedString;
    if (Serial.available() > 0) {
      // Serial.readBytes((char*)&numRows, numRowsSize); // Read numRows as an int
      receivedString = Serial.readStringUntil('\n');
      Serial.println("Received numRows: " + receivedString);
      numRows = receivedString.toInt();

      // Define the 2D array to store received data

      // Convert bytes back into 2D list of doubles
      for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < 2; j++) {
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

  if (state == STOP) {
    
    myPID.SetMode(MANUAL);
    analogWrite(WATER, OFF);
    analogWrite(SALT, OFF);
  }



  if (Serial.available() > 0) {
    
    
    // read incoming serial data:

    String inChar = Serial.readStringUntil('\n');
    Serial.println(inChar);
    if (inChar.equals("1")) {
      state = RUN;
      //Serial.println("About to start...");
      myPID.SetMode(AUTOMATIC);
      runStart = millis();
      curveIndex = 0;
      analogWrite(WATER, WATER_SPEED);
    } else if (inChar.equals("2")) {
      Serial.println(CALIBRATE);
      state = CALIBRATE;
    } else if (inChar.equals("3")) {
      state = FLUSH;
      flushStart = millis();
    } else if (inChar.equals("4")) {
      Serial.println(CURVELOAD);
      state = CURVELOAD;
    } else if (inChar.equals("5")) {
      Serial.println("Stopping");
      state = STOP;
    }
     
    else if (inChar.equals("6")) {
      Serial.println(myPID.GetKp(),3);
      Serial.println(myPID.GetKi(),3);
      Serial.println(myPID.GetKd(),3);   
    }

    else if (inChar.equals("7")){
        Serial.println("Changing Gains");
        String receivedString;
        double tempKp, tempKi, tempKd;
        while (!Serial.available()) {
      }
      if (Serial.available() > 0) {
         delay(50);
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
      myPID.SetTunings(Kp,Ki,Kd);
      
      Serial.println("Done Changing Gains");
    }

    else if (inChar.equals("8")){
      Serial.println("Changing Setpoint...");
        String receivedString;
        double tempSet;
        while (!Serial.available()) {
        }
      if (Serial.available() > 0) {
          receivedString = Serial.readString();
          Serial.println(receivedString);
          tempSet = receivedString.toDouble();
      }
      curve[0][1]= tempSet;
      Serial.println(curve[0][1]); 
      Serial.println("Done Changing Setpoint..");

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

int linearInterpolation( float x, float x1, float x0, float y1, float y0){
  float normalized_Value = (y1-y0)/(x1-x0)*(x-x0)+y0;
  return (int)normalized_Value;


}