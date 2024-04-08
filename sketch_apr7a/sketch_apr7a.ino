#include <PID_v1.h>

#define WATER 10
#define SALT 7

#define FULL_SPEED 255
#define OFF 55
#define PUMP_MIN 55

#define WINDOW_SIZE 100

#define CONCENTRATION_INLET 0
#define CONCENTRATION_OUTLET 0

float conc = 0;
double Setpoint, Input, Output;
float saltConc=200;
float waterConc=15;
double Kp = 0, Ki = 0, Kd = 0;

unsigned long currentTime;
unsigned long startTime;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(WATER, OUTPUT);
  pinMode(SALT, OUTPUT);

  Input = analogRead(CONCENTRATION_OUTLET);
  Serial.begin(115200);

  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(8);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(PUMP_MIN, 255);
}

void loop() {
  for (int i = 0; i < WINDOW_SIZE; i++) {
    conc += analogRead(CONCENTRATION_OUTLET);
  }
  conc = conc / WINDOW_SIZE;
  Input = conc;
  Setpoint=waterConc+10;
  if(currentTime-startTime>20000)
  myPID.setTunings(Kp,Ki,Kd);
  myPID.Compute();
  analogWrite(SALT, (int)Output);


}
