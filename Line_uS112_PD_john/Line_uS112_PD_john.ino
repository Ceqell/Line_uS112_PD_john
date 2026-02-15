#include "QTRSensors.h"

#define NUM_SENSORS   9     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   -1    // emitter is controlled by digital pin 2

#define PWM1    5
#define INA1    2
#define INB1    3
#define PWM2    6
#define INA2    4
#define INB2    7  

#define SW_PORT 8  

#define STOP      { motorControl(0,0); }

int minValue[9]  = {992,684,628,628,628,628,572,632,992};
int maxValue[9]  = {2500,2500,2500,2500,2500,2500,2500,2500,2500};

int leftSpeed, rightSpeed;
int error, last_error;
int PDValue;
unsigned long LastTime = 0;
unsigned long fline , sline ;
int ans = 0;
QTRSensorsRC qtrrc((unsigned char[]) {
  19,18,17,16,15,14,11,10,9
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int Position = 0;

void setup() {
  initRobot();
  //readTune();  // For Read minValue and MaxValue Only
   setValue();
}

void loop() {
  SW_press();
  delay(1000);
  runTimer(100, 0.037, 0.15, 1500); // 0 to 150 causes wheelie
  //runTimer(150, 0.045, 0.25, 1500); STOP
  //runTimer(80, 0.037, 0.15, 1500); STOP // the deadly turn
  //runTimer(150, 0.045, 0.25, 2500); STOP
  //runTimer(80, 0.037, 0.15, 1500); STOP // the deadly turn
  //runTimer(150, 0.045, 0.25, 5500); STOP
  ChkCross(120, 0.037, 0.15); STOP
  turnLeft();
  runTimer(90, 0.045, 0.25, 250); STOP
  motorControl(90, 90); delay(150); STOP
  turnRight();
  runTimer(90, 0.045, 0.25, 450); STOP
  ChkCross(150, 0.037, 0.15); STOP
  turnRight();
  runTimer(100, 0.045, 0.25, 350); STOP
  //ChkCross(130, 0.037, 0.15); STOP
  // motorControl(90, 90); delay(150); STOP
  turnLeft();
  //runTimer(90, 0.045, 0.25, 250); STOP
  runTimer(100, 0.037, 0.15, 1500); // 0 to 150 causes wheelie
  runTimer(150, 0.045, 0.25, 6100); STOP
  // while(true) drowningDin(); //uncomment this if you want it to run forever
}

void drowningDin() {
  runTimer(100, 0.037, 0.15, 1500); // 0 to 150 causes wheelie
  //runTimer(150, 0.045, 0.25, 1500); STOP
  //runTimer(80, 0.037, 0.15, 1500); STOP // the deadly turn
  //runTimer(150, 0.045, 0.25, 2500); STOP
  //runTimer(80, 0.037, 0.15, 1500); STOP // the deadly turn
  //runTimer(150, 0.045, 0.25, 5500); STOP
  ChkCross(120, 0.037, 0.15); STOP
  turnLeft();
  runTimer(90, 0.045, 0.25, 250); STOP
  motorControl(90, 90); delay(150); STOP
  turnRight();
  runTimer(90, 0.045, 0.25, 450); STOP
  ChkCross(150, 0.037, 0.15); STOP
  turnRight();
  runTimer(100, 0.045, 0.25, 350); STOP
  //ChkCross(130, 0.037, 0.15); STOP
  // motorControl(90, 90); delay(150); STOP
  turnLeft();
  //runTimer(90, 0.045, 0.25, 250); STOP
  runTimer(100, 0.037, 0.15, 1500); // 0 to 150 causes wheelie
  runTimer(150, 0.045, 0.25, 6100); STOP
}

void readTune(void) {
  SW_press();
  Serial.println("Reading");
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
    delay(20);
  }
  Serial.print("int minValue[9]  = {");
  for (int i = 0; i < 9; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    if (i < 8) Serial.print(",");
  }
  Serial.println("};");
  Serial.print("int maxValue[9]  = {");
  for (int i = 0; i < 9; i++) {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      if (i < 8) Serial.print(",");
  }
  Serial.println("};");
  while(1);
}

void setValue(void) {
  qtrrc.calibrate();
  for (int i = 0; i < 9; i++) {
    qtrrc.calibratedMinimumOn[i] = minValue[i];
    qtrrc.calibratedMaximumOn[i] = maxValue[i];
  }
}

void ChkCross(int runSpeed, float Kp, float Kd) {
  qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  while (sensorValues[0] < 600 || sensorValues[8] < 600) stdPD(runSpeed, Kp, Kd);
  qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  while (sensorValues[0] > 600 || sensorValues[8] > 600) stdPD(runSpeed, Kp, Kd);

}

void ChkCrossCounter(int runSpeed, float Kp, float Kd, int counterCross) {
  for (int i = 0 ; i < counterCross; i++) {
    if (counterCross - i != 1) {
      qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
      while (sensorValues[0] < 600 || sensorValues[8] < 600) stdPD(runSpeed, Kp, Kd);
      qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
      while (sensorValues[0] > 600 || sensorValues[8] > 600) stdPD(runSpeed, Kp, Kd);
    } else {
      qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
      while (sensorValues[0] < 600 || sensorValues[8] < 600) stdPD(runSpeed, Kp, Kd);
    }
  }
}

void stdPD(int runSpeed, float Kp, float Kd) {
    Position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 150, 50 );
    if (sensorValues[0] > 600 && sensorValues[8] > 600) motorControl(runSpeed, runSpeed);
    error = Position - 4000;
    PDValue = (Kp * error) + (Kd * (error - last_error));
    last_error = error;
    if (PDValue > runSpeed) PDValue = runSpeed;
    if (PDValue < -runSpeed) PDValue = -runSpeed;
    leftSpeed = runSpeed + PDValue;
    rightSpeed = runSpeed - PDValue;
    if (leftSpeed > 255) leftSpeed =  255;
    if (leftSpeed < -256) leftSpeed = -256;
    if (rightSpeed > 255) rightSpeed = 255;
    if (rightSpeed < -256) rightSpeed = -256;
    motorControl(leftSpeed, rightSpeed);
}

void runTimer(int runSpeed, float Kp, float Kd, int Timer) {
  LastTime = millis();
  while ((millis() - LastTime) <= Timer) {
    Position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 150, 50 );
    if (sensorValues[0] > 600 && sensorValues[8] > 600) motorControl(runSpeed, runSpeed);
    error = Position - 4000;
    PDValue = (Kp * error) + (Kd * (error - last_error));
    last_error = error;
    if (PDValue > runSpeed) PDValue = runSpeed;
    if (PDValue < -runSpeed) PDValue = -runSpeed;
    leftSpeed = runSpeed + PDValue;
    rightSpeed = runSpeed - PDValue;
    if (leftSpeed > 255) leftSpeed =  255;
    if (leftSpeed < -256) leftSpeed = -256;
    if (rightSpeed > 255) rightSpeed = 255;
    if (rightSpeed < -256) rightSpeed = -256;
    motorControl(leftSpeed, rightSpeed);
  }
}


void turnLeft(){
  motorControl(-100,100); delay(120); 
  qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  while(sensorValues[0] < 600 )qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  motorControl(100,-100); delay(30);
  STOP
}

void turnRight(){
  motorControl(100,-100); delay(120); 
  qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  while(sensorValues[7] < 600 )qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 0, 200, 50 );
  motorControl(-100,100); delay(30);
  STOP
}


void Stop(int a) {
  motorControl(0,0);
  delay(a);
}

void motorControl(int pow1,int pow2){
  motor1(pow1);
  motor2(pow2);
}
void motor2(int pow){
  bool dir  = (pow >= 0 ? true : false);
  digitalWrite(INA2,dir);
  digitalWrite(INB2,!dir);
  analogWrite(PWM2,pow);
}
void motor1(int pow){
  bool dir  = (pow >= 0 ? true : false);
  digitalWrite(INA1,dir);
  digitalWrite(INB1,!dir);
  analogWrite(PWM1,pow);  
}

void initRobot(void) {
    pinMode(PWM1, OUTPUT);
    pinMode(INA1, OUTPUT);
    pinMode(INB1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(INA2, OUTPUT);
    pinMode(INB2, OUTPUT);
    pinMode(SW_PORT, INPUT);
    Serial.begin(115200);
}

bool SW(){
    return digitalRead(SW_PORT);
}

void SW_press(){
    while(!SW());
    while(SW());
}
