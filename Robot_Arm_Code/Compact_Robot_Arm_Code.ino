#include <Wire.h>
#define PI 3.1415926535897932384626433832795
 
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>

#define MIN_PULSE_WIDTH       150
#define MAX_PULSE_WIDTH       725
#define FREQUENCY             50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
LiquidCrystal lcd(6, 7, 2, 3, 4, 5);

int potWrist = A0;
int potElbow = A1;                        //Assign Potentiometers to pins on Arduino Uno
int potShoulder = A2;
int potBase = A3;

int gripper = 11;
int wrist = 12;
int elbow = 13;                           //Assign Motors to pins on Servo Driver Board
int shoulder = 14;
int base = 15;

double motorOneTorque = 0;                   // torque in Nm
double motorTwoTorque = 0;

void resetMotorPos(){
  pwm.setPWM(12, 0, 600);
  delay(1000);
  pwm.setPWM(13, 0, 600);
  delay(1000);
  pwm.setPWM(14, 0, 150);
  delay(1000);
  pwm.setPWM(15, 0, 150);
  delay(1000);
}

void setup() { 
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(String("Torque 1: ") + String(motorOneTorque)); // initialize the LCD
  lcd.setCursor(0, 2);
  lcd.print(String("Torque 2: ") + String(motorTwoTorque));

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(11, 0, 150);                  //Set Gripper to 90 degrees (Close Gripper)
  
  pinMode(13,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  resetMotorPos();
  Serial.begin(9600);
}
 
 
void moveMotor(int controlIn, int motorOut) {
  int pulse_wide, pulse_width, potVal;
  
  potVal = analogRead(controlIn);                                                   //Read value of Potentiometer
  
  pulse_wide = map(potVal, 1013, 10, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); //accounts for if it somehow gets tied to high or low so the arm will not swing
  if (controlIn == A1){
    pulse_wide = map(potVal, 1013, 10, MAX_PULSE_WIDTH, MIN_PULSE_WIDTH); //the elbow direction is reversed as there was an issue in the build process thaty could not be altered
  }
  
  pwm.setPWM(motorOut, 0, pulse_wide);
 
}

void calculateTorques(float mass){ // function to calcuate the torque values
  const float L1 = 0.125; // known values from measurements
  const float L2 = 0.09;
  const float m1 = 0.055;
  const float m2 = 0.057;
  const float motor1 = 0.055;
  const float g = 9.80665;
  float m3 = 0.04 + mass; 

  float elbowVal, theata1, shoulderVal, theata2;
  
  // maps the potentiometer values to angles theata1 and theata 2
  elbowVal = analogRead(A1);
  theata2 = map(elbowVal, 740, 10, 0, 180);
  shoulderVal = analogRead(A2);
  theata1 = map(shoulderVal, 1013, 300, 0, 180); //tried with radians but it would only interva whole numbers making it less accurate
  /*Serial.println(theata1);
  Serial.println(elbowVal);
  Serial.println(theata2);
  Serial.println(shoulderVal);
  delay(1000);*/ // Calibration code
  if (elbowVal > 740){ // gets rid of values outside of the domain
    theata2 = 0;
  }
  if (elbowVal < 10){
    theata2 = PI;
  }
  if (shoulderVal < 300){
    theata1 = 180;
  }
  if (shoulderVal > 1013){
    theata1 = 0;
  }
  theata1 = theata1*PI/180;
  theata2 = theata2*PI/180;
  motorOneTorque = L2*sin(PI/2 - theata2 + theata1)*(m2*g/2 + m3*g);
  
  // finds what equation to use for motorTwoTorque
  float positionCase = 0;
  float L3 = sqrt(L1*L1 + (L2/2)*(L2/2) - L1*L2*cos(theata2));
  float L4 = sqrt(L1*L1 + L2*L2 - 2*L1*L2*cos(theata2));
  float theata4 = asin(L1/L3*sin(theata2));
  float theata5 = asin(L1/L4*sin(theata2));

  /* if (theata2 > theata1 && theata1 <= 1.71 && theata2 <= 1.71){
    positionCase = L3*m2*g*sin(theata4 - PI/2 + theata2 - theata1) - L4*m3*g*sin(PI/2 - theata5 + theata2 - theata1);
  }
  if (theata2 > theata1 && theata1 > 1.71 && theata2 > 1.71){
    positionCase = L3*m2*g*sin(PI/2 - theata4 - theata2 + theata1) + L4*m3*g*sin(PI/2 - theata5 - theata2 + theata1);
  }
  if (theata2 < theata1 && theata1 > 1.71 && theata2 <= 1.71){
    positionCase = L3*m2*g*sin(PI/2 - theata4 + theata1 - theata2) + L4*m3*g*sin(PI/2 - theata5 + theata1 - theata2);
  }
  else{
    positionCase = 0;
  }
  */ //code for trying to account for each case
  if (theata1 > 1.71){
    positionCase = L3*m2*g*sin(PI/2 - theata4 + theata1 - theata2) + L4*m3*g*sin(PI/2 - theata5 + theata1 - theata2);
  }
  else{
    positionCase = 0;
  }

  motorTwoTorque = L1*sin(PI/2 - theata1)*(m1*g/2 + motor1*g) + positionCase;

  if (elbowVal > 740){ // Accounts for when the elbow is resting on the shoulder and the motor is not moving the elbow
    motorOneTorque = 0;
  }
  if (shoulderVal > 1013){ // Accounts for when the shoulder is at rest
    motorTwoTorque = 0;
  }
}
 
void loop(){
  moveMotor(potWrist, wrist);
  moveMotor(potElbow, elbow);
  moveMotor(potShoulder, shoulder);
  moveMotor(potBase, base);

  lcd.clear();
  calculateTorques(1); // calculates the new torques
  lcd.setCursor(0, 0);
  lcd.print(String("Torque 1: ") + String(motorOneTorque)); // updates LCD
  lcd.setCursor(0, 1);
  lcd.print(String("Torque 2: ") + String(motorTwoTorque));

  int gripperButton = digitalRead(13);
  if (gripperButton == HIGH){
    pwm.setPWM(gripper, 0, 150);                             //Keep Gripper closed when button is not pressed
  }
  else{
    pwm.setPWM(gripper, 0, 225);                              //Open Gripper when button is pressed
  }

  int resetButton = digitalRead(12);                          // reset button to go back to start position
  if (resetButton == LOW){
    resetMotorPos();
  }
}
