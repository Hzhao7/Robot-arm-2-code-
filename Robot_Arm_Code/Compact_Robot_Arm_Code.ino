#include <Wire.h>
 
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
  pwm.setPWM(12, 0, 410);
  delay(1000);
  pwm.setPWM(13, 0, 725);
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
    pulse_wide = map(potVal, 1013, 10, MAX_PULSE_WIDTH, MIN_PULSE_WIDTH);
  }
  
  pwm.setPWM(motorOut, 0, pulse_wide);
 
}

void calculateTorques(int mass){
  float L1 = 0.125; // known values from measurements
  float L2 = 0.09;
  float m1 = 0.101;
  float m2 = 0.111;

  int elbowVal, elbowAngle;
  
  elbowVal = analogRead(A1);
  elbowAngle = map(elbowVal, 1013, 10, 0, 2*3.14159);
  
}

 
void loop(){
  moveMotor(potWrist, wrist);
  moveMotor(potElbow, elbow);
  moveMotor(potShoulder, shoulder);
  moveMotor(potBase, base);

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(String("Torque 1: ") + String(motorOneTorque)); // updates LCD
  lcd.setCursor(0, 2);
  lcd.print(String("Torque 2: ") + String(motorTwoTorque));

  int gripperButton = digitalRead(13);
  if (gripperButton == HIGH)
  {
    pwm.setPWM(gripper, 0, 150);                             //Keep Gripper closed when button is not pressed
  }
  else
  {
    pwm.setPWM(gripper, 0, 225);                              //Open Gripper when button is pressed
  }

  int resetButton = digitalRead(12);                          // reset button to go back to start position
  if (resetButton == LOW){
    resetMotorPos();
  }
}
