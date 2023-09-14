/*
 * Written by Tiny Boat Productions, 2022
 * DIY Pick up winder version 2
 * 
 * Referance Documents
 * Potentiometer: https://docs.arduino.cc/learn/electronics/potentiometer-basics
 * DC Motor: https://www.tutorialspoint.com/arduino/arduino_dc_motor.htm
 * Reed Switch: https://create.arduino.cc/projecthub/muchika/reed-switch-with-arduino-81f6d2
 * I2C LCD: https://create.arduino.cc/projecthub/Arnov_Sharma_makes/lcd-i2c-tutorial-664e5a
 * Debounce: https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce
 * H-Bridge: https://hackerstore.nl/PDFs/Tutorial298.pdf
 * 
 */

#include "Wire.h"
#include "LiquidCrystal_I2C.h" // v1.1.2

const int DEBUG_PORT = 9600;
const unsigned long DEBOUNCE_DELAY = 20;
const int LCD_COLUMNS = 16;

//Pin declarations
const int MOTOR_PIN = 9; //motor pin
const int POT_PIN = A1;  //pot switch pin
const int REED_PIN = 3;  //reed switch pin
const int CW_PIN = 5;    //clockwise pin
const int CCW_PIN = 4;
const int IN3_PIN = 12;  
const int IN4_PIN = 11;


LiquidCrystal_I2C lcd(0x27,20,4); //LCD setup

//Inital Values
int potVal;     //reading from the potentiometer
int lastPotVal = 0;
int motorSpeed;
int turnCount = 0;      //revoultion count
bool runState = false;           //run state
bool lastRunState = false;
unsigned long lastDebounceTime = 0;
int turnsSinceUpdate = 0;
int lastUpdateTime = 0;
int currentRPM = 0;
int lastPercent = 0;
int motorPercent = 0;

void handleReedUpdate() {
  int currentTime = millis();

  if (currentTime - lastDebounceTime > DEBOUNCE_DELAY) {
    turnsSinceUpdate++;
    currentRPM = 60 / (currentTime - lastUpdateTime);
    lastUpdateTime = currentTime;
  }
  lastDebounceTime = currentTime;
}

void setup() {
  //set up motor and reed switch pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), handleReedUpdate, FALLING);
  pinMode(CW_PIN, INPUT_PULLUP);
  pinMode(CCW_PIN, INPUT_PULLUP);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  Serial.begin(DEBUG_PORT);

  //set up the lcd
  lcd.init();
  lcd.backlight();    //turn on the backlight
  lcd.setCursor(0,0); //set the cursor in the uper left corner
  lcd.print("Pickup winder");
  lcd.setCursor(0,1); //set the cursor at the start of the second line
  lcd.print("By: Tiny Boat");
  delay(1500);
  
  while (analogRead(POT_PIN) > 5){ //Make sure the motor is at low speed before starting it
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Turn pot CCW");
    delay(500);
  }
  
  while(digitalRead(REED_PIN) == 0){ //Ensure you dont start on the magnet so the count is accurate
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Turn winding wheel 1/4 turn");
    delay(500);
    turnCount = 0;
  }

  while(digitalRead(CW_PIN) == 0 || digitalRead(CCW_PIN) == 0){ //Ensure the switch is in the off position
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Flip switch to");
    lcd.setCursor(0,1);
    lcd.print("off position");
    delay(500);
  }

    lcd.clear();
    lcd.print("Speed:  Count:");
    lcd.setCursor(0, 1);
    lcd.print("0");
}

void loop() {
  // put your main code here, to run repeatedly:
  potVal = analogRead(POT_PIN);
  if(digitalRead(CW_PIN) == 0){
    lastRunState = runState;
    runState = true;
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (digitalRead(CCW_PIN) == 0){
    lastRunState = runState;
    runState = true;
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else {
    lastRunState = runState;
    runState = false;
  }

  //set the motor speed var
  if (!runState){
    motorSpeed = 0;
  } else if ((potVal != lastPotVal || runState != lastRunState) && runState) { //if the motor speed or the run state has ch/anged, and the motor is not off
    motorSpeed = potVal / 4;
    lastPotVal = potVal;    
  }

  //set the motor speed pwm
  analogWrite(MOTOR_PIN, motorSpeed);

  //update the screen
  motorPercent = (motorSpeed*100)/255;
  if (motorPercent != lastPercent){
    //if( motorSpeed >= lastSpeed*0.01)||(motorSpeed <= lastSpeed*0.01){
      lcd.setCursor(0, 1);
      lcd.print("        ");
      lcd.setCursor(0, 1);
      lcd.print( motorPercent);
      lastPercent = motorPercent;
    //}
  }
  
  if (turnsSinceUpdate > 0) {
    if(digitalRead(CCW_PIN) == 0){
      turnCount += turnsSinceUpdate;
    } else {
      turnCount -= turnsSinceUpdate;
    }
    
    turnsSinceUpdate = 0;

    lcd.setCursor(LCD_COLUMNS / 2, 1);
    lcd.print(turnCount);
  }

  Serial.print("Motor speed: ");
  Serial.print(motorSpeed);
  Serial.print(",Count: ");
  Serial.print(turnCount);
  Serial.print(",run state: ");
  Serial.println(runState);
}
