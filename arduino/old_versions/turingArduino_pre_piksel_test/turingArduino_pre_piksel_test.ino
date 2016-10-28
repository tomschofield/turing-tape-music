/*
 * 
 * TURING TAPE MUSIC
 * MIT License

Copyright (c) 2016 Tom Schofield

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <Stepper.h>
#include <Servo.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on the motor shield
Stepper myStepper(stepsPerRevolution, 12, 13);


#include "LedControl.h"


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;


// give the motor control pins names:
const int pwmA = 3;
const int pwmB = 11;
const int brakeA = 9;
const int brakeB = 8;
const int dirA = 12;
const int dirB = 13;
//64*25
//int dist = 1760;
int dist = 1728;

int switchPin = 2;
int rowDist = dist / 32;
//
//const int ind1 = A0;
//const int ind2 = 47;
//const int ind3 = 48;
//const int ind4 = 49;
//const int ind5 = 50;
//const int ind6 = 51;
//const int ind7 = 52;
//const int ind8 = 53;

//reading pins
//const int rp1 = A8;
//const int rp2 = A9;
//const int rp3 = A10;
//const int rp4 = A11;
//const int rp5 = A12;
//const int rp6 = A13;
//const int rp7 = A14;
const int rp7 = A8;
const int rp6 = A9;
const int rp5 = A10;
const int rp4 = A11;
const int rp3 = A12;
const int rp2 = A13;
const int rp1 = A14;

//for serial comms
char lf = '\n';
const int charLength = 20;
char serialdata[charLength];


  
int intensity = 15;

//holds the head state
int indicators[8];

unsigned long delaytime = 100;

int counter = 30;
bool b[8];

int x = 0;
bool off [8] = {false, false, false, false, false, false, false, false};
int pos = 0;
int stepDirection = -1;

int column;
////servo stuff
unsigned int posi[] = {0, 60, 0};
unsigned int posi1[] = {60, 0, 60};

int degreesOfSweep = 180;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo myservo1;

// pin 22 is connected to the DataIn 
// pin 24 is connected to the CLK 
// pin 26 is connected to LOAD 

LedControl lc = LedControl(22, 24, 26, 4);

//LedControl lc = LedControl(0, 5, 10, 4);

int mode = 0;
int numModes = 4;
//mode 0 = read head
//mode 1 = move mode
//mode 2 = write to tape mode
//mode 3 = write state mode


void setup() {
  //Serial.begin(9600);
  // set the PWM and brake pins so that the direction pins  // can be used to control the motor:
  setupStepper();

  Serial.begin(9600);

  for (int i = 0; i < charLength; i++) {
    serialdata[i] = '!';
  }
  Serial.println(serialdata);

  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

//  for (int i = 0; i < 8; i++) {
//    indicators[i] = i + 46;
//    pinMode(indicators[i], OUTPUT);
//  }
  for (int i = 7; i >= 0; i--) {
    indicators[i] = i + 46;
    pinMode(indicators[i], OUTPUT);
  }
  //  indicators[0] = 10  ;
  //  pinMode(indicators[0] , OUTPUT);
  setupMatrix();
  setRandomMatrix();
  setupServos(4, 6);
  delay(500);
  liftHead();
  delay(500);
  setZeroPoint();
  //
    delay(500);
  //
  


}


void loop() {
 //getHeadState(600);

 //delay(10);
////
  if (stringComplete) {
    for (int i = 0; i < charLength; i++) {
      serialdata[i] = '!';
    }
    //    Serial.readBytesUntil(lf, serialdata, charLength);
    String dataname = "";
    String value = "" ;

    boolean addToValue = false;

    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ':' && addToValue == false) {
        addToValue = true;
        i++;
      }
      if (inputString.charAt(i)  != '!') {
        if (addToValue) {

          value += inputString.charAt(i) ;
        }
        else {
          dataname += inputString.charAt(i) ;
        }
      }
    }

    //    Serial.println("received name: " + dataname);
    //    Serial.println("received value: " + value);


    ///read
    if (dataname == "readTape") {

      dropHead();
      delay(500);
      getHeadState(500);
      delay(500);
      liftHead();
    }
    //write to tape
    else if (dataname == "writeTape") {



      byte newSymbol = value.toInt();
      //Serial.println(newSymbol);

      writeToTapeRows(31 + column, newSymbol, 2);

    }
    else if (dataname == "writeTapeAt") {

      String column = "";
      String symbol = "";
      boolean isColumn = true;
      //Serial.println("value : "+value);
      for (int i = 0; i < value.length(); i++) {

        if (value.charAt(i) == ':') {
          isColumn = false;
          i++;
        }

        if (isColumn) {
          column += value.charAt(i);
        }
        else {
          symbol += value.charAt(i);
        }
      }
      //      Serial.println("column : "+column);
      //      Serial.println("symbol : "+symbol);
      //
      byte newSymbol = symbol.toInt();
      int newColumn = 31 - column.toInt();
      //Serial.println(newSymbol);

      writeToTapeRows(newColumn, newSymbol, 2);

    }
    //move head
    else if (dataname == "moveHead") {

      //inByte -= 128;
      int newHeadPos = value.toInt();
      blinkIndicator(31-newHeadPos);
      //  Serial.println(newHeadPos);
      moveHeadTo(-1 * newHeadPos) ;
      Serial.write((253));

    }
    //set head state
    else if (dataname == "setHeadState" ) {
      bool lights [8];

      for (int i = 0; i < 8; i++) {
        lights[i] = true;
      }
      byte newHeadState = value.toInt();
      // Serial.println(newHeadState);
      FromByte(newHeadState, lights);
      setHeadLights(lights);

    }
    inputString = "";
    stringComplete = false;

  }


}

void blinkIndicator(int col) {
  int address;
  int whichCol;
  int blinkDuration = 100;
  address = (col / 8);
  whichCol =  col % 8;
  lc.setLed(address, whichCol, 0, false);
  delay(blinkDuration);
  lc.setLed(address, whichCol, 0, true);
  delay(blinkDuration);
  lc.setLed(address, whichCol, 0, false);
  delay(blinkDuration);
  lc.setLed(address, whichCol, 0, true);
  delay(blinkDuration);
  lc.setLed(address, whichCol, 0, false);
  delay(blinkDuration);
  lc.setLed(address, whichCol, 0, true);
  delay(blinkDuration);


}
void moveHeadTo(int col) {
  int stepDirection;
  
  //if we are moving away from the switch i.e. down
  if (abs(col) - abs(column) > 0) {
    stepDirection = -1;

  }
  else {
    stepDirection = 1;
  }
  //
  // col*=-1;
  //int pos = column;
  while (col != column) {
    column += stepDirection;
    myStepper.step(rowDist * stepDirection);

  }

}
void detachServos() {
  myservo.detach();  // attaches the servo on pin 9 to the servo object
  myservo1.detach();
}
void setRandomMatrix() {

  for (int row = 0; row < 32; row++) {
    bool cbl [8];
    for (int j = 0; j < 8; j++) {
      cbl[j] = true;
    }
    int limit = (int) random(7);
    for (int j = 0; j < limit; j++) {
      cbl[(int)random(7)] = false;
    }
    cbl[0] = true;
    writeToTapeRows(row, ToByte(cbl), 2);
  }

}
void setPatternedMatrix() {

  int col = 0;
  int add = 1;
  bool bl [8];
  for (int row = 0; row < 32; row++) {
    int ind = (int)random(8);
    for (int i = 0; i < 8; i++) {
      bl[i] = true;
    }
    //make just one true
    bl[col] = false;
    //  index++;
    //  if (index >= 8) {
    //    index = 0;
    //  }
    col += add;
    ////Serial.println(col);
    if (col <= 0 || col >= 8) {
      add *= -1;

    }

    writeToTapeRows(row, ToByte(bl), 2);
  }
}

byte getHeadState(int thresh) {
  //sensor values
  int sv0 = analogRead(rp7);
  int sv1 = analogRead(rp6);
  int sv2 = analogRead(rp5);
  int sv3 = analogRead(rp4);
  int sv4 = analogRead(rp3);
  int sv5 = analogRead(rp2);
  int sv6 = analogRead(rp1);
  //Serial.print(sv0);

  //an array to hold the results of the read
  bool headState [8];

  //set them all to false
  for (int i = 0; i < 8; i++) {
    headState[i] = false;
  }
  //now check if each reading is over the threshold
  if (sv0 > thresh) {
    headState[0] = true;
  }
  if (sv1 > thresh) {
    headState[1] = true;
  }
  if (sv2 > thresh) {
    headState[2] = true;
  }
  if (sv3 > thresh) {
    headState[3] = true;
  }
  if (sv4 > thresh) {
    headState[4] = true;
  }
  if (sv5 > thresh) {
    headState[5] = true;
  }
  if (sv6 > thresh) {
    headState[6] = true;
  }
  bool debug = false;
  if (debug) {
    Serial.print(sv0);
    Serial.print(" : ");
    Serial.print(headState[0]);
    Serial.print(" ; ");
    Serial.print(sv1);
    Serial.print(" : ");
    Serial.print(headState[1]); 
    Serial.print("; ");
    Serial.print(sv2);
    Serial.print(" : ");
    Serial.print(headState[2]);
    Serial.print("; ");
    Serial.print(sv3);
    Serial.print(" : ");
    Serial.print(headState[3]);
    Serial.print("; ");
    Serial.print(sv4);
    Serial.print(" : ");
    Serial.print(headState[4]);
    Serial.print("; ");
    Serial.print(sv5);
    Serial.print(" : ");
    Serial.print(headState[5]);
    Serial.print("; ");
    Serial.print(sv6);
    Serial.print(" : ");
    Serial.print(headState[6]);
    Serial.print("; ");
    Serial.println();
  }

  //  for (int i = 0; i < 7; i++) {
  //    //Serial.print(i);
  //    //Serial.print(" : ");
  //    //Serial.print(headState[i]);
  //    //Serial.print("; ");
  //
  //  }
  //  //Serial.println();
  headState[7] = false;
  byte state = ToByte(headState);

  Serial.write((state));
  return state;
}

void oneStepAtATime() {
  myStepper.step(rowDist * stepDirection);
  delay(2000);

  pos += rowDist * stepDirection;

  if (pos >= 0 || pos <= -dist + rowDist) {
    stepDirection *= -1;
  }

}

void setHeadLights(boolean  bo [8] ) {



  // b = FromByte(by);
  int ind = (int)random(8);
  for (int i = 0; i < 8; i++) {
    digitalWrite( indicators[i] , bo[7-i]);
  }
  //make just one true
  //digitalWrite( indicators[ind] ,LOW);
}



void liftHead() {
  myservo.write(posi[0]);
  myservo1.write(posi1[0]);

}

void dropHead() {
  myservo.write(posi[1]);
  myservo1.write(posi1[1]);
}

void setupServos(int pin1, int pin2) {
  myservo.attach(pin1);  // attaches the servo on pin 9 to the servo object
  myservo1.attach(pin2);

}
void setZeroPoint() {
  int switchState = 0;
  while (switchState == 0) {
    switchState = digitalRead(switchPin);
    myStepper.step(1);
    delay(10);

  }
  myStepper.step(-25);
  // delay(2000);
  myStepper.setSpeed(60);
  column = 0;
  ////Serial.println("found zero");
  //set zero point
  //stepperPos = 0;
}
void FromByte(unsigned char c, bool b[8])
{
  for (int i = 0; i < 8; ++i)
    b[i] = (c & (1 << i)) != 0;
}


void setupMatrix() {
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, intensity);
  /* and clear the display */
  lc.clearDisplay(0);

  lc.shutdown(1, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(1, intensity);
  /* and clear the display */
  lc.clearDisplay(1);

  lc.shutdown(2, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(2, intensity);
  /* and clear the display */
  lc.clearDisplay(2);


  lc.shutdown(3, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(3, intensity);
  /* and clear the display */
  lc.clearDisplay(3);

  //  for (int i = 0; i < 8; i++) {
  //    b[i] = false;
  //  }

}

//void EasyRows() {
//  for (int col = 0; col < 8; col++) {
//    lc.setRow(0, col, B11111111);
//    delay(delaytime);
//  }
//
//
//  for (int col = 0; col < 8; col++) {
//    lc.setRow(1, col, B11111111);
//    delay(delaytime);
//  }
//  for (int col = 0; col < 8; col++) {
//    lc.setRow(2, col, B11111111);
//    delay(delaytime);
//  }
//
//  for (int col = 0; col < 8; col++) {
//    lc.setRow(3, col, B11111111);
//    delay(delaytime);
//  }
//
//  //  lc.clearDisplay(0);
//  //  lc.clearDisplay(1);
//  //  lc.clearDisplay(2);
//  //  lc.clearDisplay(3);
//}

void writeToTapeRows(int col, byte value, int del) {
  int address;
  int whichCol;

  address = (col / 8);
  whichCol =  col % 8;
  ////Serial.print(col);
  ////Serial.print(" ");
  ////Serial.print(address);
  ////Serial.print(" ");
  ////Serial.println(whichCol);

  lc.setRow(address, whichCol, value);
  // delay(del);
  //  lc.clearDisplay(0);
  //  lc.clearDisplay(1);
  //  lc.clearDisplay(2);
  //  lc.clearDisplay(3);

  lc.setLed(address, whichCol, 0, true);
}
unsigned char ToByte(bool b[8])
{
  unsigned char c = 0;
  for (int i = 0; i < 8; ++i)
    if (b[i])
      c |= 1 << i;
  return c;
}

void setupStepper() {
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  // initialize the //Serial port:

  // set the motor speed (for multiple steps only):
  myStepper.setSpeed(60);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == 'x') {
      stringComplete = true;
    }
  }
}

