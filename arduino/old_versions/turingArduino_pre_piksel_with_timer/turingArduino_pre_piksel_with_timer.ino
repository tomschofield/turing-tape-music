#include <Timer.h>

/*

   TURING TAPE MUSIC
   MIT License

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

Timer dropTimer;
Timer readTimer;
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on the motor shield
Stepper myStepper(stepsPerRevolution, 12, 13);


#include "LedControl.h"


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;
bool zeroed = false;
bool debug = false;
boolean finishedAction = true;

boolean goSetHeadState = false;
boolean goReadTape = false;
boolean goMoveHead = false;
boolean goWriteTape = false;
boolean goWriteTapeAt = false;
boolean goLightsToggle = false;
boolean goLightsOn = false;
boolean goLightsOff = false;
boolean goLightsSetPattern = false;


byte newHeadState;
int newHeadPos;
byte newSymbol;
int newColumn;

Timer lightStateTimers [32];
float tempLightIndices [34];
int lightIndices [32];
boolean lightStates [32];
boolean lightFlashingStates [32];

int lightsFreq;
float lightsDutyCycle;


// give the motor control pins names:
const int pwmA = 3;
const int pwmB = 11;
const int brakeA = 9;
const int brakeB = 8;
const int dirA = 12;
const int dirB = 13;
//64*25
int dist = 1728;

//int dist = 1760;
int switchPin = 2;
int rowDist = dist / 32;

const int rp7 = A8;
const int rp6 = A9;
const int rp5 = A10;
const int rp4 = A11;
const int rp3 = A12;
const int rp2 = A13;
const int rp1 = A14;

//for serial comms
char lf = '\n';

///maximum length of incoming data in number of characters
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

void setup() {
  //Serial.begin(9600);
  // set the PWM and brake pins so that the direction pins  // can be used to control the motor:
  setupStepper();

  for (int i = 0; i < 32; i++) {
    lightStates[i] = false;
    lightFlashingStates[i] = false;
    lightIndices[i] = -1;
    tempLightIndices[i] = -1;
  }

  Serial.begin(9600);

  ////fill th incoming data with an identifiable character not in the
  for (int i = 0; i < charLength; i++) {
    serialdata[i] = '!';
  }
  Serial.println(serialdata);

  pinMode(switchPin, INPUT);
  ///pull up the resistor on the switch pin
  digitalWrite(switchPin, HIGH);


  for (int i = 7; i >= 0; i--) {
    indicators[i] = i + 46;
    pinMode(indicators[i], OUTPUT);
  }


  setupMatrix();
  setRandomMatrix();
  //servos are attached to pins 4 and 6
  setupServos(4, 6);
  delay(500);
  liftHead();
  delay(500);
  //send the head up until it hits the limit switch
  //

  //setZeroPoint();

  delay(500);
}


void loop() {

  ///strign complete flag is set when we get a terminating character (x) in the serial event
  if (stringComplete) {
    finishedAction = false;

    for (int i = 0; i < charLength; i++) {
      serialdata[i] = '!';
    }
    //the incoming data is in name:value pairs
    String dataname = "";
    String value = "" ;
    Serial.print("got data");
    boolean addToValue = false;

    //go through the string from the serial
    for (int i = 0; i < inputString.length(); i++) {
      //once we are at the delimiting character stop adding to the name and add to the value instead
      if (inputString.charAt(i) == ':' && addToValue == false) {
        addToValue = true;
        i++;
      }
      ///my way of
      if (inputString.charAt(i)  != '!') {
        if (addToValue) {

          value += inputString.charAt(i) ;
        }
        else {
          dataname += inputString.charAt(i) ;
        }
      }
    }
    if (dataname == "readTape") {
      goReadTape = true;

    }
    //write to tape
    else if (dataname == "writeTape") {
      newSymbol = value.toInt();
      goWriteTape = true;

    }
    else if (dataname == "lightsToggle") {

      listFromString(lightIndices , value, ' ');
      goLightsToggle = true;

    }
    else if (dataname == "lightsOn") {

      listFromString(lightIndices , value, ' ');
      goLightsOn = true;

    }
    else if (dataname == "lightsOff") {

      listFromString(lightIndices , value, ' ');
      goLightsOff = true;

    }
    else if (dataname == "lightsSetPattern") {
      /// tempLightIndices
      floatListFromString(tempLightIndices, value, ' ');

      lightsFreq = tempLightIndices[0];
      lightsDutyCycle =  tempLightIndices[1];

      float onDuration = lightsFreq *  lightsDutyCycle;

      for (int i = 0; i < 32; i++) {
        lightIndices[i] = tempLightIndices[i + 2];
        if (lightIndices[i] != -1) {
          lightStateTimers[lightIndices[i]].start(onDuration);
          lightStateTimers[lightIndices[i]].setFreq(lightsFreq);
          lightStateTimers[lightIndices[i]].setDutyCycle(lightsDutyCycle);
        }
      }
      goLightsSetPattern = true;

    }

    else if (dataname == "writeTapeAt") {
      boolean isColumn = true;
      String column = "";
      String symbol = "";
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

      newSymbol = symbol.toInt();
      newColumn = 31 - column.toInt();
      goWriteTapeAt = true;
    }
    //move head
    else if (dataname == "moveHead") {
      newHeadPos = value.toInt();
      goMoveHead = true;

    }
    //set head state
    else if (dataname == "setHeadState" ) {
      newHeadState = value.toInt();
      goSetHeadState = true;

    }
    inputString = "";
    stringComplete = false;


  }
  ///now lets do whatever action we need to
  if (goReadTape) {

    dropHead();

    //ADELAY
    //delay(500);
    if (!dropTimer.isRunning()) {
      dropTimer.start(500);
    }

    if (dropTimer.timeIsUp()) {
      dropTimer.stop();
      getHeadState(500);
      readTimer.start(500);
    }
    //ADELAY
    if (readTimer.timeIsUp()) {
      liftHead();
      goReadTape = false;
      readTimer.stop();
    }
    //delay(500);

  }

  if (goLightsToggle) {
    for (int i = 0; i < 32; i++) {
      if (lightIndices[i] != -1) {
        if (lightStates[lightIndices[i]]) {
          //set light on
          setIndicator(lightIndices[i], false);
          lightStates[lightIndices[i]] = false;
        }
        else {
          // setLight off
          setIndicator(lightIndices[i], true);
          lightStates[lightIndices[i]] = true;
        }

      }

    }


    goLightsToggle = false;
  }
  if (goLightsOn) {
    for (int i = 0; i < 32; i++) {
      if (lightIndices[i] != -1) {
        //set light on
        setIndicator(lightIndices[i], true);
        lightStates[lightIndices[i]] = true;
        lightFlashingStates[lightIndices[i]] = false;
      }
    }
    goLightsOn = false;
  }
  if (goLightsOff) {
    for (int i = 0; i < 32; i++) {
      if (lightIndices[i] != -1) {
        //set light on
        setIndicator(lightIndices[i], false);
        lightStates[lightIndices[i]] = false;
        lightFlashingStates[lightIndices[i]] = false;
      }
    }
    goLightsOff = false;
  }

  //mark the flashing lights as havign flashing state
  if (goLightsSetPattern) {
    for (int i = 0; i < 32; i++) {
      if (lightIndices[i] != -1) {
        lightFlashingStates[lightIndices[i]] = true;
        //lightsFreq
      }
    }
    goLightsSetPattern = false;
  }
  //write to tape
  if ( goWriteTape) {
    //Serial.println(newSymbol);
    writeToTapeRows(31 + column, newSymbol, 2);
    goWriteTape = false;
  }
  if (goWriteTapeAt) {
    writeToTapeRows(newColumn, newSymbol, 2);
    goWriteTapeAt = false;
  }
  //move head
  if (goMoveHead) {

    //inByte -= 128;

    //if (stringComplete

    //blinkIndicator(31 - newHeadPos);
    //  Serial.println(newHeadPos);

    if (!moveHeadTo(-1 * newHeadPos)) {


    }
    else {
      Serial.write((253));
      goMoveHead = false;
    }
    //send message to control software that we have arrived


  }
  //set head state
  if (goSetHeadState ) {
    bool lights [8];

    for (int i = 0; i < 8; i++) {
      lights[i] = true;
    }

    // Serial.println(newHeadState);
    FromByte(newHeadState, lights);
    setHeadLights(lights);
    goSetHeadState = false;

  }

  ///flash lights
  //setIndicator(lightIndices[i], true);
  for (int i = 0; i < 32; i++) {
    if (lightFlashingStates[i]) {

      //      if (lightStateTimers[i].timeIsUp()) {
      //        long duration = lightStateTimers[i].getDuration();
      //        float dutyCycle = lightStateTimers[i].getDutyCycle();
      //        //if the light was on
      //        if (lightStates[i]) {
      //
      //          lightStateTimers[i].stop();
      //          //reset the timer
      //          setIndicator(i, false);
      //          ///set it to off for the other part of the cycle
      //          //int newFreq = duration * (1.0f - dutyCycle);
      //          lightStateTimers[i].start(500);
      //        }
      //        //if light was off
      //        else {
      //          lightStateTimers[i].stop();
      //          setIndicator(i, true);
      //          ///set it to off for the other part of the cycle
      //          int newFreq = duration *  dutyCycle;
      //          lightStateTimers[i].start(500);
      //
      //        }
      //
      //      }

      if (lightStateTimers[i].timeIsUp()) {
        if (lightStates[i]) {
          setIndicator(i , false);
          lightStates[i] = false;
          lightStateTimers[i].stop();
          lightStateTimers[i].start(float(lightStateTimers[i].getFreq()) * (1.0f - lightStateTimers[i].getDutyCycle()));

        }
        else {
          setIndicator(i , true);
          lightStates[i] = true;
          lightStateTimers[i].stop();
          lightStateTimers[i].start(float(lightStateTimers[i].getFreq())*lightStateTimers[i].getDutyCycle());
        }
      }

    }

  }

}
void setIndicator(int col, boolean isOn) {
  int address;
  int whichCol;
  int blinkDuration = 100;
  address = (col / 8);
  whichCol =  col % 8;
  lc.setLed(address, whichCol, 0, isOn);
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
  //ADELAY
  delay(blinkDuration);


}
boolean moveHeadTo(int col) {
  int stepDirection;
  boolean arrived = false;

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
  if (col != column) {
    column += stepDirection;
    myStepper.step(rowDist * stepDirection);

  }
  else {
    arrived = true;
  }
  return arrived;
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
    digitalWrite( indicators[i] , bo[7 - i]);
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
void setZeroPointNoWhile() {
  if (!zeroed) {
    int switchState = 0;
    switchState = digitalRead(switchPin);

    if (switchState == 0) {

      myStepper.step(1);
      delay(10);

    }
    else {
      zeroed = true;
      myStepper.step(-25);
      // delay(2000);
      myStepper.setSpeed(60);
      column = 0;
    }
  }

}
void listFromString(int list[32] , String raw, char delimiter) {
  int listIndex = 0;
  boolean collect = true;
  boolean pCollect = true;
  String thisNumber = "";
  raw += delimiter;

  for (int i = 0; i < raw.length(); i++) {
    char l = raw.charAt(i);
    //Serial.println(l);
    if (l == delimiter) {
      collect = false;
    }
    else {
      collect = true;
    }

    if (collect) {
      thisNumber += (String)l;
    }
    else {
      if (thisNumber.length() > 0) {
        list[listIndex] = thisNumber.toInt();
        thisNumber = "";
        listIndex++;
      }
    }
  }
  //fill up the rest with -1
  for (int i = listIndex; i < 32; i++) {
    list[i] = -1;
  }
}

void floatListFromString(float list[32] , String raw, char delimiter) {
  int listIndex = 0;
  boolean collect = true;
  boolean pCollect = true;
  String thisNumber = "";
  raw += delimiter;

  for (int i = 0; i < raw.length(); i++) {
    char l = raw.charAt(i);
    //Serial.println(l);
    if (l == delimiter) {
      collect = false;
    }
    else {
      collect = true;
    }

    if (collect) {
      thisNumber += (String)l;
    }
    else {
      if (thisNumber.length() > 0) {
        list[listIndex] = thisNumber.toFloat();
        thisNumber = "";
        listIndex++;
      }
    }
  }
  //fill up the rest with -1
  for (int i = listIndex; i < 32; i++) {
    list[i] = -1;
  }
}
void FromByte(unsigned char c, bool b[8])
{
  for (int i = 0; i < 8; ++i)
    b[i] = (c & (1 << i)) != 0;
}


//initiate the max7219 driver objects
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
}

void writeToTapeRows(int col, byte value, int del) {
  int address;
  int whichCol;

  address = (col / 8);
  whichCol =  col % 8;

  lc.setRow(address, whichCol, value);
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

