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

//replacement for delays uses Timer library also in github
Timer dropTimer;
Timer readTimer;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on the motor shield
Stepper myStepper(stepsPerRevolution, 12, 13);


#include "LedControl.h"


String inputString = "";         // a string to hold incoming data
//flag for when all message is received
boolean stringComplete = false;
//set as true when we hit the limit switch
bool zeroed = false;
bool debug = false;

//flags for main commands to separate serial receiving and execution
boolean goSetHeadState = false;
boolean goReadTape = false;
boolean goMoveHead = false;
boolean goWriteTape = false;
boolean goWriteTapeAt = false;
boolean goLightsToggle = false;
boolean goLightsOn = false;
boolean goLightsOff = false;
boolean goLightsSetPattern = false;

//globals for main command parameters
byte newHeadState;
int newHeadPos;
byte newSymbol;
int newColumn;

//each light has data associated
Timer lightStateTimers [32];
//the first two extra indices in this are a hack to get the freq and duty cycle
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
//32*54
int dist = 1728;
//prev entire distanc in steps was 1760 but appears to have changed...
//int dist = 1760;
int switchPin = 2;
//how many  steps between rows
int rowDist = dist / 32;

//these are out LDR pins
const int rp7 = A8;
const int rp6 = A9;
const int rp5 = A10;
const int rp4 = A11;
const int rp3 = A12;
const int rp2 = A13;
const int rp1 = A14;

//for serial comms - triggers stringcomplete flag
char lf = '\n';

///maximum length of incoming data in number of characters
const int charLength = 100;
char serialdata[charLength];

//for the matrix
int intensity = 15;

//holds the machine state
int indicators[8];

unsigned long delaytime = 100;

int counter = 30;
//used for ligth setting
bool b[8];

int x = 0;
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


void setup() {
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

  pinMode(switchPin, INPUT);
  ///pull up the resistor on the switch pin
  digitalWrite(switchPin, HIGH);

  //set the machine state pins
  for (int i = 7; i >= 0; i--) {
    indicators[i] = i + 46;
    pinMode(indicators[i], OUTPUT);
  }


  setupMatrix();
  setRandomMatrix();
  //servos are attached to pins 4 and 6
  setupServos(4, 6);
  delay(500);
  //  liftHead();
  //  delay(500);
  //send the head up until it hits the limit switch
  //

  setZeroPoint();

  delay(500);
}


void loop() {

  ///strign complete flag is set when we get a terminating character (x) in the serial event
  if (stringComplete) {

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

    ///no depending on the dataname we have raise the appropriate flag
    if (dataname == "readTape") {
      goReadTape = true;

    }
    //write to tape
    else if (dataname == "writeTape") {
      newSymbol = value.toInt();
      goWriteTape = true;

    }
    else if (dataname == "lightsToggle") {
      //populate lightIndices array with the indices of the leds we are interested in taken from serial message
      listFromString(lightIndices , value, ' ');
      goLightsToggle = true;

    }
    else if (dataname == "lightsOn") {
      //populate lightIndices array with the indices of the leds we are interested in taken from serial message

      listFromString(lightIndices , value, ' ');
      goLightsOn = true;

    }
    else if (dataname == "lightsOff") {
      //populate lightIndices array with the indices of the leds we are interested in taken from serial message

      listFromString(lightIndices , value, ' ');
      goLightsOff = true;

    }
    else if (dataname == "lightsSetPattern") {
      //populate lightIndices array with the indices of the leds we are interested in taken from serial message

      floatListFromString(tempLightIndices, value, ' ');
      //this time I've cheated and used the first two index positions to store freq and duty cycle
      lightsFreq = tempLightIndices[0];
      lightsDutyCycle =  tempLightIndices[1];

      float onDuration = lightsFreq *  lightsDutyCycle;

      for (int i = 0; i < 32; i++) {
        lightIndices[i] = tempLightIndices[i + 2];
        if (lightIndices[i] != -1) {
          //I"m using the Timer class to remmeber what the initial freq was - it;s just more convenient to store it here but isn't great for the OOP clarity
          lightStateTimers[lightIndices[i]].start(onDuration);
          lightStateTimers[lightIndices[i]].setFreq(lightsFreq);
          lightStateTimers[lightIndices[i]].setDutyCycle(lightsDutyCycle);
        }
      }
      goLightsSetPattern = true;

    }
    //this command contains two values separated by a colon so involves a little further string splitting
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
    ///empty out the string that holds our serial data
    inputString = "";
    //rest the flag
    stringComplete = false;


  }
  ///now lets do whatever action we need to
  if (goReadTape) {

    dropHead();

    //ADELAY
    //rather than a delay we use the timer class to be non blocking
    if (!dropTimer.isRunning()) {
      dropTimer.start(500);
    }
    //once we're sure the head is down, read it and then start the timer to read it before we
    if (dropTimer.timeIsUp()) {
      dropTimer.stop();
      getHeadState(500);
      readTimer.start(100);
    }
    //we're done reading so lift the head and set the action flag (goReadTape) to false
    if (readTimer.timeIsUp()) {
      liftHead();
      goReadTape = false;
      readTimer.stop();
    }

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

    //movehead to returns true when it reaches its destination
    if (!moveHeadTo(-1 * newHeadPos)) {
      //do nothing while we're still moving
    }
    else {
      //send message to control software that we have arrived
      Serial.write((253));
      goMoveHead = false;
    }
    

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


///after all those choices deal with any flashing lights that might be around
  for (int i = 0; i < 32; i++) {
    //if this particular light is flagged as flashing
    if (lightFlashingStates[i]) {
      //and it's timer has pinged
      if (lightStateTimers[i].timeIsUp()) {

        //if it's on then turn it off, set its state flag to off, stop its timer and restart the timer with the off part of its duty cycle
        if (lightStates[i]) {
          setIndicator(i , false);
          lightStates[i] = false;
          lightStateTimers[i].stop();
          lightStateTimers[i].start(float(lightStateTimers[i].getFreq()) * (1.0f - lightStateTimers[i].getDutyCycle()));

        }
        //if it's off then do the opposite of course
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
  //the matrix is made up of 8 by 8 panels so we need to know which panel (address) and the remainder is the appropriate column
  address = (col / 8);
  whichCol =  col % 8;
  lc.setLed(address, whichCol, 0, isOn);
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

//to indicate machine state
void setHeadLights(boolean  bo [8] ) {

  for (int i = 0; i < 8; i++) {
    digitalWrite( indicators[i] , bo[7 - i]);
  }
 
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
  myStepper.setSpeed(60);
  column = 0;
 
}

//not use but might be useful!
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

//this function and the below will explode a string into an array given a delimiter
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
        if(listIndex<32) listIndex++;
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
        if(listIndex<32) listIndex++;
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

