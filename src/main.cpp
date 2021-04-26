#include <Arduino.h>
#include <Servo.h>
// #include <EEPROM.h>


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 16;   //4  D0
int encoderVal = 0;
int encoderClickValue = 1;
bool prevStateA = true;
bool currentStateA = true;
unsigned long msWaitBeforeServoStart = 500;
unsigned long msWaitServoTimer = 0;
bool goingToRotate = false;

//-------ENCODER BUTTON-------
int encoderPinBtn = 12; //6  D6
int btnState;             // the current nowReading from the input pin. used for debugging can be removed later
int prevBtnReading = true;   // the previous nowReading from the input pin
unsigned long msDebounceTimer = 0;  // the last time the output pin was toggled
const unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers

//-------SERVO PUSH CONFIG-------
int servoPinPush = 4; //13
Servo servoPush;
int servoLocationPush = 160;    //from testing: 175;
int servoLocationHome = 90;

//-------SERVO ROTATE CONFIG-------
int servoPinRotate = 5; //14
Servo servoRotate;
int servoIncreaseBrightness = 100;
int servoDecreaseBrightness = 80;
unsigned long servoRotateTimeConstant = 100;    //in milliseconds

//-------PIN I/O-------
// int MOSFET = 13; //7


//-------LIGHT-------
bool globalLightState = false;
float globalLightBrightness = 0;


//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
void lightSetState(bool);
void lightChangeBrightness(float);
void lightSetBrightness(float);
void readSerial();

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinBtn, INPUT_PULLUP);

  servoPush.attach(servoPinPush);
  servoPush.write(servoLocationHome);
  servoPush.detach();
  servoRotate.attach(servoPinRotate);
  servoRotate.write(90);
  servoRotate.detach();

  btnState = digitalRead(encoderPinBtn);
  // currentStateA = digitalRead(encoderPinA);

}

void loop() 
{
  checkEncoderBtn();
  checkEncoderRotation();

  readSerial();
  
}

void checkEncoderBtn()
{
  bool nowReading = digitalRead(encoderPinBtn);    //read the state of the switch into a local variable

  if (nowReading != prevBtnReading)   //if the reading of the button has changed due to noise or pressing
    msDebounceTimer = millis();       //start debounce timer

  if (((millis() - msDebounceTimer) > debounceDelay) && !nowReading)  //if debounce timer is up and button is still being pressed
  {
    while(!digitalRead(encoderPinBtn)) {}    //wait for button to be released
    btnState = !btnState;    //toggle the button state
    Serial.print("Button state = ");
    Serial.println(btnState);
    lightSetState(!globalLightState);    //toggle the light
    msDebounceTimer = millis();       //restart debounce timer to break out of statement
  }
  prevBtnReading = nowReading;    //save the nowReading. Next time through the loop, it'll be the prevBtnReading:
}

void checkEncoderRotation()
{
  currentStateA = digitalRead(encoderPinA);
  
  if(currentStateA != prevStateA)
  {
    if(digitalRead(encoderPinB) != currentStateA)
      encoderVal += encoderClickValue;
    else
      encoderVal -= encoderClickValue;

    Serial.println(encoderVal);
    msWaitServoTimer = millis();    //reset timer each time encoder has rotated
    goingToRotate = true;     //flag that the brightness is going to change
  }

  if(((millis() - msWaitServoTimer) > msWaitBeforeServoStart) && goingToRotate)   //if encoder has stopped rotating and the brightness is going to change
  {
    lightChangeBrightness(encoderVal);     //change brightness
    goingToRotate = false; 
    encoderVal = 0;
  }

  prevStateA = currentStateA;
}

void lightSetState(bool state)
{
  if(state != globalLightState)   //if light is getting toggled. ignore if state command is same as current state
  {
    // digitalWrite(MOSFET, HIGH);   //turns on power to the servos
    servoPush.attach(servoPinPush);
    servoPush.write(servoLocationPush);
    delay(600);
    servoPush.write(servoLocationHome);
    // digitalWrite(MOSFET, LOW);    //turns off power to the servos
    servoPush.detach();
    globalLightState = state;
  }
}

void lightChangeBrightness(float change)
{
  if(globalLightBrightness + change > 100)    //if the encoder is rotated to increase brightness beyond 100
    lightSetBrightness(100);          //set brightness to 100
  else if(globalLightBrightness + change < 0)
    lightSetBrightness(0);

  servoRotate.attach(servoPinRotate);
  if(change > 0)
    servoRotate.write(servoIncreaseBrightness);
  else 
    servoRotate.write(servoDecreaseBrightness);
  delay(abs(change)*servoRotateTimeConstant);     //times for starting the motor to a time corresponding to the changing brightness value

  servoRotate.write(90);    //stops the motor
  servoRotate.detach();

  globalLightBrightness += change;    //updates global brightness variable
}

void lightSetBrightness(float brightness)
{
  if(brightness >= 100)
    brightness = 100;
  else if(brightness < 0)
    brightness = 0;
  
  if((brightness == 0))       //if brightness is set to 0 and the light is currently on
  {
    lightSetState(false);
    // globalLightState = false;
  }
  else if((brightness != globalLightBrightness) || ((brightness > 0) && !globalLightState) )   //if brightness has changed while light is on, OR, if light is off when a brightness value is gived
  {
    lightSetState(true);    //make sure light is on
    lightChangeBrightness(brightness - globalLightBrightness);
  }

    //break; dont rotate servo

  //if OFF and changed, turn on, adjust lights
  //if ON and less than 0, turn off
  //update global value
  

  Serial.print("Light state: ");
  Serial.println(globalLightState);
  
}

String cmdBuffer = ""; 

void readSerial()
{
  //if serial available
  //send start signal to receive new commands
  //store commands in buffer
  //\n is end of command
  //process buffer by handling GCODE
  //loop

  float inVal;

  if(Serial.available() > 0)    //if serial communication is avalable
  {
    char inChar = Serial.read();    //reads next character

    if(inChar == '\n')    //if string ended with new line process command
    {
      inVal = cmdBuffer.toFloat();
      lightSetBrightness(inVal);
      // handleGCODE();    //processes the received commans
      Serial.print("Given brightness: ");   //prints GO to signal the arduino is ready for the next command
      Serial.println(inVal);
      cmdBuffer = "";
    }
    else
      cmdBuffer += inChar;    //stores the incomming character string
  }

}

// if(cmdBuffer.indexOf("G1") != -1 || cmdBuffer.indexOf("G01") != -1)
//   {
//     if(cmdBuffer.indexOf("X") != -1 || cmdBuffer.indexOf("Y") != -1 || cmdBuffer.indexOf("Z") != -1)
//       G1xyz();   //moves the robot to the given coordinates
//     else if(cmdBuffer.indexOf("L") != -1 || cmdBuffer.indexOf("R") != -1)
//       G1lr();
//   }

//   if(yVal.toFloat() != -1)
//         currentY = yVal.toFloat();