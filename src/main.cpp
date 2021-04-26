#include <Arduino.h>
#include <Servo.h>
// #include <EEPROM.h>


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 16;   //4  D0
int encoderVal = 0;
int encoderClickValue = 1;
bool prevStateA = true;
bool currentStateA;
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
bool lightState = false;
unsigned float lightBrightness = 0;


//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
void lightSetState(bool);
void lightChangeBrightness(float);
void lightSetBrightness(unsigned float);

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
  currentStateA = digitalRead(encoderPinA);

}

void loop() 
{
  checkEncoderBtn();
  checkEncoderRotation();
  
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
    lightSetState(!lightState);    //toggle the light
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
  if(state != lightState)
  {
    // digitalWrite(MOSFET, HIGH);   //turns on power to the servos
    servoPush.attach(servoPinPush);
    servoPush.write(servoLocationPush);
    delay(500);
    servoPush.write(servoLocationHome);
    lightState = state;
    // digitalWrite(MOSFET, LOW);    //turns off power to the servos
    servoPush.detach();
  }
}

void lightChangeBrightness(float change)
{
  if(lightBrightness + change > 100)    //if the encoder is rotated to increase brightness beyond 100
    lightSetBrightness(100);          //set brightness to 100
  else if(lightBrightness + change < 0)
    lightSetBrightness(0);

  servoRotate.attach(servoPinRotate);
  if(change > 0)
    servoRotate.write(servoIncreaseBrightness);
  else 
    servoRotate.write(servoDecreaseBrightness);
  delay(abs(change)*servoRotateTimeConstant);     //times for starting the motor to a time corresponding to the changing brightness value

  servoRotate.write(90);    //stops the motor
  servoRotate.detach();

  lightBrightness += change;    //updates global brightness variable
}

void lightSetBrightness(unsigned float brightness)
{
  if(brightness > 100)
    brightness = 100;
  lightChangeBrightness(brightness - lightBrightness);
}

