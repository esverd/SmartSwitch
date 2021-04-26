#include <Arduino.h>
#include <Servo.h>
// #include <EEPROM.h>


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 16;   //4  D0
int encoderVal = 0;
int encoderClickValue = 2;
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
int servoIncreaseBrightness = 105;
int servoDecreaseBrightness = 75;
unsigned long servoRotateTimeConstant = 100;    //in milliseconds

//-------PIN I/O-------
int MOSFET = 13; //7


//-------LIGHT-------
bool lightState = false;
unsigned int lightBrightness = 0;


//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
void lightSetState(bool);
void lightChangeBrightness(int);
void lightSetBrightness(unsigned int);

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinBtn, INPUT_PULLUP);

  pinMode(MOSFET, OUTPUT);

  // servoRotate.attach(servoPinRotate);
  servoPush.attach(servoPinPush);
  servoPush.write(servoLocationHome);
  servoPush.detach();
  servoRotate.attach(servoPinRotate);
  servoRotate.write(90);
  servoRotate.detach();

  // EEPROM.begin(512);  //Initialize EEPROM
  // Serial.print("The stored value is: ");
  // Serial.println(EEPROM.read(128));
  // servoLocationPush = (int)EEPROM.read(128);

  btnState = digitalRead(encoderPinBtn);
  currentStateA = digitalRead(encoderPinA);
  digitalWrite(MOSFET, HIGH);

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
  
  if(currentStateA != prevStateA) // || (!currentReadingB && prevStateB))
  {
    if(digitalRead(encoderPinB) != currentStateA)
      encoderVal += encoderClickValue;
    else
      encoderVal -= encoderClickValue;

    Serial.println(encoderVal);
    msWaitServoTimer = millis();
    goingToRotate = true;
  }

  if(((millis() - msWaitServoTimer) > msWaitBeforeServoStart) && goingToRotate)
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

void lightChangeBrightness(int change)
{
  servoRotate.attach(servoPinRotate);
  // lightBrightness
  if(change > 0)
    servoRotate.write(servoIncreaseBrightness);
  else 
    servoRotate.write(servoDecreaseBrightness);
  delay(abs(change)*servoRotateTimeConstant);
  servoRotate.write(90);
  lightBrightness += change;
  servoRotate.detach();
}

void lightSetBrightness(unsigned int brightness)
{
  lightChangeBrightness(lightBrightness - brightness);
}

