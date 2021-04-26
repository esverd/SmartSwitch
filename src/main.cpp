#include <Arduino.h>
#include <Servo.h>
// #include <EEPROM.h>


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 16;   //4  D0
int encoderVal = 0;
int encoderClickValue = 5;
bool prevStateA = false;
bool currentStateA;

//-------ENCODER BUTTON-------
int encoderPinBtn = 12; //6  D6
int currentBtnState;             // the current currentReading from the input pin
int prevBtnState = true;   // the previous currentReading from the input pin
unsigned long msDebounceTimer = 0;  // the last time the output pin was toggled
const unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers

//-------SERVO CONFIG-------
int servoPinRotate = 5; //14
int servoPinPush = 4; //13
Servo servoRotate;
Servo servoPush;
int servoLocationPush = 160;    //from testing: 175;
int servoLocationHome = 90;

//-------PIN I/O-------
int MOSFET = 13; //7


//-------LIGHT-------
bool lightState = false;
unsigned int lightBrightness = 0;


//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
// void handleLight(bool, int);
void lightSetState(bool);

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinBtn, INPUT_PULLUP);

  pinMode(MOSFET, OUTPUT);

  servoRotate.attach(servoPinRotate);
  servoPush.attach(servoPinPush);
  servoPush.write(servoLocationHome);

  // EEPROM.begin(512);  //Initialize EEPROM
  // Serial.print("The stored value is: ");
  // Serial.println(EEPROM.read(128));
  // servoLocationPush = (int)EEPROM.read(128);

  currentBtnState = digitalRead(encoderPinBtn);
  digitalWrite(MOSFET, HIGH);

}

void loop() 
{
  checkEncoderBtn();
  checkEncoderRotation();
  
}

void checkEncoderBtn()
{
  bool currentReading = digitalRead(encoderPinBtn);    //read the state of the switch into a local variable

  if (currentReading != prevBtnState)   //ff the switch changed, due to noise or pressing, reset the debouncing timer
    msDebounceTimer = millis();

  if ((millis() - msDebounceTimer) > debounceDelay)  //whatever the currentReading is at, it's been there for longer than the debounce delay, so take it as the actual current state
  {
    if (currentReading != currentBtnState)    //if the button state has changed:
    {
      currentBtnState = currentReading;

      Serial.print("Button state = ");
      Serial.println(currentBtnState);

      lightSetState(!lightState);    //toggle the light
    }
  }
  prevBtnState = currentReading;    //save the currentReading. Next time through the loop, it'll be the prevBtnState:
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
    //change brightness
  }
  prevStateA = currentStateA;

}

void handleLight(bool on, int value)
{
  // int servoLocationPush = 175;
  // int servoLocationHome = 90;
  // servoToggle.write(servoLocationPush);
}

void lightSetState(bool state)
{
  if(state != lightState)
  {
    digitalWrite(MOSFET, HIGH);   //turns on power to the servos
    servoPush.write(servoLocationPush);
    delay(500);
    servoPush.write(servoLocationHome);
    // delay(100);
    // for (int i = servoLocationHome; i <= servoLocationPush; i++)
    // {
    //   servoPush.write(i);
    // }
    // delay(500);
    // for (int i = servoLocationPush; i >= servoLocationHome; i--)
    // {
    //   servoPush.write(i);
    // }
    
    lightState = state;
    digitalWrite(MOSFET, LOW);    //turns off power to the servos
  }
}


//increaseBrightness
//toggle light
//set to state
//set to brightness


// const int ledPin = 13;      // the number of the LED pin



// void setup() {
//   pinMode(buttonPin, INPUT);
//   pinMode(ledPin, OUTPUT);

//   // set initial LED state
//   digitalWrite(ledPin, ledState);
// }

// void loop() {
  
  
// }



// void loop()
// {
  // if(!digitalRead(encoderPinA))
  // {
  //   encoderVal--;
  //   Serial.println(encoderVal);
  // }
  // else if(!digitalRead(encoderPinB))
  // {
  //   encoderVal++;
  //   Serial.println(encoderVal);
  // }


  // if(digitalRead(encoderPinA) != digitalRead(encoderPinB))
  // {

  //   Serial.print("A = ");
  //   Serial.print(digitalRead(encoderPinA));
  //   Serial.print("        B = ");
  //   Serial.println(digitalRead(encoderPinB));
  // }
  // delay(10);

//   stateA = digitalRead()
//   if(digitalRead(encoderPinA) && !stateA)
//   {
//     if(!digitalRead(encoderPinB))
//       encoderVal--;
//     else
//       encoderVal++;
//     Serial.println(encoderVal);
//   }




// }





/* Read Quadrature Encoder
   Connect Encoder to Pins encoderPinA, encoderPinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

*/

// int encoderVal;
// int encoderPinA = 5;
// int encoderPinB = 4;
// int encoderVal = 128;
// int prevStateA = LOW;
// int currentStateA = LOW;


// int rotationPin = 14;

// void setup() {
//   pinMode (encoderPinA, INPUT_PULLUP);
//   pinMode (encoderPinB, INPUT_PULLUP);
//   pinMode(rotationPin, OUTPUT);
//   Serial.begin (115200);
// }

// void loop() {
//   currentStateA = digitalRead(encoderPinA);
//   if ((prevStateA == LOW) && (currentStateA == HIGH)) {
//     if (digitalRead(encoderPinB) == LOW) {
//       encoderVal--;
//     } else {
//       encoderVal++;
//     }
//     Serial.print (encoderVal);
//     // analogWrite(rotationPin, encoderVal);
//     Serial.print ("/");
//   }
//   prevStateA = currentStateA;
// }

// Servo myservo;  // create servo object to control a servo
// // twelve servo objects can be created on most boards

// int pos = 0;    // variable to store the servo position

// void setup() {
//   myservo.attach(14);  // attaches the servo on pin 9 to the servo object
// }

// void loop() {
//   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(15);                       // waits 15ms for the servo to reach the position
//   }
//   for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(15);                       // waits 15ms for the servo to reach the position
//   }
// }





// Servo myservo;  // create servo object to control a servo

// int potpin = 0;  // analog pin used to connect the potentiometer
// int encoderVal;    // variable to read the value from the analog pin

// void setup() 
// {
//   myservo.attach(6);  // attaches the servo on pin 9 to the servo object
// }

// void loop() 
// {
//   encoderVal = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
//   encoderVal = map(encoderVal, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
//   myservo.write(encoderVal);                  // sets the servo position according to the scaled value
//   delay(15);                           // waits for the servo to get there
// }


/* Example sketch to control a 28BYJ-48 stepper motor with ULN2003 driver board and Arduino UNO. More info: https://www.makerguides.com */
// Include the Arduino Stepper.h library:
// #include <Arduino.h>
// #include <Stepper.h>
// // Define number of steps per rotation:
// const int stepsPerRevolution = 2048;
// // Wiring:
// // Pin 8 to IN1 on the ULN2003 driver
// // Pin 9 to IN2 on the ULN2003 driver
// // Pin 10 to IN3 on the ULN2003 driver
// // Pin 11 to IN4 on the ULN2003 driver
// // Create stepper object called 'myStepper', note the pin order:
// Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

// int potpin = 0;  // analog pin used to connect the potentiometer
// int encoderVal;    // variable to read the value from the analog pin

// void setup() {
//   // Set the speed to 5 rpm:
//   myStepper.setSpeed(5);
//   // Begin Serial communication at a baud rate of 9600:
//   Serial.begin(9600);

//   pinMode(potpin, INPUT);
// }
// void loop() {

//   encoderVal = analogRead(potpin);
//   if(encoderVal < 300)
//     myStepper.step(1);
//   else if(encoderVal > 700)
//     myStepper.step(-1);
  
//   delay(5);

//   // Step one revolution in one direction:
//   // Serial.println("clockwise");
//   // myStepper.step(stepsPerRevolution);
//   // delay(500);
  
//   // // Step one revolution in the other direction:
//   // Serial.println("counterclockwise");
//   // myStepper.step(-stepsPerRevolution);
//   // delay(500);
// }