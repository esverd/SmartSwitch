#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>



// int encoderVal = 0;
// int encoderPinA = 5;
// int encoderPinB = 4;
// bool stateA = false;
// bool encoderAfirst = false;


int encoderPinA = 5;
int encoderPinB = 4;
int encoderVal = 90;
int encoderClickCalue = 10;
bool prevStateA = false;
bool currentStateA = false;


int encoderPinBtn = 2;
const int buttonPin = 2;    // the number of the pushbutton pin
int currentBtnState;             // the current currentReading from the input pin
int prevBtnState = LOW;   // the previous currentReading from the input pin
unsigned long prevDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers


Servo servoRotate;
Servo servoToggle;
int servoLocationPush = 175;
int servoLocationHome = 90;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinBtn, INPUT_PULLUP);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  pinMode(0, INPUT);

  servoRotate.attach(14);
  servoToggle.attach(12);
  servoToggle.write(servoLocationHome);

  EEPROM.begin(512);  //Initialize EEPROM
  Serial.print("The stored value is: ");
  Serial.println(EEPROM.read(128));
  servoLocationPush = (int)EEPROM.read(128);

}

void loop() 
{

  int analogVal = analogRead(0);
  int servoLocationPush = map(analogVal, 0, 1023, 0, 180);
  servoToggle.write(servoLocationPush);
  // Serial.println(mappedVal);

  int currentReading = digitalRead(buttonPin);    // read the state of the switch into a local variable:

  if (currentReading != prevBtnState)   // If the switch changed, due to noise or pressing, reset the debouncing timer
    prevDebounceTime = millis();

  if ((millis() - prevDebounceTime) > debounceDelay)  // whatever the currentReading is at, it's been there for longer than the debounce delay, so take it as the actual current state:
  {
    if (currentReading != currentBtnState)    // if the button state has changed:
    {
      currentBtnState = currentReading;

      if(!currentBtnState)
      {
        servoToggle.write(servoLocationHome);
        delay(600);
        servoToggle.write(servoLocationPush);
        delay(600);
        servoToggle.write(servoLocationHome);

        // EEPROM.write(128, servoLocationPush);  
        // EEPROM.commit();    //Store data to EEPROM
      }
    }
  }
  prevBtnState = currentReading;    // save the currentReading. Next time through the loop, it'll be the prevBtnState:


  currentStateA = digitalRead(encoderPinA);
  if (prevStateA && !currentStateA)     //if pinA has toggled 
  {
    if (!digitalRead(encoderPinB)) 
      encoderVal += encoderClickCalue;
    else 
      encoderVal -= encoderClickCalue;
    Serial.println (encoderVal);
    servoRotate.write(encoderVal);
    // servoToggle.write(encoderVal);
  }
  prevStateA = currentStateA;
}



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