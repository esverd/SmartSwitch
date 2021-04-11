#include <Arduino.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() 
{
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
}

void loop() 
{
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}


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
// int val;    // variable to read the value from the analog pin

// void setup() {
//   // Set the speed to 5 rpm:
//   myStepper.setSpeed(5);
//   // Begin Serial communication at a baud rate of 9600:
//   Serial.begin(9600);

//   pinMode(potpin, INPUT);
// }
// void loop() {

//   val = analogRead(potpin);
//   if(val < 300)
//     myStepper.step(1);
//   else if(val > 700)
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