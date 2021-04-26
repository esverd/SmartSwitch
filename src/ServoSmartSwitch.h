
#include "esphome.h"
#include "Servo.h"
// #include <EEPROM.h>

class ServoSmartSwitch : public Component, public LightOutput {
 public:
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
  //remove function prototypes

  void setup() override {
    // This will be called by App.setup()
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

    // register_service(&ServoSmartSwitch::turn_on, true);
    // ServoSmartSwitch.set_default_transition_length(10000);

  }

  void loop() override {
    checkEncoderBtn();
    checkEncoderRotation();
  }

  LightTraits get_traits() override {
    // return the traits this light supports
    auto traits = LightTraits();
    traits.set_supports_brightness(true);
    traits.set_supports_rgb(false);
    traits.set_supports_rgb_white_value(false);
    traits.set_supports_color_temperature(false);
    return traits;
  }

  void write_state(LightState *state) override {
    // This will be called by the light to get a new state to be written.
    // float red, green, blue;
    // use any of the provided current_values methods
    // state->current_values_as_rgb(&red, &green, &blue);
    // Write red, green and blue to HW
    // ...
    state->set_default_transition_length(0);
    // float brightness2;
    float brightnessTemp;
    state->current_values_as_brightness(&brightnessTemp);
    // Convert to 0-100
    int brightness = floor(brightnessPercent * 100);
    // brightness2 = brightness;
    lightSetBrightness(brightness);

    // state->
    //if state has changed, call function for servo push

    // char buffer[140];
    // snprintf_P(buffer, sizeof(buffer), PSTR("AT+UPDATE=\"sequence\":\"%d%03d\",\"switch\":\"%s\",\"light_type\":1,\"colorR\":%d,\"colorG\":%d,\"colorB\":%d,\"bright\":%d,\"mode\":%d"),
    // millis(), millis()%1000,
    // ledState ? "on" : "off",
    // redValue, greenValue, blueValue,
    // brightness,
    // SONOFF_L1_MODE_COLORFUL);

  }

  

  // LightCall LightState::turn_on() override { 
  //   lightSetState(true);
  //   return this->make_call().set_state(true); 
  // }

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
    float orig = brightness;

    // brightness *= 100;
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
    Serial.print(globalLightState);
    Serial.print("    Brightness given: ");
    Serial.print(orig);
    Serial.print("    Brightness global: ");
    Serial.println(globalLightBrightness);
    
  }


};


