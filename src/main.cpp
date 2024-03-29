#include <Arduino.h>
#include <Servo.h>
#include <WIFI_CREDS.h>     //remove this in your code. i just used it to keep various credentials secret on github


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 13;   //16;   //4  D0
float encoderVal = 0.0;   //the value being changed by the encoder as it rotates
long encoderClickValue = 4.0;    //how much to change the brightness with one "click" on the encoder
bool prevStateA = true;
bool currentStateA = true;
unsigned long msWaitBeforeServoStart = 600;   //delay after encoder has finished rotating before starting to move the servo
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
int servoLocationPush = 168;    //from testing: 175;
int servoLocationHome = 130;
int servoPushPrevLocation;

//-------SERVO ROTATE CONFIG-------
int servoPinRotate = 5; //14
Servo servoRotate;
int servoIncreaseBrightness = 80;    //speed for rotation. 90 is the middle/still
int servoDecreaseBrightness = 100;     //speed for rotation
unsigned long servoRotateTimeConstant = 40;    //23 after calibrating one click at the time    //in milliseconds. multiplied by encoderVal to get total time for motor to rotate

//-------LIGHT-------
bool globalLightState = false;
float globalLightBrightness = 100;

//-------MQTT SETTINGS------- 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
const char* ssid = CREDS_WIFI_SSID;   //replace with your ssid
const char* password = CREDS_WIFI_PWD;  //replace with your wifi password
const char* mqtt_server = "192.168.1.101";
const char* mqttUser = CREDS_MQTT_USER;   //replace with your mqtt server user
const char* mqttPwd = CREDS_MQTT_PWD;     //replace with your mqtt server user password
const char* command_topic = "smartswitch/set";     //subscribe
const char* state_topic = "smartswitch/state/status";    //publish
const char* brightness_command_topic = "smartswitch/brightness";   //subscribe
const char* brightness_state_topic = "smartswitch/state/brightness";    //publish
const char* brightness_override_topic = "smartswitch/override/brightness";   //subscribe. to manually set the brightness state without change from servo

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

//-------OTA SETTINGS-------
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
void lightSetState(bool);
void lightChangeBrightness(float);
void lightSetBrightness(float);
void readSerial();
void setupWifi();
void callback(char*, byte*, unsigned int);    //being called each time a MQTT message is received
void reconnect();
void sendStates();    //transfers the global states to home assistant over MQTT
void setupOTA();
int sweepServoTo(Servo, int, int, int);


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinBtn, INPUT_PULLUP);

  servoPush.attach(servoPinPush);
  servoPush.write(servoLocationHome);
  delay(50);
  servoPush.detach();
  servoPushPrevLocation = servoLocationHome;

  servoRotate.attach(servoPinRotate);
  servoRotate.write(90);
  delay(50);
  servoRotate.detach();

  btnState = digitalRead(encoderPinBtn);
  // currentStateA = digitalRead(encoderPinA);

  setupWifi();    //connect to wifi
  setupOTA();     //set up OTA code upload
  client.setServer(mqtt_server, 1883);    //connect to MQTT server
  client.setCallback(callback);     //handle incoming MQTT messages  
}

void loop() 
{
  ArduinoOTA.handle();

  if (!client.connected()) 
    reconnect();

  client.loop();

  checkEncoderBtn();
  checkEncoderRotation();
  // readSerial();

  // unsigned long timer = millis();
  // servoRotate.attach(servoPinRotate);
  // while(!digitalRead(encoderPinBtn))
  // {
  //   servoRotate.write(servoDecreaseBrightness);
  // }
  // servoRotate.write(servoLocationHome);
  // client.publish("smartswitch/test", String(millis() - timer).c_str());

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

  prevStateA = currentStateA;   //save current state for next loop through function
}

void lightSetState(bool state)
{
  if(state != globalLightState)   //if light is getting toggled. ignore if state command is same as current state
  {
    servoPushPrevLocation = sweepServoTo(servoPush, servoPushPrevLocation, servoLocationPush, servoPinPush);
    delay(100);
    servoPushPrevLocation = sweepServoTo(servoPush, servoPushPrevLocation, servoLocationHome, servoPinPush);
    delay(50);

    globalLightState = state;

    sendStates();   //publish state change to MQTT
  }
}

void lightChangeBrightness(float change)
{
  if(!globalLightState)
    lightSetState(true);    //make sure light is on

  unsigned long extraTime = 0;
  if(globalLightBrightness + change > 100)    //if the encoder is rotated to increase brightness beyond 100
  {
    change = 100 - globalLightBrightness;         //set brightness to 100
    extraTime = 500;
  }
  else if(globalLightBrightness + change < 0)     //if the change would give negative brightness
  {
    change = -globalLightBrightness;      //decrease to 0 brightness
    extraTime = 500;
  }

  servoRotate.attach(servoPinRotate);
  if(change > 0)
    servoRotate.write(servoIncreaseBrightness);
  else if(change < 0)
    servoRotate.write(servoDecreaseBrightness);
  
  delay((abs(change)*servoRotateTimeConstant) + extraTime);     //timer for starting the motor to a time corresponding to the changing brightness value
  servoRotate.write(90);    //stops the motor
  servoRotate.detach();

  globalLightBrightness += change;    //updates global brightness variable
  
  sendStates();   //publish state change to MQTT
}

void lightSetBrightness(float brightness)
{
  if(brightness >= 100)
    brightness = 100;
  else if(brightness < 0)
    brightness = 0;

  if(brightness != globalLightBrightness)
    lightChangeBrightness(brightness - globalLightBrightness);

  Serial.print("Light state: ");
  Serial.println(globalLightState);  
}

int sweepServoTo(Servo inServo, int currentPosition, int destination, int servoPin)
{
  inServo.attach(servoPin);
  while(currentPosition != destination)   //prevents the servo getting told to move to the position it's already in
  {
    if(destination > currentPosition)     //100 -> 155
      currentPosition++;
    else
      currentPosition--;
    inServo.write(currentPosition);     //150 -> 90
    delay(8);
  }
  inServo.detach();
  return currentPosition;
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  String payloadString = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
    payloadString += (char)payload[i];
  }
  Serial.println();

  if(String(topic) == command_topic)
  {
    if(payloadString == "ON")
      lightSetState(true);
    else if(payloadString == "OFF")
      lightSetState(false);
  }
  else if(String(topic) == brightness_command_topic)
    lightSetBrightness(payloadString.toFloat());
  else if(String(topic) == brightness_override_topic)
  {
    globalLightBrightness = payloadString.toFloat();
    sendStates();
  }

}

void sendStates()
{
  if(globalLightState)
    client.publish(state_topic, "ON");
  else 
    client.publish(state_topic, "OFF");
  
  client.publish(brightness_state_topic, String(globalLightBrightness).c_str());
}

void reconnect() 
{
  while (!client.connected())   // Loop until we're reconnected
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqttUser, mqttPwd ))    // Attempt to connect
    {
      Serial.println("connected");
      client.subscribe(command_topic);
      client.subscribe(brightness_command_topic);
      client.subscribe(brightness_override_topic);
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);    // Wait 5 seconds before retrying
    }
  }
}

void setupWifi() 
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupOTA()
{
  ArduinoOTA.setHostname("ESP8266-SmartSwitch");
  ArduinoOTA.setPassword("ESPOTA");    // No authentication by default

  ArduinoOTA.onStart([]() 
  {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() 
  {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) 
  {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) 
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) 
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) 
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) 
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) 
      Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());
}


String cmdBuffer = ""; 
void readSerial()     //used for debugging
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