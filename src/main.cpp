#include <Arduino.h>
#include <Servo.h>
#include <WIFI_CREDS.h>     //remove this in your code. i just used it to keep various credentials secret on github


//-------ROTARY ENCODER-------
int encoderPinA = 14;   //5  D5
int encoderPinB = 13;   //16;   //4  D0
float encoderVal = 0.0;   //the value being changed by the encoder as it rotates
float encoderClickValue = 0.5;    //how much to change the brightness with one "click" on the encoder
bool prevStateA = true;
bool currentStateA = true;
unsigned long msWaitBeforeServoStart = 500;   //delay after encoder has finished rotating before starting to move the servo
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
int servoIncreaseBrightness = 100;    //speed for rotation
int servoDecreaseBrightness = 80;     //speed for rotation
unsigned long servoRotateTimeConstant = 100;    //in milliseconds. multiplied by envoderVal to get total time for motor to rotate

//-------PIN I/O-------
// int MOSFET = 13; //7

//-------LIGHT-------
bool globalLightState = false;
float globalLightBrightness = 0;

//-------------MQTT Settings------------- 
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

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;


//-------FUNCTION PROTOTYPES-------
void checkEncoderBtn();
void checkEncoderRotation();
void lightSetState(bool);
void lightChangeBrightness(float);
void lightSetBrightness(float);
void readSerial();
void setup_wifi();
void callback(char*, byte*, unsigned int);    //being called each time a MQTT message is received
void reconnect();
void sendStates();    //transfers the global states to home assistant over MQTT


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinBtn, INPUT_PULLUP);

  servoPush.attach(servoPinPush);
  servoPush.write(servoLocationHome);
  servoPush.detach();
  servoRotate.attach(servoPinRotate);
  servoRotate.write(90);
  servoRotate.detach();

  btnState = digitalRead(encoderPinBtn);
  // currentStateA = digitalRead(encoderPinA);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() 
{
  // readSerial();

  if (!client.connected()) 
    reconnect();

  client.loop();

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
    servoPush.attach(servoPinPush);
    servoPush.write(servoLocationPush);
    delay(800);
    servoPush.write(servoLocationHome);
    delay(200);
    // digitalWrite(MOSFET, LOW);    //turns off power to the servos
    servoPush.detach();
    globalLightState = state;

    sendStates();   //publish state change to MQTT
  }
}

void lightChangeBrightness(float change)
{
  if(!globalLightState)
    lightSetState(true);    //make sure light is on

  if(globalLightBrightness + change > 100)    //if the encoder is rotated to increase brightness beyond 100
    change = 100 - globalLightBrightness;         //set brightness to 100
  else if(globalLightBrightness + change < 0)     //if the change would give negative brightness
    change = -globalLightBrightness;      //decrease to 0 brightness

  servoRotate.attach(servoPinRotate);
  if(change > 0)
    servoRotate.write(servoIncreaseBrightness);
  else if(change < 0)
    servoRotate.write(servoDecreaseBrightness);
  
  delay(abs(change)*servoRotateTimeConstant);     //timer for starting the motor to a time corresponding to the changing brightness value
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

void setup_wifi() 
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