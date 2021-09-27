/***************************************************
  Bobblemeter control by Reach and Teach to allow IFTTT
  to control digital output and a servo output from HUZZAH Breakout

  Uses code from  Adafruit MQTT Library ESP8266 Example by Tony DiCola

  Permission to use granted under Creative Commons Attribution + ShareAlike

  9/24/2021 added analog output on pin #15

  This code fetches data from a feed configured on io.adafruit.com. The data 
  will be one of the following:
  on, off, zap, or a number 0 - 180

  on or off will set PINOUT4 high or low respectively
  zap will set PINOUT4 high for 750ms and then go low
  0 to 180 will set SERVOPIN to that angle using PWM
  0 to 180 will set the output voltage of APINOUT to a scaled voltage between 0 and 1023
  0 to 59 will set PINOUT2 LOW and PINOUT3 HIGH
  60 to 119 will set PINOUT2 HIGH and PINOUT3 HIGH
  120 to 180 will set PINOUT2 HIGH and PINOUT3 LOW

  For controlling an external motor using a L293d motor driver chip:
  connect PINOUT2 and PINOUT3 to the motor direction pins of the chip
  connect PINOUT4 to the motor enable pin of the chip

  For the Huzzah breakout board from Adafruit:
  #define APINOUT 15 
  #define PINOUT2 12  
  #define PINOUT3 14 
  #define PINOUT4 16 
  #define SERVOPIN 13
  
   
 ****************************************************/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>  //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Servo.h>
#include <string.h>
#define APINOUT 15 // analog voltage reflects servo value scaled to 3.3V
#define PINOUT2 12 // motor direction (active high) advance/reverse 
#define PINOUT3 14 // motor direction (active low) reverse/advance
#define PINOUT4 16 // motor enable (ON/OFF)
#define SERVOPIN 13
#define CTRLC 3


/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
char aio_username[40] = "";
char aio_key[50] = "";
char aio_myfeed[40] = "controldevice";

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class as a global using dummy info to be filled in later
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, aio_username, aio_key);

String aio_feedname = String(aio_feedname + "/feeds/controldevice");
Adafruit_MQTT_Subscribe controldevice = Adafruit_MQTT_Subscribe(&mqtt, aio_feedname.c_str());

/************************* Servo Setup *************************************/

int servoPin = SERVOPIN;  // Declare the Servo pin
int servoValue = 0; // Servo value
Servo Servo1;  // Create a servo object
bool ServoOn = false;

int analogOut = 0;


/*************************** Setup for Input Serial Prompt ******************/
const byte numChars = 80;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false;

void recvWithEndMarker(bool noecho) {
  static byte ndx = 0;
  char endMarker = 13;
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (!noecho) {
      Serial.print(rc); //echo character
    }

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}



/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);      // turn Red LED off
  pinMode(PINOUT2, OUTPUT);  // motor output active high
  pinMode(PINOUT3, OUTPUT);  // motor output active low
  pinMode(PINOUT4, OUTPUT);  // motor enable (optional)
  pinMode(APINOUT, OUTPUT); // analog pin out
  analogWrite(APINOUT, 0); // output 0V to APINOUT
  digitalWrite(PINOUT3, LOW);
  digitalWrite(PINOUT2, HIGH);
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Reach and Teach Bobble Interface"));

  Serial.begin(115200);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(aio_username, json["aio_username"]);
          strcpy(aio_key, json["aio_key"]);
          strcpy(aio_myfeed, json["aio_myfeed"]);

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read


 //WiFiManager
  WiFiManagerParameter custom_aio_username("aio_username", "aio username", aio_username, 40);
  WiFiManagerParameter custom_aio_key("aio_key", "aio key", aio_key, 50);
  WiFiManagerParameter custom_aio_myfeed("aio_myfeed", "aio myfeed", aio_myfeed, 40);
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //add all your parameters here
  wifiManager.addParameter(&custom_aio_username);
  wifiManager.addParameter(&custom_aio_key);
  wifiManager.addParameter(&custom_aio_myfeed);

  //exit after config instead of connecting
  wifiManager.setBreakAfterConfig(true);

  //hold GPIO0 down if you want to reconfigure wifi
  bool reconfig=false;
  delay(2000); // wait 2 seconds before checking
  pinMode(0, INPUT); // get ready to sense GPIO0 button
  while (digitalRead(0) == LOW) { reconfig= true; delay(100);}
  if (reconfig) {
    wifiManager.resetSettings();
    reconfig= false;  
  }

  //tries to connect to last known settings
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP" with password "password"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("BobbleConnectAP", "password")) {
    Serial.println("failed to connect, we should reset and see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected...yay :)");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  //read updated parameters
  strcpy(aio_username, custom_aio_username.getValue());
  strcpy(aio_key, custom_aio_key.getValue());
  strcpy(aio_myfeed, custom_aio_myfeed.getValue());

  //save the custom parameters to FS

  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["aio_username"] = aio_username;
  json["aio_key"] = aio_key;
  json["aio_myfeed"] = aio_myfeed;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  //end save
  

  // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
  mqtt = Adafruit_MQTT_Client(&client, AIO_SERVER, AIO_SERVERPORT, aio_username, aio_key);


  // Setup a feed called for subscribing to changes.
  aio_feedname = String(aio_username) + "/feeds/" + String(aio_myfeed);
  Serial.println(aio_feedname );
  controldevice = Adafruit_MQTT_Subscribe(&mqtt, aio_feedname.c_str());

  // Setup MQTT subscription for controldevice feed.

  mqtt.subscribe(&controldevice);
  pinMode(0, OUTPUT);
  delay(1000);
  digitalWrite(0, LOW); // turn on Red LED to indicate we are connected WiFi
  Servo1.attach(servoPin); // Attach servo
  ServoOn=true;

}

uint32_t x = 0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &controldevice) {
      Serial.print(F("Got: "));
      String line = (char *)controldevice.lastread;
      Serial.println(line.substring(0, 3));

      //---------Detect whether the text value is On or Off or a number between 0 and 180
      if (line.substring(0, 2) == "on") {
        Serial.println("\n\r PINOUT4 (motor enable) ON");
        digitalWrite(0, LOW); // red led on
        digitalWrite(PINOUT4, HIGH);
      }
      if (line.substring(0, 3) == "off") {
        Serial.println("\n\r PINOUT4 (motor enable) OFF");
        digitalWrite(0, HIGH);  // red led off
        digitalWrite(PINOUT4, LOW);
      }
      if (line.substring(0, 3) == "zap") { // pulse on/off
        Serial.println("\n\r PINOUT4 (motor enable) pulse ON/OFF");
        digitalWrite(0, LOW);  // red led on
        digitalWrite(PINOUT4, HIGH);
        delay(750);
        digitalWrite(0, HIGH);  // red led off
        digitalWrite(PINOUT4, LOW);

      }
      
      if (line.substring(0, 1) != "o" && line.substring(0, 1) != "z") {
        servoValue = (line.substring(0, 3)).toInt();
        Serial.println("servoValue=" + String(servoValue));
        if (servoValue < 0) {
          servoValue = 0;
        }
        if (servoValue > 180) {
          servoValue = 180;
        }
        if (servoValue > 119) {
          Serial.println("\n\r PINOUT2 HIGH, PINOUT3 LOW, SERVOPIN=" + String(servoValue));
          digitalWrite(PINOUT2, HIGH);
          digitalWrite(PINOUT3, LOW);
        } else if (servoValue > 59 && servoValue < 120) {
          Serial.println("\n\r PINOUT2 HIGH, PINOUT3 HIGH, SERVOPIN=" + String(servoValue));
          digitalWrite(PINOUT2, HIGH);
          digitalWrite(PINOUT3, HIGH);
        } else {
          Serial.println("\n\r PINOUT2 LOW, PINOUT3 HIGH, SERVOPIN=" + String(servoValue));
          digitalWrite(PINOUT2, LOW);
          digitalWrite(PINOUT3, HIGH);
        }   
        analogOut= floor(servoValue*5.69); // scale from 0 to 180 to 0 to 1023
        Serial.println("\n\r analogOut=" + String(analogOut));
        analogWrite(APINOUT, min(analogOut,1023)); // drive voltage out on APINOUT
        if (ServoOn) Servo1.write(servoValue); // Drive servo     
      }
    }
  }


  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
    if(! mqtt.ping()) {
    mqtt.disconnect();
    }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  digitalWrite(0, HIGH); // turn off Red LED to indicate we are connected
}
