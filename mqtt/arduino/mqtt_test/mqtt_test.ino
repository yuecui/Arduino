#include <PubSubClient.h>
#include <WiFiNINA.h>
#include "keys.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = TEST_SSID;        // your network SSID (name)
char pass[] = TEST_PASS;    // your network password (use for WPA, or use as key for WEP)
char user[] = AUM_USER;

WiFiClient wifiClient;
PubSubClient mqttClient;

const char* server  = "172.16.38.200";
int        port     = 1883;
const char topic[]  = "test/room1";
const char* clientID = "Arduino_mkr1010";
const char* mqttUser = "username";
const char* mqttPassword = "password";

//set interval for sending messages (milliseconds)
const long interval = 8000;
unsigned long previousMillis = 0;
void function()
{
  
}

int count = 0;

void callback(char* topic, byte* payload, unsigned int length)
{

}


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

/*
  while (WiFi.beginEnterprise(ssid, user, pass) != WL_CONNECTED) 
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  */
  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(server);
  
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(server, port);
  mqttClient.setCallback(callback);
  mqttClient.connect(clientID, mqttUser, mqttPassword);
  

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker


  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //record random value from A0, A1 and A2
    int Rvalue = analogRead(A0);
    int Rvalue2 = analogRead(A1);
    int Rvalue3 = analogRead(A2);

    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    //Serial.println(Rvalue);

    char outmsg[]={"Test message from arduino"};

    //Serial.println(mqttClient.state());
    boolean rc = mqttClient.publish(topic, outmsg);

  }
  mqttClient.loop();
}