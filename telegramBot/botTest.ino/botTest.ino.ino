
#include <WiFiNINA.h>
#include "keys.h"
#include <SPI.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
// Initialize Wifi connection to the router
char ssid[] = HOME_SSID; 
char pass[] = HOME_PASS;     

const int ledPin = 6;
const int motionSensor = 7;
bool ledState = LOW;
bool motionDetected = false;

#define BOTtoken "6770348494:AAF-Mz1kMCOwe_XpR1AK5aA5Ldr26GwD3GI"
#define CHAT_ID "6350339401"
WiFiSSLClient client;
UniversalTelegramBot bot(BOTtoken, client);

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;


void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
   
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "Use the following commands to control your outputs.\n\n";
      welcome += "/led_on to turn light ON \n";
      welcome += "/led_off to turn ligh OFF \n";
      welcome += "/state to request current GPIO state \n";
      bot.sendMessage(chat_id, welcome, "");
    }

    if (text == "/led_on") {
      bot.sendMessage(chat_id, "LED state set to ON", "");
      ledState = HIGH;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/led_off") {
      bot.sendMessage(chat_id, "LED state set to OFF", "");
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/state") {
      if (digitalRead(ledPin)){
        bot.sendMessage(chat_id, "LED is ON", "");
      }
      else{
        bot.sendMessage(chat_id, "LED is OFF", "");
      }
    }
  }
}
void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  delay(3000);


  // attempt to connect to Wifi network:
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  pinMode(motionSensor, INPUT);
  }

void loop() {
  
 
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
    // int val = digitalRead(motionSensor);
    
    // if (val == HIGH)
    // {
    //   if (motionDetected == false) {
    //     bot.sendMessage(CHAT_ID, "Motion detected!!", "");
    //     Serial.println("Motion Detected");
    //     motionDetected = true;
    //   }
    // }else{
    //   if(motionDetected == true){
    //     Serial.println("Motion Ended");
    //     bot.sendMessage(CHAT_ID, "All-Cleared", "");
    //     motionDetected = false;
    //   }
    // }
  
  }
}