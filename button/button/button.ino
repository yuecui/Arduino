#include <WiFiNINA.h>
#include "keys.h"
#include <SPI.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
char ssid[] = HOME_SSID; 
char pass[] = HOME_PASS;  

int button_state = 0;  // variable for reading the pushbutton status
const int BUTTON_PIN = 6; 
volatile bool button_flag = false;


int Bot_mtbs = 1000; //mean time between scan messages
long Bot_lasttime;   //last time messages' scan has been done
WiFiSSLClient client;
UniversalTelegramBot bot(BOTtoken, client);


int botRequestDelay = 1000;
unsigned long lastTimeBotRan;
void getButtonState()
{ 
    Serial.println("Button Pressed");
    button_state = digitalRead(BUTTON_PIN);
    if (button_state == HIGH)
    {
        button_flag = true;
        Serial.println(button_flag);
    }
    return;
}
void sendMessage()
{
   
    String message = "Button pressed!";
   
    bool result = bot.sendMessage(CHAT_ID, message, "");
   
    if(result)
    {
        Serial.println("TELEGRAM Successfully sent");
    }
    button_flag = false;
}
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, getButtonState, RISING);
 
}

// the loop function runs over and over again forever
void loop() {
  if ( button_flag ) {
  
    sendMessage();
   
  }
}