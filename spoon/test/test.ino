#include <Servo.h>


Servo myservo1;  // create servo object to control a servo
Servo myservo2;
//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin

void setup() {
  myservo1.attach(2);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(3);
  Serial.begin(9600);
}

void loop() {
//  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
//  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  int val = random(0, 180);
  Serial.print("Writing I : ");Serial.println(val);
  myservo1.write(val);                  // sets the servo position according to the scaled value
  val = random(0, 180);
  Serial.print("Writing II : ");Serial.println(val);
  myservo2.write(val);                  // sets the servo position according to the scaled value

  delay(15);                           // waits for the servo to get there
}