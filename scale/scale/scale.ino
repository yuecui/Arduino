

#include "HX711.h"
#include "LiquidCrystal_I2C.h"
#include "Wire.h"
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

LiquidCrystal_I2C lcd(0x27, 16, 2);
HX711 scale;

void print_message(int col, int row, String message)
{
    lcd.setCursor(col, row);
    lcd.print(message);
}
int take_user_input(){
  Serial.print("Enter the known weight: ");
  while(!Serial.available()){}
  int weight = Serial.parseInt();
  Serial.println(weight);
  return weight;

}
int get_califactor()
{
  long reading;
  int known_weight;
  if(scale.is_ready())
  {
    print_message(0, 0, "Tare...");
    print_message(0, 1, "Remove Weights");
    delay(5000);
    scale.tare();
    lcd.clear();

    print_message(0, 0, "Tare Done!");
    delay(2000);
    lcd.clear();

    print_message(0, 0, "Place a Known");
    print_message(0, 1, "Weight");
    delay(5000);
    lcd.clear();

    known_weight= take_user_input();
    reading = scale.get_units(10);
    print_message(0, 0, "Preparing...");
    
  }else{
    print_message(0, 0, "HX711 not found.");
    print_message(0, 1, "Resetting");
    reading = -1;
  }
  int cali_factor;
  if (reading != -1) {
    cali_factor = (reading) / known_weight;
  }else{
    cali_factor = -1;
  }
  delay(2000);
  lcd.clear();
  return cali_factor;
}
void scale_setup(int cali_factor){
Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)
            
  scale.set_scale(cali_factor);
  scale.tare();                     // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
}
void setup() {
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  lcd.init();
  lcd.backlight();
  int cali_factor = get_califactor();
  scale_setup(cali_factor);
}

void loop() {
  print_message(0, 0, "one reading:");
  print_message(0, 1, String(scale.get_units(), 1));
  delay(3000);
  lcd.clear();
  print_message(0, 0, "average: ");
  print_message(0, 1, String(scale.get_units(10), 5));
  //Serial.print(scale.get_units(), 1);
  //Serial.print("\t| average:\t");
  //Serial.println(scale.get_units(10), 5);
/Users/yue/Developer/self-stabilized-spoon/code/code.ino
  delay(5000);
 
}

//calibration factor will be the (reading)/(known weight)