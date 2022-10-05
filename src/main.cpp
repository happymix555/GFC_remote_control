#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DAC_PIN A0 // Make code a bit more legible
#define POT_read_pin A3 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int button_pin = 1;     // the number of the pushbutton pin
const int POT_vcc_pin = 2;
bool current_state = LOW; // 0 mean sending command to GFC, 1 mean read POT for new setpoint

unsigned long last_pressed_time = 0; //variable to store time from millis() used for switch debouncing.
unsigned int print_time = 200;
unsigned int dis_update_time = 20;
int setpoint = 0;
int POT_value;

void change_state(){ 
  static unsigned int debouncing_time = 200; //set debouncing for 200 ms

  if (millis() - last_pressed_time >= debouncing_time){
    SerialUSB.println("interrupting...");
    setpoint = POT_value;
    current_state =! current_state;
    last_pressed_time = millis();
  }
}

void setup() 
{

  pinMode(button_pin, INPUT_PULLDOWN);
  pinMode(POT_vcc_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(button_pin), change_state, RISING);

  analogWriteResolution(10); // Set analog out resolution to max, 10-bits
  analogReadResolution(10); // Set analog input resolution to max, 12-bits

  SerialUSB.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display.begin(SSD1306_EXTERNALVCC, 0x3C);
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
}

void loop() 
{
  digitalWrite(POT_vcc_pin, HIGH);

  static unsigned int last_print_time;
  static unsigned int last_dis_update_time;
  
  if (millis() - last_print_time >= print_time) {
    last_print_time = millis();
    Serial.print("Millis is : ");
    Serial.println(millis());
    if (!current_state){
      SerialUSB.println("writing...");
      SerialUSB.print("Setpoint is: ");
      SerialUSB.println(setpoint);
      SerialUSB.print("POT value is: ");
      SerialUSB.println(POT_value);
      // SerialUSB.println(POT_value);
    } else {
      SerialUSB.println("reading...");
      SerialUSB.print("Setpoint is: ");
      SerialUSB.println(setpoint);
      SerialUSB.print("POT value is: ");
      SerialUSB.println(POT_value);
      // SerialUSB.println(POT_value);
    }
    // SerialUSB.println("current state is %d", current_state); // Print the current mode of operation.
  }
  POT_value = analogRead(POT_read_pin);
  analogWrite(DAC_PIN, setpoint);
  // if (current_state == LOW){
  //   //send command to GFC
  //   // digitalWrite(POT_vcc_pin, LOW);
  //   // analogWrite(DAC_PIN, POT_value);
  // }
  // else if (current_state == HIGH){
  //   //read value from POT to change setpoint
  //   // analogWrite(DAC_PIN, 0);
  //   // float voltage = 3.3 * POT_value / 4095; //convert analog read to voltage level
  // }

  if (millis() - last_dis_update_time >= dis_update_time){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(100,0);
    display.print("L Meter:");
    display.display();
    last_dis_update_time = millis();
  }

}

// void change_state(){ 
//   static int debouncing_time = 200; //set debouncing for 200 ms

//   SerialUSB.println("interrupting...");
//   if (millis() - last_pressed_time >= debouncing_time){
//     current_state =! current_state;
//     last_pressed_time = millis();
//   }

// }


// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// #define DAC_PIN A0 // Make code a bit more legible
// #define POT_read_pin A3 

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels

// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// const int button_pin = 1;     // the number of the pushbutton pin
// const int POT_vcc_pin = 2;
// bool current_state = LOW; // 0 mean sending command to GFC, 1 mean read POT for new setpoint

// unsigned long last_pressed_time = 0; //variable to store time from millis() used for switch debouncing.
// static unsigned int print_time = 500;
// int setpoint = 0;
// int POT_value;

// void change_state(){ 
//   static unsigned int debouncing_time = 200; //set debouncing for 200 ms

//   if (millis() - last_pressed_time >= debouncing_time){
//     SerialUSB.println("interrupting...");
//     setpoint = POT_value;
//     current_state =! current_state;
//     last_pressed_time = millis();
//   }
// }

// void setup() 
// {

//   Wire.begin();
//   Serial.begin(9600);
//   Serial.println("\nI2C Scanner");
// }

// void loop() 
// {
//     byte error, address;
//     int nDevices;

//     Serial.println("Scanning...");

//     nDevices = 0;
//     for(address = 0; address <= 127; address++ )
//     {
//         Wire.beginTransmission(address);
//         error = Wire.endTransmission();
//         if (error == 0)
//         {
//             Serial.print("I2C device found at address 0x");
//             if (address<16)
//                 Serial.print("0");
//             Serial.print(address, HEX);
//             Serial.println(" !");
//             nDevices++;
//         }
//         else if (error==4)
//         {
//             Serial.print("Unknow error at address 0x");
//             if (address<16)
//                 Serial.print("0");
//             Serial.println(address,HEX);
//         }
//     }
//     if (nDevices == 0)
//         Serial.println("No I2C devices found\n");
//     else
//         Serial.println("done\n");
//     delay(30000);

// }

// // void change_state(){ 
// //   static int debouncing_time = 200; //set debouncing for 200 ms

// //   SerialUSB.println("interrupting...");
// //   if (millis() - last_pressed_time >= debouncing_time){
// //     current_state =! current_state;
// //     last_pressed_time = millis();
// //   }

// // }