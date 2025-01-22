#include "HX711.h"
#define DOUT  16
#define CLK  33
HX711 scale(DOUT, CLK);
 
float weight; 
float calibration_factor = -3084.7; // for me this value works just perfect 419640 
 
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Tutorial: https://www.electroniclinic.com/hx711-load-cell-arduino-hx711-calibration-weighing-scale-strain-gauge/
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  delay(2000);
}
void loop() {
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  Serial.print("Reading: ");
  weight = scale.get_units(5); 
  if(weight<0) { weight=0.00;}
  //Serial.print(scale.get_units(), 2);
  // Serial.print(" lbs"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person

  // 57.5 = 50%
  Serial.print("Kilogram:");
  Serial.print( weight); 
  Serial.print(" Kg");
  Serial.print("Volume: ");
  Serial.print(((short) mapFloat(weight, 0.00, 101.00, 0.00, 100.00)) - 17); // -17 because load cell is affected by grounded power supply
  Serial.print(" - ");
  Serial.print(mapFloat(weight, 0.00, 109.00, 0.00, 100.00));
  Serial.print("% ");
  Serial.println();
  delay(500);
}

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/arduino-load-cell-hx711/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// // Calibrating the load cell
// #include "HX711.h"

// // HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = 16;
// const int LOADCELL_SCK_PIN = 33;

// HX711 scale;

// void setup() {
//   Serial.begin(57600);
//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
//   Serial.println("Scale is initializing...");
//   scale.set_scale();
//   scale.tare();
//   Serial.println("Send '+' to measure weight.");
// }

// // float calibrationFac = -8013.8; // without rooftop
// float calibrationFac = -3084.7;

// void loop() {
//   if (scale.is_ready()) {
//     Serial.println("Restarting..");
    
//     while (true) { // Wait for user input
//       if (Serial.available()) {
//         char input = Serial.read();
//         if (input == '+') {
//           scale.set_scale(calibrationFac/100);
//           scale.tare();
//           Serial.println("Tare done, place a known weight, and select '-' to scale... Then '+' to tare (empty).");
//           break; // Exit the waiting loop
//         }
//         if (input == '-') {
//           Serial.println("Calibrating...");
//           long reading = scale.get_units(20);
//           Serial.print("Result: ");
//           Serial.println(reading);
//           Serial.println("Send '+' to tare again or restart the scale.");
//           break; // Exit the waiting loop
//         }
//         if (input == 'a') {
//           calibrationFac += 50;
//           break; // Exit the waiting loop
//         }
//         if (input == 'b') {
//           calibrationFac -= 50;
//           break; // Exit the waiting loop
//         }
//       }
//       // If no valid input, do nothing
//     }
//   } else {
//     Serial.println("HX711 not found.");
//   }
//   delay(1000);
// }
