#include "HX711.h"

const int LOADCELL_DOUT_PIN = 34;
const int LOADCELL_SCK_PIN = 23;

HX711 scale;
bool calibrated = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println("\n=== HX711 Calibration Utility ===");
  Serial.println("Ensure nothing is resting on the load cell.");
  Serial.println("Taring in 3 seconds...");
  delay(3000);

  scale.set_scale(); // Default 1:1 scale
  scale.tare(20);    // Zero out the scale using 20 averaged samples
  Serial.println("Tare complete. Zero reference established.");

  Serial.println("\nPlace your KNOWN weight on the cell.");
  Serial.println("Type the weight in grams into the input bar and press Enter:");
}

void loop() {
  if (!calibrated && Serial.available() > 0) {
    float known_weight = Serial.parseFloat();

    // Clear leftover line ending characters
    while (Serial.available()) Serial.read();

    if (known_weight > 0) {
      Serial.print("Sampling raw counts for ");
      Serial.print(known_weight, 2);
      Serial.println(" g...");

      long raw_offset = scale.get_value(20);
      float calibration_factor = (float)raw_offset / known_weight;

      scale.set_scale(calibration_factor);
      calibrated = true;

      Serial.println("\n-------------------------------------------");
      Serial.print("CALIBRATION FACTOR: ");
      Serial.println(calibration_factor, 2);
      Serial.println("Save this value for your final logger sketch!");
      Serial.println("-------------------------------------------\n");
      Serial.println("Now streaming live weight:");
      delay(1500);
    }
  }

  if (calibrated) {
    Serial.print("Live Weight: ");
    Serial.print(scale.get_units(5), 1);
    Serial.println(" g");
    delay(500);
  }
}