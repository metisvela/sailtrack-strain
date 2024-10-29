#include <Arduino.h>
#include <HX711.h>

#define DOUT  16
#define CLK   17

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, CLK);
  Serial.println("HX711 calibration sketch");
  delay(2000);
}

void loop() {
	if (scale.is_ready()) {
		long reading = scale.get_units(10);
		Serial.println(reading);
	}
	delay(1);
}
