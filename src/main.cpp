#include <Arduino.h>
#include <SailtrackModule.h>


// -------------------------- Configuration -------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		  5

#define BATTERY_ADC_PIN 		    	35
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 	  	32
#define BATTERY_READING_DELAY_MS	20

#define LOOP_TASK_DELAY_MS		  	1000 / MQTT_PUBLISH_FREQ_HZ

// ------------------------------------------------------------------- //

SailtrackModule stm;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onStatusPublish(JsonObject status) {
		JsonObject battery = status.createNestedObject("battery");
		float avg = 0;
		for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
			avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
			delay(BATTERY_READING_DELAY_MS);
		}
		battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
	}
};


void setup() {
  stm.begin("strain", IPAddress(192, 168, 42, 105), new ModuleCallbacks());
}

void loop() {
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
  //TODO create json file
  stm.publish("sensor/strain0", doc.as<JsonObjectConst>());
}
