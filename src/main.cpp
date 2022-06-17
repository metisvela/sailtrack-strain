#include <Arduino.h>
#include <SailtrackModule.h>
#include <HX711.h>


// -------------------------- Configuration -------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		5

#define BATTERY_ADC_PIN 		    35
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 	  	32
#define BATTERY_READING_DELAY_MS	20

#define HX711_DOUT_PIN				25
#define HX711_SCK_PIN				27
#define LOADCELL_OFFSET 			50682624
#define LOADCELL_DIVIDER			5895655

#define LOOP_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ

// ------------------------------------------------------------------- //

SailtrackModule stm;
HX711 hx;

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
  stm.begin("strain", IPAddress(192, 168, 103, 105), new ModuleCallbacks());
  hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
//   hx.set_scale(LOADCELL_DIVIDER);
//   hx.set_offset(LOADCELL_OFFSET);
}

void loop() {
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
	doc["load"] = hx.read_average(10);
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}