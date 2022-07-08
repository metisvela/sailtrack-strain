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
// TODO: Adjust value
#define LOADCELL_BASE_LOAD_KG		36.1 
#define LOADCELL_NUM_READING		32
1241.496

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
	stm.begin("strain", IPAddress(192, 168, 42, 105), new ModuleCallbacks());
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	hx.set_scale();
	hx.tare();
	digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);
	delay(30000);
	digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
	long reading = hx.get_units(10);
	log_printf("Calibration factor: %ld/%.1f=%.3f\n", reading, LOADCELL_BASE_LOAD_KG, reading/LOADCELL_BASE_LOAD_KG);
	hx.set_scale(reading/LOADCELL_BASE_LOAD_KG);
}

void loop() {
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
	long val = hx.get_units();
	doc["load"] = val;
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}
