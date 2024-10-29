#include <Arduino.h>
#include <SPI.h>
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

#define HX711_DOUT_PIN				16
#define HX711_SCK_PIN				17
#define HX711_NUM_READING			10

#define MQTT_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ
#define LOOP_TASK_INTERVAL_MS		100

//Quadratic intrapolation like: load = A*measure^3+B*measure^2+C*measure+D
#define A                           0
#define B                           2.3008880046
#define C                           -1365.9396282640
#define D                           37538.3624881730
// ------------------------------------------------------------------- //

SailtrackModule stm;
HX711 hx;

int load = -1;

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

void mqttTask(void * pvArguments) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true) {
		StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;

		JsonObject euler = doc.createNestedObject("tension");
		euler["load"] = load;

		stm.publish("sensor/imu0", doc.as<JsonObjectConst>());

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
	}
}

long measureToLoad(long measure){
    long load = -1;
    load = A*measure*measure*measure+B*measure*measure+C*measure+D;
    return load;
}

void setup() {
	stm.begin("strain", IPAddress(192, 168, 42, 105), new ModuleCallbacks());
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    xTaskCreate(mqttTask, "mqttTask", STM_TASK_MEDIUM_STACK_SIZE, NULL, STM_TASK_MEDIUM_PRIORITY, NULL);
}

void loop() {
    TickType_t lastWakeTime = xTaskGetTickCount();
	
    if (hx.is_ready()) {
		long reading = hx.get_units(10);
		load = measureToLoad(reading);
	}

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}