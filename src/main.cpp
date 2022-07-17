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
//taken from grafana, so no more
//#define LOADCELL_BASE_LOAD_KG		36.1 
#define LOADCELL_NUM_READING		32
//divider:1241.496

#define LOOP_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ

// ------------------------------------------------------------------- //

SailtrackModule stm;
//libreria per la comunicazione tra codice-amp-cella_di_carico
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
	
}

void loop() {
	calibration(36.1);
	
}

void calibration(int load_kg){
	int calibration_factor,load;
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	hx.set_scale();
	//set offset a tare weight
	hx.tare();
	//initiate calibration
	digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);
	//30 seconds to attach the load cell shroud
	delay(30000);
	digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
	//takes the average based on 10 reading
	long reading = hx.get_units(10);
	if(load_kg==0)
		load=0;
	else
	{
	calibration_factor=reading/load_kg;
	log_printf("Calibration factor: %ld/%.1f=%.3f\n", reading, load_kg, calibration_factor);
	hx.set_scale(calibration_factor);
	load=reading/calibration_factor;
	////////////////????????////////////////
	//doc["load"] = val;
	}
	//it gives the weight measured
	doc["load"]=load;
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}