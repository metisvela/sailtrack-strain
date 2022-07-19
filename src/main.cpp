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
#define LOADCELL_NUM_READING		10
//passed by grafana if diffent
#define DEFAULT_TIME				30000
#define g							9.80
#define LOOP_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ

//WEIGHT= (val-OFFSET)/DIVIDER
// ------------------------------------------------------------------- //

SailtrackModule stm;
//libreria per la comunicazione tra codice-amp-cella_di_carico
HX711 hx;
bool cal_status=false;
int cal_time;
int load_kg;
void read_data();
void calibration(int load_kg=0, int cal_time=DEFAULT_TIME);
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
	//parso il json per prendere peso e tempo per la calibrazione
	/*void onMqttMessage(const char * topic,JsonObject payload) {
		if(strcmp(topic,"sensor/strain0/calibration")){
			stm.subscribe("sensor/strain0/calibration");
			StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
			deserializeJson(doc,payload);
			//come sono i mex mqtt??
			load_kg=doc["load"];
			cal_time=doc["time"];
			cal_status=true;
		}
	}*/

};


void setup() {
	stm.begin("strain", IPAddress(192, 168, 1, 10), new ModuleCallbacks());
}

void loop() {
	if(cal_status)
		calibration(36.1,60);
	else
		//altrimenti leggo i dati dall'amp e li spedisco nel relativo topic
		read_data();

}
void read_data(){	
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
	long reading = hx.get_units(LOADCELL_NUM_READING);
	doc["raw_data"]=reading;
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());	
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}
void calibration(int load_kg, int cal_time){
	TickType_t lastWakeTime = xTaskGetTickCount();
	int divider,load;
	long reading, offset;	
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> cal;
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	hx.set_scale();
	hx.tare();
	//offset is the average value now measured
	offset=hx.get_units(LOADCELL_NUM_READING);
	//initiate calibration
	digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);

	//time to attach the loadcell to the shroud
	delay(cal_time);
	digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
	//takes the average based on 10 reading
	reading = hx.get_units(LOADCELL_NUM_READING);
	if(load_kg==0){
		load=0;
		offset=0;
		divider=0;
	}
	else
	{
	divider=reading/load_kg;
	log_printf("Divider: %ld/%.1f=%.3f\n", reading, load_kg, divider);
	hx.set_scale(divider);
	load=reading-offset/divider;
	}	
	
	//data measured by the loadcell
	cal["raw_data"]=reading;
	//it gives the weight measured
	cal["load_kg"]=load;
	cal["load_newton"]=load*g;
	//calibration factor 
	cal["divider"]=divider;
	cal["offset"]=offset;
	stm.publish("sensor/strain0/calibration", cal.as<JsonObjectConst>());
	cal_status=false;	
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}