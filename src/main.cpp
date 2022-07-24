#include <Arduino.h>
#include <SailtrackModule.h>
#include <HX711.h>
#include <EEPROM.h>
// -------------------------- Configuration -------------------------- //
#define EEPROM_SIZE					4096

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
//passed by grafana if different
#define DEFAULT_TIME				30000
#define DEFAULT_LOAD				0
#define g							9.80
#define LOOP_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ

//WEIGHT= (val-OFFSET)/DIVIDER
// ------------------------------------------------------------------- //

SailtrackModule stm;
//libreria per la comunicazione tra codice-amp-cella_di_carico
HX711 hx;
bool cal_status=false;

double divider,in_load=DEFAULT_LOAD;
long int offset;
int address=0, cal_time=DEFAULT_TIME;
void read_data();
void calibration(double in_load=0, int cal_time=DEFAULT_TIME);
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
	void onMqttMessage(const char * topic,JsonObject payload) {
		if(strcmp(topic,"sensor/strain0/calibration")){
			in_load=payload["load"];
			cal_time=payload["time"];
			cal_status=true;
		}
	}

};

void readMemory()
{
	EEPROM.get(address,offset);
	address+=sizeof(offset);
	EEPROM.get(address,divider);
	address=0;
}
void setup() {
	stm.begin("strain", IPAddress(192, 168, 1, 243), new ModuleCallbacks());	
	EEPROM.begin(EEPROM_SIZE);
	//stm.subscribe("sensor/strain0/calibration");
	hx.begin(HX711_DOUT_PIN,HX711_SCK_PIN);	
	//iscrizione serve solo per la ricezione di dati	
	Serial.begin(115200);	
	Serial.println("setup");
	
}

void loop() {

	calibration(3);
	
}
void read_data(){	
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
	long reading = hx.get_units(LOADCELL_NUM_READING);
	doc["raw_data"]=reading;
	readMemory();
	int load=(reading-0)/divider;
	//int load=(reading-reading)/DEFAULT_DIVIDER;
	doc["load_kg"]=load;
	doc["load_newton"]=load*g;
	doc["offset"]=offset;
	doc["divider"]=divider;	
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());	
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}
void calibration(double in_load, int cal_time){
	TickType_t lastWakeTime = xTaskGetTickCount();
	double out_load;
	long int reading;
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> cal;
	hx.set_scale();
	hx.tare(LOADCELL_NUM_READING);
	//offset is the average value now measured
	offset=hx.get_units(LOADCELL_NUM_READING);
	EEPROM.put(address,offset);
	address+=sizeof(offset);
	//initiate calibration
	digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);

	//time to attach the loadcell to the shroud
	delay(cal_time);
	digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
	//takes the average based on 10 reading
	reading = hx.get_units(LOADCELL_NUM_READING);
	
	if(in_load==0 || reading==0){
		out_load=0;
		offset=0;
		divider=0;
		Serial.println(offset);
		Serial.println(reading);
		Serial.println(divider);
		Serial.println(out_load);
	}
	else
	{
	divider=reading/in_load;
	EEPROM.put(address,divider);
	address=0;
	//log_printf("Divider: %ld/%.1f=%.3f\n", reading, load_kg, divider);
	hx.set_scale(divider);
	out_load=(reading-offset)/divider;
	Serial.println(offset);
	Serial.println(reading);
	Serial.println(divider);
	Serial.println(out_load);
	Serial.println("fine calibrazione");
	}	
	
	//data measured by the loadcell
	cal["raw_data"]=reading;
	//it gives the weight measured
	cal["load_kg"]=out_load;
	cal["load_newton"]=out_load*g;
	//calibration factor 
	cal["offset"]=offset;
	cal["divider"]=divider;	
	stm.publish("sensor/strain0/calibration", cal.as<JsonObjectConst>());
	//cal_status=true;
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}