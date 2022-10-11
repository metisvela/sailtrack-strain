#include <Arduino.h>
#include <SailtrackModule.h>
#include <HX711.h>
#include <EEPROM.h>
#include <stdint.h>

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
#define HX711_NUM_READING			1

#define LOOP_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ

#define EEPROM_ALLOC_SIZE_BYTES		512
#define EEPROM_CAL_SIZE_BYTES		2+sizeof(long)+sizeof(float)+2
#define EEPROM_CAL_ADDR				42
#define EEPROM_CAL_ID_0				0x30
#define EEPROM_CAL_ID_1				0x55

#define CAL_TARE_DELAY_S			10
#define CAL_NUM_READINGS			10
#define	CAL_SCALE_DELAY_S			10
#define CAL_TARE_LED_DELAY_MS		700
#define CAL_SCALE_LED_DELAY_MS		200
#define CAL_NUM_TRIES				5
#define CAL_TRIES_DELAY_MS			500
#define CAL_ERROR_LED_DELAY_MS		150		
// ------------------------------------------------------------------- //

SailtrackModule stm;
HX711 hx;

bool calibration;
int calibrationTries=CAL_NUM_TRIES;
int calibrationScaleDelay;
float calibrationLoad;
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
	void onMqttMessage(const char * topic, JsonObjectConst message) {
		calibrationLoad=message["calibrationLoad"];
		calibrationScaleDelay=message["calibrationScaleDelay"]?message["calibrationScaleDelay"]:CAL_SCALE_DELAY_S;
		calibration=true;
	}

};
//to read
uint16_t crc16Update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else crc = (crc >> 1);
    }
    return crc;
}
//to read
bool loadCalibration() {
	uint8_t buf[EEPROM_CAL_SIZE_BYTES];

	uint16_t crc = 0xFFFF;
	for (uint16_t a = 0; a < EEPROM_CAL_SIZE_BYTES; a++) {
		buf[a] = EEPROM.read(a + EEPROM_CAL_ADDR);
		crc = crc16Update(crc, buf[a]);
	}

	if (crc != 0 || buf[0] != EEPROM_CAL_ID_0 || buf[1] != EEPROM_CAL_ID_1) 
		return false;	


	long offset;
	//divider=scale
	float scale;
	memcpy(&offset, buf + 2, sizeof(offset));
	memcpy(&scale,buf+2+sizeof(offset),sizeof(scale));
	hx.set_offset(offset);
	hx.set_scale(scale);

	return true;
}


bool saveCalibration() {

	uint8_t buf[EEPROM_CAL_SIZE_BYTES];
	memset(buf, 0, EEPROM_CAL_SIZE_BYTES);
	buf[0] = EEPROM_CAL_ID_0;
	buf[1] = EEPROM_CAL_ID_1;

	long offset=hx.get_offset();
	float scale=hx.get_scale();
	memcpy(buf+2,&offset,sizeof(offset));
	memcpy(buf+2+sizeof(offset),&scale,sizeof(scale));

	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < EEPROM_CAL_SIZE_BYTES - 2; i++) 
		crc = crc16Update(crc, buf[i]);
	buf[EEPROM_CAL_SIZE_BYTES - 2] = crc & 0xFF;
	buf[EEPROM_CAL_SIZE_BYTES - 1] = crc >> 8;

	for (uint16_t a = 0; a < EEPROM_CAL_SIZE_BYTES; a++) 
		EEPROM.write(a + EEPROM_CAL_ADDR, buf[a]);

	EEPROM.commit();
	
	return true;
}


bool calibrate(float calLoad, int calScaleDelay){
	//if zero return false
	if(!calLoad) return false;
	//setting tare
	for(int i=0;i<(1000*CAL_TARE_DELAY_S)/(2*CAL_TARE_LED_DELAY_MS);i++){		
		digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);
		delay(CAL_TARE_LED_DELAY_MS);
		digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
		delay(CAL_TARE_LED_DELAY_MS);
	}
	hx.tare(CAL_NUM_READINGS);
	//setting scale	
	for(int i=0;i<(1000*calScaleDelay)/(2*CAL_SCALE_LED_DELAY_MS);i++){		
		digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);
		delay(CAL_SCALE_LED_DELAY_MS);
		digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
		delay(CAL_SCALE_LED_DELAY_MS);
	}
	double reading=hx.get_value(CAL_NUM_READINGS);
	//set new tare
	hx.set_scale(reading/calLoad);
	return saveCalibration();
}
void read_data(){	
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;	
	doc["raw"]=hx.read_average(HX711_NUM_READING);
	doc["load"]=hx.get_units(HX711_NUM_READING);
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());	
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}

void setup() {
	stm.begin("strain", IPAddress(192, 168, 42, 105), new ModuleCallbacks());	
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	EEPROM.begin(EEPROM_ALLOC_SIZE_BYTES);
	loadCalibration();
	stm.subscribe("sensor/strain0/calibration");
}

void loop() {
	
	if(calibration){
		if(calibrate(calibrationLoad,calibrationScaleDelay)){
			calibration=false;
			calibrationTries=CAL_NUM_TRIES;
		}
		else
			{
				for(int i=0;i<3;i++){
					digitalWrite(STM_NOTIFICATION_LED_PIN, LOW);
					delay(CAL_ERROR_LED_DELAY_MS);
					digitalWrite(STM_NOTIFICATION_LED_PIN, HIGH);
					delay(CAL_ERROR_LED_DELAY_MS);
				}
				if(calibrationTries--){
					calibration=false;
					calibrationTries=CAL_NUM_TRIES;
				}									
				delay(CAL_TRIES_DELAY_MS);
			}
	}
	else
		read_data();
	
}
