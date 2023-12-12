#include <Arduino.h>
#include <SailtrackModule.h>
#include <HX711.h>
#include <EEPROM.h>

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
#define CAL_ERROR_BLINKING_TIMES 	3
// ------------------------------------------------------------------- //

SailtrackModule stm;
HX711 hx;
/*
	offset->tare
	scale->divider
*/
bool startReading;
int calibrationTries=CAL_NUM_TRIES;
int calibrationScaleDelay;
float load;
/**
 * @brief This class contains functions that will be called asynchronously by the SailTrack Module library.
 * 
 */
class ModuleCallbacks: public SailtrackModuleCallbacks {
	/**
	 * @brief This function appends battery status to the other hardware informations provided by the Json 
	 * status.
	 * @details This function is called asynchronously by the SailTrack Module library when it necessary
	 * to know battery information.
	 * Battery information is calculated over BATTERY_NUM_READINGS readings to have a more robust value.
	 * 
	 * @param status is the Json object the stores hardware informations.
	 */
	void onStatusPublish(JsonObject status) {
		JsonObject battery = status.createNestedObject("battery");
		float avg = 0;
		for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
			avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
			delay(BATTERY_READING_DELAY_MS);
		}
		battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
	}
	/**
	 * @brief This function initiate the calibration procedure by setting load and time to attach 
	 * the load cell to the shroud.
	 * 
	 * @details When the MQTT message has been sent, it calls asynchronously this function that starts the
	 * calibration procedure.
	 * By using MQTT protocol MQTT message is simply a Json file.
	 * The sender of the message can provide two values:
	 * load of the shroud->compulsory
	 * time to attach the load cell to the shroud->not compulsory
	 * If time is not provided, it will be used default value.
	 * 
	 * @param topic is the "scope" where the message has been published.
	 * 
	 * @param message is what has been sent via MQTT protocol
	 */
	void onMqttMessage(const char * topic, JsonObjectConst message) {
		load=message["load"];
		startReading=true;
	}

};

/**
 * @brief This routine carries out the saving operation of the calibration.
 * 
 * @details this function stores offset and divider values into the EEPROM memory for future uses.
 * It uses crc to check wheter the memory had been written before.
 * After adding offset and divider values into the memory, the algoritm appends the checksum of those values
 * to check while getting values from memory, if they had been modified or not.
 * 
 * @return true if the saving calibration operation works properly
 * 
 * @return false otherwise
 */

/**
 * @brief The readData is responsible for publishing the raw and load data of
 * the load cell.
 * 
 * @details This function creates a new Json object, useful to store the raw reading from
 * the load cell and the actual load calculated by the load cell.
 * The raw reading is the average of HX711_NUM_READING values.
 * At the end the Json object is published in the sensor/strain0 topic. 
 */
void readData(){	
	TickType_t lastWakeTime = xTaskGetTickCount();
	StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;	
	double reading=hx.read_average(HX711_NUM_READING);
	doc["raw"]=reading;
	doc["load"]=load;
	stm.publish("sensor/strain0", doc.as<JsonObjectConst>());	
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}
/**
 * @brief Setup function is used to initiliaze the system.
 * 
 * @details The following procedures are executed:
 * The system amplifier is initiated with the hx.begin() function.
 * EEPROM size used to store the data is set with EEPROM.begin().
 * The system loads the previous calibration stored in the EEPROM memory.
 * It subscribes to the sensor/strain0/calibration topic,
 *  to be able to receive MQTT messages from that topic.
 */
void setup() {
	stm.begin("strain", IPAddress(192, 168, 42, 105), new ModuleCallbacks());
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	stm.subscribe("sensor/strain0/calibration");
}
/**
 * @brief Loop performs the calibration process or the reading data process.
 * 
 * @details It can either perform the calibration function or the reading data procedure.
 * If the calibrate operation return false for some error, it will be performed a blinking to
 * signal it.
 * Once the calibration ends correctly, the reading data operation will start.
 */
void loop() {
	if (startReading){
		readData();		
	}	
}
