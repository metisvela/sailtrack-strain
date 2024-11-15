#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <HX711.h>

// -------------------------- Configuration -------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		5

#define BATTERY_ADC_PIN 		    36
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 	  	32
#define BATTERY_READING_DELAY_MS	20
#define BATTERY_MIN_VOLTAGE			3.0
#define BATTERY_MAX_VOLTAGE			4.2
#define MIN_USB_VOLTAGE				4.2

#define HX711_DOUT_PIN				16
#define HX711_SCK_PIN				17
#define HX711_NUM_READING			10

#define MQTT_TASK_INTERVAL_MS		1000 / MQTT_PUBLISH_FREQ_HZ
#define LOOP_TASK_INTERVAL_MS		100

//Quadratic intrapolation like: load = A*measure^3+B*measure^2+C*measure+D
#define A                           0
#define B                           0
#define C                           0
#define D                           0
// ------------------------------------------------------------------- //

SailtrackModule stm;
HX711 hx;

int load = -1;
long reading = -1;
float readBatteryVoltage();
int readBatteryPercentage();


class ModuleCallbacks: public SailtrackModuleCallbacks {

	void onStatusPublish(JsonObject status) {
		JsonObject battery = status.createNestedObject("battery");
		battery["voltage"] = "Not Working";//readBatteryVoltage();
		battery["percentage"] = "Not Working";//readBatteryPercentage();
	}

	uint32_t notificationLed(){
	//	int batteryPerc = readBatteryPercentage();
	// 	if(readBatteryVoltage() >= MIN_USB_VOLTAGE){
	// 		return 0x0000FF00;
	// 	}

	// 	if(batteryPerc<=20){
	// 		return 0x00FF0000;
	// 	}

	// 	if (batteryPerc>20 && batteryPerc<90){
	// 		return 0x000000FF;
	// 	}

	// 	if (batteryPerc>=90){
	// 		return 0x00FF00FF;
	// 	}

	 	return 0x00000000;
	}
};

void mqttTask(void * pvArguments) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true) {
		StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;

		JsonObject euler = doc.createNestedObject("tension");
		euler["load"] = load;
		euler["raw"] = reading;
		stm.publish("sensor/strain2", doc.as<JsonObjectConst>());
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
	}
}

long measureToLoad(long measure){
    long load = -999999;
    load = A*measure*measure*measure+B*measure*measure+C*measure+D;
    return load;
}

void setup() {
	Serial.begin(115200);
	stm.begin("strain0", IPAddress(192, 168, 42, 105), new ModuleCallbacks());
	hx.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    xTaskCreate(mqttTask, "mqttTask", STM_TASK_MEDIUM_STACK_SIZE, NULL, STM_TASK_MEDIUM_PRIORITY, NULL);
	Serial.begin(115200);
}

void loop() {
    TickType_t lastWakeTime = xTaskGetTickCount();
	
    if (hx.is_ready()) {
		reading = hx.get_units(10);
		load = measureToLoad(reading);
		}

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}


float readBatteryVoltage(){
	float avg = 0;
	for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
		avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
		delay(BATTERY_READING_DELAY_MS);
	}
	return 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
}

int readBatteryPercentage() {
    float voltage = readBatteryVoltage();
    if (voltage < BATTERY_MIN_VOLTAGE) {
        return 0;
    } else if (voltage > BATTERY_MAX_VOLTAGE) {
        return 100;
    }
    return (int)((voltage - BATTERY_MIN_VOLTAGE) * 100 /  (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE));
}