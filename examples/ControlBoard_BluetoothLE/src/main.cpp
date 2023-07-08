#include <Arduino.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_err.h"

#define IDLE -1
#define TRIGGER_TRIAC_INTERVAL 20
#define MAX_BRIGHTNESS 800
#define MIN_BRIGHTNESS 7000

#define ZC_DETECT  		GPIO_NUM_26
#define TRIAC_TRIGGER 	GPIO_NUM_27

#define RELAY1 GPIO_NUM_19
#define RELAY2 GPIO_NUM_21
#define RELAY3 GPIO_NUM_22
#define RELAY4 GPIO_NUM_23

#define NTC1 ADC1_CHANNEL_0
#define NTC2 ADC1_CHANNEL_3

const char *GenericAttribute_UUID = "c84b77b2-914e-43ad-b4d7-d085cc622312";
const char *WriteCharacteristicUUID = "3b9858ab-02e0-4d92-8f05-16917771b8ab";
const char *ReadCharacteristicUUID = "0333a746-1bad-11ee-be56-0242ac120002";

static BLECharacteristic *writeCharacs;
static BLECharacteristic *readCharacs;

static const char *TAG = "ControlBoard_Main";

static const float R1 = 10000.0; // voltage divider resistor value
static const float Beta = 3435;  // Beta value
static const float To = 298.15;  // Temperature in Kelvin for 25 degree Celsius
static const float Ro = 10000.0; // Resistance of Thermistor at 25 degree Celsius
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool isPinHighEnabled = false;
static volatile long currentBrightness = MIN_BRIGHTNESS;
static void IRAM_ATTR setTimerPinLow(void);
static void IRAM_ATTR setTimerPinHigh(int brightness);
volatile bool isConnected;


static void IRAM_ATTR ZEROCross_ISREvent(void* arguments){
	if(currentBrightness == IDLE){
		return;
	}
	portENTER_CRITICAL_ISR(&mux);
	if(!isPinHighEnabled){
		setTimerPinHigh(currentBrightness);
	}
	portEXIT_CRITICAL_ISR(&mux);
}

static bool IRAM_ATTR ISR_turnPinHigh(void* arguments){
	portENTER_CRITICAL_ISR(&mux);
	GPIO.out_w1ts = (1 << TRIAC_TRIGGER);
	setTimerPinLow();
	portEXIT_CRITICAL_ISR(&mux);

	return true;
}

static bool IRAM_ATTR ISR_turnPinLow(void* arguments){
	portENTER_CRITICAL_ISR(&mux);
	GPIO.out_w1tc = (1 << TRIAC_TRIGGER);
	isPinHighEnabled = false;
	portEXIT_CRITICAL_ISR(&mux);

	return true;
}
static void IRAM_ATTR setTimerPinLow(void){
	timer_config_t timerConfig ;

	timerConfig.alarm_en = TIMER_ALARM_EN;
	timerConfig.auto_reload = TIMER_AUTORELOAD_DIS;
	timerConfig.counter_dir = TIMER_COUNT_UP;
	timerConfig.divider = 80;
	timerConfig.counter_en = TIMER_PAUSE;

	timer_init(TIMER_GROUP_1, TIMER_0, &timerConfig);
	timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TRIGGER_TRIAC_INTERVAL);
	timer_enable_intr(TIMER_GROUP_1, TIMER_0);
	timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, ISR_turnPinLow, NULL, 0);
	timer_start(TIMER_GROUP_1, TIMER_0);
}

static void IRAM_ATTR setTimerPinHigh(int brightness){
	timer_config_t timerConfig;

	timerConfig.alarm_en = TIMER_ALARM_EN;
	timerConfig.auto_reload = TIMER_AUTORELOAD_DIS;
	timerConfig.counter_dir = TIMER_COUNT_UP;
	timerConfig.divider = 80;
	timerConfig.counter_en = TIMER_PAUSE;

	timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, brightness);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ISR_turnPinHigh, NULL, 0);
	timer_start(TIMER_GROUP_0, TIMER_0);
}

IRAM_ATTR void ControlBoard_SetDimmerPot(int potency){
	int newBrightness = 7000 - (int)(6200*((float)potency/100.0f));
	portENTER_CRITICAL(&mux);
	currentBrightness = newBrightness;
	portEXIT_CRITICAL(&mux);
}

class BLECharacteristicCallBack : public BLECharacteristicCallbacks
{
private:
  char command[24];
  int value;
  void onRead(BLECharacteristic *pCharacteristic)
  {
    Serial.printf("on read\r\n");
  }
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    Serial.printf("on write\r\n");
    const char *receive = pCharacteristic->getValue().c_str();
    memset(command, 0, sizeof(command));
    sscanf(receive, "%s %i", command, &value);
    Serial.printf("command: %s value: %i\r\n", command, value);

    if (strcmp(command, "Relay1") == 0)
    {
      digitalWrite(RELAY1, value);
    }
    else if (strcmp(command, "Relay2") == 0)
    {
      digitalWrite(RELAY2, value);
    }
    else if (strcmp(command, "Relay3") == 0)
    {
      digitalWrite(RELAY3, value);
    }
    else if (strcmp(command, "Relay4") == 0)
    {
      digitalWrite(RELAY4, value);
    }
    else if (strcmp(command, "Potency") == 0)
    {
      Serial.print("setting potency");
      ControlBoard_SetDimmerPot(value);
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    isConnected = true;
    Serial.printf("connected\r\n");
  };
  void onDisconnect(BLEServer *pServer)
  {
    Serial.printf("disconnected\r\n");
    esp_restart();
  }
};

void ControlBoard_InitAdc()
{
  adcAttachPin(GPIO_NUM_36);
  adcAttachPin(GPIO_NUM_39);

  analogSetPinAttenuation(GPIO_NUM_36, ADC_11db);
  analogSetPinAttenuation(GPIO_NUM_36, ADC_11db);
}

void ControlBoard_InitGpio()
{
  gpio_config_t gpioConfig = {0};
  esp_err_t err = ESP_OK;

  gpioConfig.intr_type = GPIO_INTR_DISABLE;
  gpioConfig.mode = GPIO_MODE_OUTPUT;
  gpioConfig.pin_bit_mask = ((uint64_t)1 << RELAY1) | ((uint64_t)1 << RELAY2) | ((uint64_t)1 << RELAY3) | ((uint64_t)1 << RELAY4) | ((uint64_t)1 << TRIAC_TRIGGER);
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;

  err = gpio_config(&gpioConfig);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "error in gpio output configuration");
  }
  gpioConfig.intr_type = GPIO_INTR_POSEDGE;
  gpioConfig.mode = GPIO_MODE_INPUT;
  gpioConfig.pin_bit_mask = ((uint64_t)1 << ZC_DETECT);
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;

  err = gpio_config(&gpioConfig);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "error in gpio input configuration");
  }
  gpio_install_isr_service(0);

  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(ZC_DETECT, ZEROCross_ISREvent, NULL);

}

float ControlBoard_GetTemperature(adc1_channel_t channel)
{
  float vOut = 0;
  float Rtherm = 0;
  const float VSupply = 3.3f;
  float tempKelvin = 0;
  float tempCelsius = 0;

  vOut = (float)(VSupply * adc1_get_raw(channel)) / 4095.0f;

  Rtherm = R1 * (VSupply / vOut - 1);

  tempKelvin = 1 / (1 / To + log(Rtherm / Ro) / Beta); // Temperature in Kelvin
  tempCelsius = tempKelvin - 273.15;                   // Celsius
  return tempCelsius;
}
void setup()
{
  Serial.begin(115200);
  currentBrightness = IDLE;
  ControlBoard_InitAdc();
  ControlBoard_InitGpio();
  
  Serial.println("Control Board IoT DevKit");

  BLEDevice::init("Control Board");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(GenericAttribute_UUID);

  writeCharacs = pService->createCharacteristic(WriteCharacteristicUUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  readCharacs = pService->createCharacteristic(ReadCharacteristicUUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);

  writeCharacs->setCallbacks(new BLECharacteristicCallBack());
  readCharacs->setCallbacks(new BLECharacteristicCallBack());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  pAdvertising->addServiceUUID(GenericAttribute_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
}

void loop()
{
  if (isConnected)
  {
    float temp1 = 0;
    float temp2 = 0;
    char buffer[32];

    temp1 = ControlBoard_GetTemperature(NTC1);
    temp2 = ControlBoard_GetTemperature(NTC2);
    sprintf(buffer, "%.1f,%.1f", temp1, temp2);
    readCharacs->setValue((uint8_t *)buffer, strlen((char *)buffer));
    readCharacs->notify();

    memset(buffer, 0, sizeof(buffer));
    delay(500);
  }
  delay(500);
}