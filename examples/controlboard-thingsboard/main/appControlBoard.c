/*
 * appControlBoard.c
 *
 *  Created on: 7 de jan de 2023
 *      Author: deivedwilliam
 */
#include "appControlBoard.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

#include "stdbool.h"
#include "math.h"


#define IDLE 					-1
#define TRIGGER_TRIAC_INTERVAL 	20
#define MAX_BRIGHTNESS 			800
#define MIN_BRIGHTNESS 			7000


static const float R1 = 10000.0;   	// voltage divider resistor value
static const float Beta = 3435;  	// Beta value
static const float To = 298.15;    	// Temperature in Kelvin for 25 degree Celsius
static const float Ro = 10000.0;   		// Resistance of Thermistor at 25 degree Celsius
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool isPinHighEnabled = false;
static volatile long currentBrightness = MIN_BRIGHTNESS;
static const char* TAG = "App_ControlBoard";
static void IRAM_ATTR setTimerPinLow(void);
static void IRAM_ATTR setTimerPinHigh(int brightness);
static int App_ReadAdc(adc_channel_t channel);



static esp_adc_cal_characteristics_t adc1_chars;




static bool App_AdcCalibration(void){
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

static esp_err_t App_AdcConfig(void){
	esp_err_t err = ESP_OK;

	bool cali_enable = App_AdcCalibration();

	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
	adc1_config_channel_atten(NTC1, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(NTC2, ADC_ATTEN_DB_11);

	return err;
}

static esp_err_t App_PWMConfig(void){
	ledc_timer_config_t ledcTimerConfig = {0};
	ledc_channel_config_t ledcChannelConfig = {0};
	esp_err_t err = ESP_OK;

	ledcTimerConfig.duty_resolution = LEDC_TIMER_13_BIT;
	ledcTimerConfig.freq_hz = 5000;
	ledcTimerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
	ledcTimerConfig.timer_num = LEDC_TIMER_0;
	ledcTimerConfig.clk_cfg = LEDC_AUTO_CLK;

	ledcChannelConfig.channel = LEDC_CHANNEL_0;
	ledcChannelConfig.gpio_num = PWM_OUT;
	ledcChannelConfig.hpoint = 0;
	ledcChannelConfig.timer_sel = LEDC_TIMER_0;
	ledcChannelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
	ledcChannelConfig.duty = 0;
	ledcChannelConfig.intr_type = LEDC_INTR_DISABLE;

	err = ledc_timer_config(&ledcTimerConfig);
	if(err != ESP_OK){
		ESP_LOGI(TAG, "ledc timer configuration failed");
		return err;
	}
	err = ledc_channel_config(&ledcChannelConfig);
	if(err != ESP_OK){
		ESP_LOGI(TAG, "ledc channel configuration failed");
		return err;
	}

	return err;
}

int App_ReadAdc(adc_channel_t channel){
	return adc1_get_raw(channel);
}

float App_GetTemperature(adc_channel_t channel) {
	float vOut = 0;
	float Rtherm = 0;
	const float VSupply = 3.3f;
	float tempKelvin = 0;
	float tempCelsius = 0;

	vOut = (float)(VSupply * App_ReadAdc(channel))/4095.0f ;

	Rtherm = R1 * (VSupply/vOut - 1);

	tempKelvin = 1/(1/To + log(Rtherm/Ro)/Beta);    		// Temperature in Kelvin
	tempCelsius = tempKelvin - 273.15;                   	// Celsius
	return tempCelsius;
}

IRAM_ATTR void App_SetDimmerPot(int potency){
	int newBrightness = 7000 - (int)(6200*((float)potency/100.0f));
	portENTER_CRITICAL(&mux);
	currentBrightness = newBrightness;
	portEXIT_CRITICAL(&mux);
}

esp_err_t App_SetPWM(float duty){
	int calcDuty = (int)((1 << LEDC_TIMER_13_BIT) * duty/100.0f);
	printf("%i\r\n", calcDuty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, calcDuty);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    return ESP_OK;
}


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
	timer_config_t timerConfig = {0};

	timerConfig.alarm_en = true;
	timerConfig.auto_reload = false;
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
	timer_config_t timerConfig = {0};

	timerConfig.alarm_en = true;
	timerConfig.auto_reload = false;
	timerConfig.counter_dir = TIMER_COUNT_UP;
	timerConfig.divider = 80;
	timerConfig.counter_en = TIMER_PAUSE;

	timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, brightness);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ISR_turnPinHigh, NULL, 0);
	timer_start(TIMER_GROUP_0, TIMER_0);
}


static esp_err_t App_GpioConfig(void){
	gpio_config_t configGpio = {0};
	esp_err_t err = ESP_OK;

	configGpio.intr_type = GPIO_INTR_DISABLE;
	configGpio.mode = GPIO_MODE_OUTPUT;
	configGpio.pin_bit_mask = (1 << RELAY1)|(1 << RELAY2)|(1 << RELAY3)|(1 << RELAY4)|(1 << TRIAC_TRIGGER);
	configGpio.pull_down_en = false;
	configGpio.pull_up_en = false;

	ESP_LOGI(TAG, "Configuring output GPIO...");
	err = gpio_config(&configGpio);
	if(err != ESP_FAIL){
		gpio_set_level(RELAY1, 0);
		gpio_set_level(RELAY2, 0);
		gpio_set_level(RELAY3, 0);
		gpio_set_level(RELAY4, 0);
		gpio_set_level(TRIAC_TRIGGER, 0);
	}

	configGpio.intr_type = GPIO_INTR_POSEDGE;
	configGpio.mode = GPIO_MODE_INPUT;
	configGpio.pin_bit_mask = (1 << ZC_DETECT);
	configGpio.pull_down_en = false;
	configGpio.pull_up_en = false;
	err = gpio_config(&configGpio);
	if(err != ESP_OK){
		ESP_LOGE(TAG, "config of zero cross input detect failed");
		return err;
	}
    //install gpio isr service
    gpio_install_isr_service(0);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ZC_DETECT, ZEROCross_ISREvent, NULL);
	return err;
}

void App_TurnOnTriac(void){
	portENTER_CRITICAL(&mux);
	currentBrightness = MAX_BRIGHTNESS;
	GPIO.out_w1ts = (1 << TRIAC_TRIGGER);
	portEXIT_CRITICAL(&mux);
}

void App_TurnOffTriac(void){
	portENTER_CRITICAL(&mux);
	currentBrightness = IDLE;
	GPIO.out_w1tc = (1 << TRIAC_TRIGGER);
	portEXIT_CRITICAL(&mux);
}

esp_err_t App_InitializeBoard(void){
	esp_err_t err = ESP_OK;

	currentBrightness = IDLE;

	err = App_GpioConfig();
	if(err != ESP_OK){
		ESP_LOGE(TAG, "gpio initialization failed");
		return err;
	}
	err = App_AdcConfig();
	if(err != ESP_OK){
		ESP_LOGE(TAG, "adc initialization failed");
		return err;
	}
	err = App_PWMConfig();
	if(err != ESP_OK){
		ESP_LOGE(TAG, "pwm initialization failed");
		return err;
	}
	App_SetPWM(0);

	return err;
}

