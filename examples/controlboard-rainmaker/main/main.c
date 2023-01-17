#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/timers.h"
#include "freertos/portmacro.h"
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>

#include <esp_rmaker_common_events.h>

#include <app_wifi.h>
#include <app_insights.h>

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"
#include "math.h"
#include "stdbool.h"
#include "appControlBoard.h"



static const char* TAG = "ControlBoard";

esp_rmaker_device_t *ntcOne;
esp_rmaker_device_t *ntcTwo;

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t RainMaker_WriteCallBack(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param, const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx){
	char* deviceName = NULL;
	if(ctx != NULL) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
    	deviceName =  esp_rmaker_device_get_name(device);
        ESP_LOGI(TAG, "Received value = %s for %s - %s", val.val.b? "true" : "false", esp_rmaker_device_get_name(device), esp_rmaker_param_get_name(param));

        if(strcmp(deviceName, "RelayOne") == 0){
        	gpio_set_level(RELAY1, val.val.b);
        }
        else if(strcmp(deviceName, "RelayTwo") == 0){
        	gpio_set_level(RELAY2, val.val.b);
		}
        else if(strcmp(deviceName, "RelayThree") == 0){
        	gpio_set_level(RELAY3, val.val.b);
		}
        else if(strcmp(deviceName, "RelayFour") == 0){
        	gpio_set_level(RELAY4, val.val.b);
		}
        else if(strcmp(deviceName, "TriacAc") == 0){

        	if(val.val.b == true){
        		App_TurnOnTriac();
        	}
        	else if(val.val.b == false){
        		App_TurnOffTriac();
        	}
		}

        esp_rmaker_param_update_and_report(param, val);
    }
    else if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_BRIGHTNESS_NAME) == 0) {
    	deviceName =  esp_rmaker_device_get_name(device);
    	if(strcmp(deviceName, "TriacAc") == 0){
    		App_SetPWM((float)val.val.i);
    		App_SetDimmerPot(val.val.i);
    	}
    }
    return ESP_OK;
}

/* Event handler for catching RainMaker events */
static void RainMaker_EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	if (event_base == RMAKER_EVENT) {
		switch (event_id) {
			case RMAKER_EVENT_INIT_DONE:
				ESP_LOGI(TAG, "RainMaker Initialised.");
				break;
			case RMAKER_EVENT_CLAIM_STARTED:
				ESP_LOGI(TAG, "RainMaker Claim Started.");
				break;
			case RMAKER_EVENT_CLAIM_SUCCESSFUL:
				ESP_LOGI(TAG, "RainMaker Claim Successful.");
				break;
			case RMAKER_EVENT_CLAIM_FAILED:
				ESP_LOGI(TAG, "RainMaker Claim Failed.");
				break;
			default:
				ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
		}
	}
	else if (event_base == RMAKER_COMMON_EVENT) {
		switch (event_id) {
			case RMAKER_EVENT_REBOOT:
				ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
				break;
			case RMAKER_EVENT_WIFI_RESET:
				ESP_LOGI(TAG, "Wi-Fi credentials reset.");
				break;
			case RMAKER_EVENT_FACTORY_RESET:
				ESP_LOGI(TAG, "Node reset to factory defaults.");
				break;
			case RMAKER_MQTT_EVENT_CONNECTED:
				ESP_LOGI(TAG, "MQTT Connected.");
				break;
			case RMAKER_MQTT_EVENT_DISCONNECTED:
				ESP_LOGI(TAG, "MQTT Disconnected.");
				break;
			case RMAKER_MQTT_EVENT_PUBLISHED:
				ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
				break;
			default:
				ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
		}
	}
	else if (event_base == APP_WIFI_EVENT) {
		switch (event_id) {
			case APP_WIFI_EVENT_QR_DISPLAY:
				ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
				break;
			case APP_WIFI_EVENT_PROV_TIMEOUT:
				ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
				break;
			case APP_WIFI_EVENT_PROV_RESTART:
				ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
				break;
			default:
				ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
				break;
		}
	}
	else {
		ESP_LOGW(TAG, "Invalid event received!");
	}
}

static void RainMaker_PushTemperature(TimerHandle_t handle){
	esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(ntcOne, ESP_RMAKER_PARAM_TEMPERATURE), esp_rmaker_float(App_GetTemperature(NTC1)));
	esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(ntcTwo, ESP_RMAKER_PARAM_TEMPERATURE), esp_rmaker_float(App_GetTemperature(NTC2)));
}

void app_main(void){
	esp_rmaker_device_t *relayOne;
	esp_rmaker_device_t *relayTwo;
	esp_rmaker_device_t *relayThree;
	esp_rmaker_device_t *relayFour;
	esp_rmaker_device_t *triacAc;
	static TimerHandle_t sensor_timer;

	/* Initialize Application specific hardware drivers and
	 * set initial state.
	 */
	esp_rmaker_console_init();
	App_InitializeBoard();

	/* Initialize NVS. */
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
	/* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
	 */
	app_wifi_init();

	/* Register an event handler to catch RainMaker events */
	ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &RainMaker_EventHandler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &RainMaker_EventHandler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(APP_WIFI_EVENT, ESP_EVENT_ANY_ID, &RainMaker_EventHandler, NULL));

	/* Initialize the ESP RainMaker Agent.
	 * Note that this should be called after app_wifi_init() but before app_wifi_start()
	 * */
	esp_rmaker_config_t rainmaker_cfg = {
		.enable_time_sync = false,
	};
	esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Control Board", "Switch");
	if (node == NULL) {
		ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
		vTaskDelay(5000/portTICK_PERIOD_MS);
		abort();
	}
	/* Create the relay Switch devices.
	 * You can optionally use the helper API esp_rmaker_switch_device_create() to
	 * avoid writing code for adding the name and power parameters.
	 */
	relayOne = esp_rmaker_device_create("RelayOne", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
	relayTwo = esp_rmaker_device_create("RelayTwo", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
	relayThree = esp_rmaker_device_create("RelayThree", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
	relayFour = esp_rmaker_device_create("RelayFour", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
	relayFour = esp_rmaker_device_create("RelayFour", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);
	triacAc = esp_rmaker_device_create("TriacAc", ESP_RMAKER_DEVICE_LIGHTBULB, NULL);

	ntcOne = esp_rmaker_temp_sensor_device_create("ntcOne", NULL, App_GetTemperature(NTC1));
	ntcTwo = esp_rmaker_temp_sensor_device_create("ntcTwo", NULL, App_GetTemperature(NTC2));

	/* Add the write callback for the device. We aren't registering any read callback yet as
	 * it is for future use.
	 */
	esp_rmaker_device_add_cb(relayOne, RainMaker_WriteCallBack, NULL);
	esp_rmaker_device_add_cb(relayTwo, RainMaker_WriteCallBack, NULL);
	esp_rmaker_device_add_cb(relayThree, RainMaker_WriteCallBack, NULL);
	esp_rmaker_device_add_cb(relayFour, RainMaker_WriteCallBack, NULL);
	esp_rmaker_device_add_cb(triacAc, RainMaker_WriteCallBack, NULL);

	/* Add the standard name parameter (type: esp.param.name), which allows setting a persistent,
	 * user friendly custom name from the phone apps. All devices are recommended to have this
	 * parameter.
	 */
	esp_rmaker_device_add_param(relayOne, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "relayOne"));
	esp_rmaker_device_add_param(relayTwo, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "relayTwo"));
	esp_rmaker_device_add_param(relayThree, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "relayThree"));
	esp_rmaker_device_add_param(relayFour, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "relayFour"));
	esp_rmaker_device_add_param(triacAc, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "triac"));

	/* Add the standard power parameter (type: esp.param.power), which adds a boolean param
	 * with a toggle switch ui-type.
	 */
	esp_rmaker_param_t *powerToRelayOne = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
	esp_rmaker_param_t *powerToRelayTwo = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
	esp_rmaker_param_t *powerToRelayThree = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
	esp_rmaker_param_t *powerToRelayFour = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
	esp_rmaker_param_t *powerToTriac = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
	esp_rmaker_param_t *brightnessToTriac = esp_rmaker_brightness_param_create(ESP_RMAKER_DEF_BRIGHTNESS_NAME, 0);

	esp_rmaker_device_add_param(relayOne, powerToRelayOne);
	esp_rmaker_device_add_param(relayTwo, powerToRelayTwo);
	esp_rmaker_device_add_param(relayThree, powerToRelayThree);
	esp_rmaker_device_add_param(relayFour, powerToRelayFour);
	esp_rmaker_device_add_param(triacAc, powerToTriac);
	esp_rmaker_device_add_param(triacAc, brightnessToTriac);

	/* Assign the power parameter as the primary, so that it can be controlled from the
	 * home screen of the phone apps.
	 */
	esp_rmaker_device_assign_primary_param(relayOne, powerToRelayOne);
	esp_rmaker_device_assign_primary_param(relayTwo, powerToRelayTwo);
	esp_rmaker_device_assign_primary_param(relayThree, powerToRelayThree);
	esp_rmaker_device_assign_primary_param(relayFour, powerToRelayFour);
	esp_rmaker_device_assign_primary_param(triacAc, powerToTriac);

	/* Add devices to the node */
	esp_rmaker_node_add_device(node, relayOne);
	esp_rmaker_node_add_device(node, relayTwo);
	esp_rmaker_node_add_device(node, relayThree);
	esp_rmaker_node_add_device(node, relayFour);
	esp_rmaker_node_add_device(node, ntcOne);
	esp_rmaker_node_add_device(node, ntcTwo);
	esp_rmaker_node_add_device(node, triacAc);

	/* Enable OTA */
	esp_rmaker_ota_enable_default();

	/* Enable timezone service which will be require for setting appropriate timezone
	 * from the phone apps for scheduling to work correctly.
	 * For more information on the various ways of setting timezone, please check
	 * https://rainmaker.espressif.com/docs/time-service.html.
	 */
	esp_rmaker_timezone_service_enable();

	/* Enable scheduling. */
	esp_rmaker_schedule_enable();

	/* Enable Scenes */
	esp_rmaker_scenes_enable();

	/* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
	app_insights_enable();

	/* Start the ESP RainMaker Agent */
	esp_rmaker_start();

	/* Start the Wi-Fi.
	 * If the node is provisioned, it will start connection attempts,
	 * else, it will start Wi-Fi provisioning. The function will return
	 * after a connection has been successfully established
	 */
	err = app_wifi_start(POP_TYPE_RANDOM);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
		vTaskDelay(5000/portTICK_PERIOD_MS);
		abort();
	}

	sensor_timer = xTimerCreate("TempUpdate", (5 * 1000) / portTICK_PERIOD_MS, pdTRUE, NULL, RainMaker_PushTemperature);
	if (sensor_timer != NULL) {
		xTimerStart(sensor_timer, 0);
	}
    while (true) {
       vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}
