/*
 * appControlBoard.h
 *
 *  Created on: 7 de jan de 2023
 *      Author: deivedwilliam
 */

#ifndef MAIN_APPCONTROLBOARD_H_
#define MAIN_APPCONTROLBOARD_H_

#include "driver/adc.h"

#define RELAY1 			GPIO_NUM_19
#define RELAY2 			GPIO_NUM_21
#define RELAY3			GPIO_NUM_22
#define RELAY4 			GPIO_NUM_23
#define NTC1 			ADC_CHANNEL_0
#define NTC2 			ADC_CHANNEL_3
#define ZC_DETECT  		GPIO_NUM_26
#define TRIAC_TRIGGER 	GPIO_NUM_27

#define PWM_OUT			GPIO_NUM_12



IRAM_ATTR void App_SetDimmerPot(int potency);
float App_GetTemperature(adc_channel_t channel);
void App_TurnOnTriac(void);
void App_TurnOffTriac(void);
esp_err_t App_InitializeBoard(void);
esp_err_t App_SetPWM(float duty);

#endif /* MAIN_APPCONTROLBOARD_H_ */
