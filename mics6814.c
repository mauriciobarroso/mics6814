/**
  ******************************************************************************
  * @file           : mics6814.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Dec 6, 2022
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mics6814.h"
#include "esp_log.h"

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const static char * TAG = "mics6814";

static adc_oneshot_unit_handle_t adc_unit_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static esp_err_t adc_configure_channel(adc_conf_t *adc_conf);
static int adc_get_value(adc_channel_t channel);
static float calculate_ratio(mics6814_t *const me, mics6814_channel_e channel);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief Initialize a MiCS-6814 sensor instance
  */
esp_err_t mics6814_init(mics6814_t *const me, adc_channel_t nh3_channel,
		adc_channel_t co_channel, adc_channel_t no2_channel) {
	ESP_LOGI(TAG, "Initializing MiCS-6814 instance...");

	esp_err_t ret = ESP_OK;

	/* Fill mics6814 channels */
	me->nh3.adc_channel = nh3_channel;
	me->co.adc_channel = co_channel;
	me->no2.adc_channel = no2_channel;

	/* Initialize ADC channels */
	adc_configure_channel(&me->nh3);
	adc_configure_channel(&me->co);
	adc_configure_channel(&me->no2);

	/* Fill default calibration values */
	me->calib_values.nh3 = NH3_DEFAULT_CALIB_VALUE;
	me->calib_values.co = CO_DEFAULT_CALIB_VALUE;
	me->calib_values.no2 = NO2_DEFAULT_CALIB_VALUE;

	/* Print success message */
	ESP_LOGI(TAG, "Initialized successfully");

	/* Return ESP_OK */
	return ret;
}

/**
  * @brief Load calibration values for a MICS-6814 sensor instance
  */
void mics6814_load_calibration_data(mics6814_t *const me, uint16_t nh3_value,
		uint16_t co_value, uint16_t no2_value) {
	me->calib_values.nh3 = nh3_value;
	me->calib_values.co = co_value;
	me->calib_values.no2 = no2_value;
}

/**
  * @brief Get a value for a specific gas
  */
float mics6814_get_gas(mics6814_t *const me, gas_e gas) {
	float gas_value;

	/* Calculate the ratio for all the sensor channels */
	float ratio0 = calculate_ratio(me, MICS6814_CH_NH3);
	float ratio1 = calculate_ratio(me, MICS6814_CH_CO);
	float ratio2 = calculate_ratio(me, MICS6814_CH_NO2);

	/* Calculate the specific gas */
	switch (gas) {
		case MICS6814_GAS_CO:
			gas_value = pow(ratio1, -1.179) * 4.385;
			break;
    case MICS6814_GAS_NO2:
    	gas_value = pow(ratio2, 1.007) / 6.855;
			break;
    case MICS6814_GAS_NH3:
    	gas_value = pow(ratio0, -1.67) / 1.47;
    	break;
    case MICS6814_GAS_C3H8:
    	gas_value = pow(ratio0, -2.518) * 570.164;
			break;
		case MICS6814_GAS_C4H10:
			gas_value = pow(ratio0, -2.138) * 398.107;
			break;
    case MICS6814_GAS_CH4:
    	gas_value = pow(ratio1, -4.363) * 630.957;
    	break;
    case MICS6814_GAS_H2:
    	gas_value = pow(ratio1, -1.8) * 0.73;
    	break;
    case MICS6814_GAS_C2H5OH:
    	gas_value = pow(ratio1, -1.552) * 1.622;
    	break;
    default:
    	gas_value = -1;	/* Negative number for error */
			break;
	}

	return gas_value;
}

/**
  * @brief Get a value for a specific gas
  */
void mics6814_calibrate(mics6814_t *const me) {
	/* The number of seconds that must pass before than we will assume that the
	 * calibration is complete (less than 64 seconds to avoid overflow) */
	uint8_t seconds = 10;

	/* Tolerance for the average of the current value */
	uint8_t delta = 2;

	/* Measurement buffers */
	uint16_t bufferNH3[seconds];
	uint16_t bufferCO[seconds];
	uint16_t bufferNO2[seconds];

	/* Pointers for the next item in the buffer */
	uint8_t pntrNH3 = 0;
	uint8_t pntrCO = 0;
	uint8_t pntrNO2 = 0;

	/* The current floating amount in the buffer */
	uint32_t fltSumNH3 = 0;
	uint32_t fltSumCO = 0;
	uint32_t fltSumNO2 = 0;

	/* Current measurement */
	uint16_t curNH3;
	uint16_t curCO;
	uint16_t curNO2;

	/* Flag of stability of indications */
	bool isStableNH3 = false;
	bool isStableCO = false;
	bool isStableNO2 = false;

	/* Clear the measurements buffers */
	for (uint8_t i = 0; i < seconds; i++) {
		bufferNH3[i] = 0;
		bufferCO[i] = 0;
		bufferNO2[i] = 0;
	}

	/* Perform the calibration process */
	do {
		vTaskDelay(pdMS_TO_TICKS(1000));

		/* Get current values */
		curNH3 = adc_get_value(me->nh3.adc_channel);
		curCO = adc_get_value(me->co.adc_channel);
		curNO2 = adc_get_value(me->no2.adc_channel);

		printf("curNH3: %d\r\n", curNH3);
		printf("curCO: %d\r\n", curCO);
		printf("curNO2: %d\r\n", curNO2);

		/* Store new values in the buffer */
		fltSumNH3 += curNH3 - bufferNH3[pntrNH3];
		fltSumCO += curCO - bufferCO[pntrCO];
		fltSumNO2 += curNO2 - bufferNO2[pntrNO2];

		printf("fltSumNH3: %ld\r\n", fltSumNH3);
		printf("fltSumCO: %ld\r\n", fltSumCO);
		printf("fltSumNO2: %ld\r\n\n", fltSumNO2);

		/* Store new values in the buffer */
		bufferNH3[pntrNH3] = curNH3;
		bufferCO[pntrCO] = curCO;
		bufferNO2[pntrNO2] = curNO2;

		/* Set flag states */
		isStableNH3 = abs(fltSumNH3 / seconds - curNH3) < delta;
		isStableCO = abs(fltSumCO / seconds - curCO) < delta;
		isStableNO2 = abs(fltSumNO2 / seconds - curNO2) < delta;

		/* Pointer to a buffer */
		pntrNH3 = (pntrNH3 + 1) % seconds;
		pntrCO = (pntrCO + 1) % seconds;
		pntrNO2 = (pntrNO2 + 1) % seconds;
	} while (!isStableNH3 || !isStableCO || !isStableNO2);

	printf("nh3 calib: %d\r\n", me->calib_values.nh3);
	printf("co calib: %d\r\n", me->calib_values.co);
	printf("no2 calib: %d\r\n", me->calib_values.no2);

	/* Assign the new calibration values */
	me->calib_values.nh3 = fltSumNH3 / seconds;
	me->calib_values.co = fltSumCO / seconds;
	me->calib_values.no2 = fltSumNO2 / seconds;

	printf("nh3 calib: %d\r\n", me->calib_values.nh3);
	printf("co calib: %d\r\n", me->calib_values.co);
	printf("no2 calib: %d\r\n", me->calib_values.no2);
}

/* Private functions ---------------------------------------------------------*/
static esp_err_t adc_configure_channel(adc_conf_t *adc_conf) {
	esp_err_t ret = ESP_OK;

	/* Fill an ADC unit configuration */
	adc_oneshot_unit_init_cfg_t adc_config = {
			.unit_id = ADC_UNIT_1,
			.ulp_mode = ADC_ULP_MODE_DISABLE,
	};

	/* Create a new ADC unit if does not exist */
	if (adc_unit_handle == NULL) {
		ret = adc_oneshot_new_unit(&adc_config, &adc_unit_handle);

		/* Check if the ADC unit was created correctly */
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to create ADC unit");
			return ret;
		}
	}

	/* Set up the ADC channel structure */
	adc_oneshot_chan_cfg_t adc_channel_config = {
			.bitwidth = ADC_BITWIDTH_13,
			.atten = ADC_ATTEN_DB_11,
	};

	/* Configure ADC channel instance */
	ret = adc_oneshot_config_channel(adc_unit_handle, adc_conf->adc_channel, &adc_channel_config);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure ADC channel %d", adc_conf->adc_channel);
		return ret;
	}

	/* Return ESP_OK */
	return ret;
}

static int adc_get_value(adc_channel_t channel) {
	int adc_value = 0;
	int adc_acum = 0;
	uint8_t count = 0;

	while (count < 32) {
		/* Get ADC raw value and add to acumulator when the ADC read is successfully */
		if (adc_oneshot_read(adc_unit_handle, channel, &adc_value) == ESP_OK) {
			adc_acum += adc_value;
			count++;
		}
	}

	/* Acumulate the obtained ADC value 64 times */
//	for (uint8_t i = 0; i < 32; i++) {
//		/* Get ADC raw value and add to acumulator when the ADC read is successfully */
//		if (adc_oneshot_read(adc_unit_handle, channel, &adc_value) == ESP_OK) {
//			adc_acum += adc_value;
//		}
//	}

	/* Divide by 32 and assign to output variable */
	adc_value = adc_acum >> 5;

	return adc_value;
}

static float calculate_ratio(mics6814_t *const me, mics6814_channel_e channel) {
	float base_resistance, current_resistance;

	switch (channel) {
		case MICS6814_CH_NH3:
			base_resistance = (float)me->calib_values.nh3;
			current_resistance = (float)adc_get_value(me->nh3.adc_channel);
			break;
		case MICS6814_CH_CO:
			base_resistance = (float)me->calib_values.co;
			current_resistance = (float)adc_get_value(me->co.adc_channel);
			break;
		case MICS6814_CH_NO2:
			base_resistance = (float)me->calib_values.no2;
			current_resistance = (float)adc_get_value(me->no2.adc_channel);
			break;
		default:
			/* Base and current resistance are 0 if the channel is invalid */
			base_resistance = 0;
			current_resistance = 0;
			break;
	}

	return current_resistance / base_resistance * (8191.0 - base_resistance) / (8191.0 - current_resistance);
}

/***************************** END OF FILE ************************************/
