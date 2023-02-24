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
#include "mics6814.h"
#include "esp_log.h"

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const static char * TAG = "mics6814";

static adc_oneshot_unit_handle_t adc_unit_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static esp_err_t adc_configure_channel(adc_conf_t * adc_conf);
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static int adc_get_value(mics6814_t * const me, mics6814_channel_e channel);
static float calculate_ratio(mics6814_t * const me, mics6814_channel_e channel);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief Initialize a MiCS-6814 sensor instance
  */
esp_err_t mics6814_init(mics6814_t * const me, adc_channel_t nh3_channel,
		adc_channel_t co_channel, adc_channel_t no2_channel) {
	esp_err_t ret = ESP_OK;

	ESP_LOGI(TAG, "Initializing MICS-6814 instance...");

	/* Fill mics6814 channels */
	me->co.adc_channel = co_channel;
	me->no2.adc_channel = no2_channel;
	me->nh3.adc_channel = nh3_channel;

	/* Initialize ADC channels */
	adc_configure_channel(&me->nh3);
	adc_configure_channel(&me->co);
	adc_configure_channel(&me->no2);

	/* Initialize ADC calibration */
	adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &me->nh3.adc_cali_handle);
	adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &me->co.adc_cali_handle);
	adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &me->no2.adc_cali_handle);

	/* Fill default calibration values */
	me->calib_values.nh3 = NH3_DEFAULT_CALIB_VALUE;
	me->calib_values.co = CO_DEFAULT_CALIB_VALUE;
	me->calib_values.no2 = NO2_DEFAULT_CALIB_VALUE;

	ESP_LOGI(TAG, "Initialization success");

	return ret;
}

/**
  * @brief Load calibration values for a MICS-6814 sensor instance
  */
void mics6814_load_calibration_data(mics6814_t * const me, uint16_t nh3_value,
		uint16_t co_value, uint16_t no2_value) {
	me->calib_values.nh3 = nh3_value;
	me->calib_values.co = co_value;
	me->calib_values.no2 = no2_value;
}

/**
  * @brief Get a value for a specific gas
  */
float mics6814_get_gas(mics6814_t * const me, gas_e gas) {
	float gas_value;

	/* Calculate the ratio for all the sensor channels */
	float ratio0 = calculate_ratio(me, NH3_CHANNEL);
	float ratio1 = calculate_ratio(me, CO_CHANNEL);
	float ratio2 = calculate_ratio(me, NO2_CHANNEL);

	/* Calculate the specific gas */
	switch (gas) {
		case CO_GAS:
			gas_value = pow(ratio1, -1.179) * 4.385;
			break;
    case NO2_GAS:
    	gas_value = pow(ratio2, 1.007) / 6.855;
			break;
    case NH3_GAS:
    	gas_value = pow(ratio0, -1.67) / 1.47;
    	break;
    case C3H8_GAS:
    	gas_value = pow(ratio0, -2.518) * 570.164;
			break;
		case C4H10_GAS:
			gas_value = pow(ratio0, -2.138) * 398.107;
			break;
    case CH4_GAS:
    	gas_value = pow(ratio1, -4.363) * 630.957;
    	break;
    case H2_GAS:
    	gas_value = pow(ratio1, -1.8) * 0.73;
    	break;
    case C2H5OH_GAS:
    	gas_value = pow(ratio1, -1.552) * 1.622;
    	break;
    default:
    	gas_value = -1;	/* Negative numbre for error */
			break;
	}

	return gas_value;
}

/* Private functions ---------------------------------------------------------*/
static esp_err_t adc_configure_channel(adc_conf_t * adc_conf) {
	esp_err_t ret;

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
			ESP_LOGE(TAG, "Error creating new ADC unit");
			return ret;
		}
	}

	/* Set up the ADC channel structure */
	adc_oneshot_chan_cfg_t adc_channel_config = {
			.bitwidth = ADC_BITWIDTH_12,
			.atten = ADC_ATTEN_DB_11,
	};

	/* Configure ADC channel instance */
	ret = adc_oneshot_config_channel(adc_unit_handle, adc_conf->adc_channel, &adc_channel_config);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error configuring ADC channel");
		return ret;
	}

	return ret;
}

static bool adc_calibration_init (adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t * out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

  if (!calibrated) {
      ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
      adc_cali_line_fitting_config_t cali_config = {
          .unit_id = unit,
          .atten = atten,
          .bitwidth = ADC_BITWIDTH_12,
      };
      ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
      if (ret == ESP_OK) {
          calibrated = true;
      }
  }

  * out_handle = handle;
  if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
      ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
      ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

static int adc_get_value(mics6814_t * const me, mics6814_channel_e channel) {
	adc_conf_t * adc_conf = NULL;

	switch (channel) {
		case NH3_CHANNEL:
			adc_conf = &me->nh3;
			break;
		case CO_CHANNEL:
			adc_conf = &me->co;
			break;
		case NO2_CHANNEL:
			adc_conf = &me->no2;
			break;
		default:
			break;
	}

	esp_err_t ret;
	int adc_value = 0;
	int adc_acum = 0;

	/* Acumulate the obtained ADC value 64 times */
	for (uint8_t i = 0; i < 64; i++) {
		/* Get ADC raw value */
		ret = adc_oneshot_read(adc_unit_handle, adc_conf->adc_channel, &adc_value);

		/* Add the ADC value to acumulator when the ADC read is successfully. In
		 * other cases add 0 to acumulator */
		if (ret != ESP_OK) {
			adc_acum += 0;
		}
		else {
			adc_acum += adc_value;
		}
	}

	/* Divide by 64 and assign to output variable */
	adc_value = adc_acum >> 6;

	return adc_value;
}

static float calculate_ratio(mics6814_t * const me, mics6814_channel_e channel) {
	float base_resistance, current_resistance;

	switch (channel) {
		case NH3_CHANNEL:
			base_resistance = (float)me->calib_values.nh3;
			current_resistance = (float)adc_get_value(me, NH3_CHANNEL);
			break;
		case CO_CHANNEL:
			base_resistance = (float)me->calib_values.co;
			current_resistance = (float)adc_get_value(me, CO_CHANNEL);
			break;
		case NO2_CHANNEL:
			base_resistance = (float)me->calib_values.no2;
			current_resistance = (float)adc_get_value(me, NO2_CHANNEL);
			break;
		default:
			/* Base and current resistance are 0 if the channel is invalid */
			base_resistance = 0;
			current_resistance = 0;
			break;
	}

	return current_resistance / base_resistance * (4096.0 - base_resistance) / (4096.0 - current_resistance);;
}

/***************************** END OF FILE ************************************/
