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
#include "mics6814.h"
#include "esp_log.h"

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const static char * TAG = "mics6814";

/* todo: array? */
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

int adc_raw[2][10];

/* Private function prototypes -----------------------------------------------*/
static esp_err_t adc_configure_channel(adc_conf_t * adc_conf);
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static int get_adc_value (adc_conf_t adc_conf);

/* Exported functions --------------------------------------------------------*/
esp_err_t mics6814_init (mics6814_t * const me) {
	esp_err_t ret = ESP_OK;

	ESP_LOGI(TAG, "Initializing MICS-6814 instance...");

	adc_configure_channel(&me->nh3);
	adc_configure_channel(&me->co);
	adc_configure_channel(&me->no2);

	/* Initialize ADC calibration */
	adc_calibration_init(me->nh3.adc_unit, ADC_ATTEN_DB_11, &me->nh3.adc_cali_handle);
	adc_calibration_init(me->co.adc_unit, ADC_ATTEN_DB_11, &me->co.adc_cali_handle);
	adc_calibration_init(me->no2.adc_unit, ADC_ATTEN_DB_11, &me->no2.adc_cali_handle);

	ESP_LOGI(TAG, "Done!");

	return ret;
}

int mics6814_get_gas(mics6814_t * const me, gas_e gas) {
	int gas_value = 0;

	switch (gas) {
		case NH3:
			gas_value = get_adc_value(me->nh3);
			break;
		case CO:
			gas_value = get_adc_value(me->co);
			break;
		case NO2:
			gas_value = get_adc_value(me->no2);
			break;
		default:
			gas_value = 0;
			break;
	}

	return gas_value;
}

/* Private functions ---------------------------------------------------------*/
static esp_err_t adc_configure_channel (adc_conf_t * adc_conf) {
	esp_err_t ret;

	/* If the ADC unit handle are initialized, then ... */
	if (adc1_handle != NULL) {
		adc_conf->adc_handle = adc1_handle;
	}
	else if (adc2_handle != NULL) {
		adc_conf->adc_handle = adc2_handle;
	}
	/* Check if the ADC units are currently initialized */
	else {
		/* Fill an ADC unit configuration */
		adc_oneshot_unit_init_cfg_t adc_config = {
				.unit_id = adc_conf->adc_unit,
				.ulp_mode = ADC_ULP_MODE_DISABLE,
		};

		/* Create a new ADC unit according the ADC unit specified in the instance
		 * structure */
		if (adc_conf->adc_unit == ADC_UNIT_1) {
			ret = adc_oneshot_new_unit (&adc_config, &adc1_handle);
			adc_conf->adc_handle = adc1_handle;
		}
		else {
			ret = adc_oneshot_new_unit (&adc_config, &adc2_handle);
			adc_conf->adc_handle = adc2_handle;
		}

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
	ret = adc_oneshot_config_channel(adc_conf->adc_handle, adc_conf->adc_channel, &adc_channel_config);

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
          .bitwidth = ADC_BITWIDTH_DEFAULT,
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

static int get_adc_value (adc_conf_t adc_conf) {
	esp_err_t ret;
	int adc_value = 0;
	int adc_acum = 0;

	/* Acumulate the ADC value obtained 32 times */
	for (uint8_t i = 0; i < 31; i++) {
		/* Get ADC raw value */
		ret = adc_oneshot_read (adc_conf.adc_handle, adc_conf.adc_channel, &adc_value);

		/* Add the ADC value to acumulator when the ADC read is successfully. In
		 * other cases add 0 to acumulator */
		if (ret != ESP_OK) {
			adc_acum += 0;
		}
		else {
			adc_acum += adc_value;
		}
	}

	/* Divide by 32 and assign to output variable */
	return (adc_acum >> 5);
}

/***************************** END OF FILE ************************************/
