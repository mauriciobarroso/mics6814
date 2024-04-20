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

/* Private function prototypes -----------------------------------------------*/
static int16_t adc_get_value(mics6814_t *const me, mics6814_channel_e channel);
static float calculate_ratio(mics6814_t *const me, mics6814_channel_e channel);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief Initialize a MiCS-6814 sensor instance
  */
esp_err_t mics6814_init(mics6814_t *const me, i2c_master_bus_handle_t i2c_bus_handle, gpio_num_t int_pin) {
	ESP_LOGI(TAG, "Initializing MiCS-6814 instance...");

	/* Variable to return error code */
	esp_err_t ret = ESP_OK;

	/* Initialize ADC */
	ret = ads101x_init(&me->adc, ADS101X_MODEL_5, int_pin, i2c_bus_handle, ADS101X_I2C_ADDRESS);

	if (ret != ESP_OK) {
		return ESP_FAIL;
	}

	ads101x_set_gain(&me->adc, ADS101X_GAIN_ONE);
	ads101x_set_data_rate(&me->adc, ADS101X_DATA_RATE_3300SPS);

	/* Fill default calibration values */
	me->calib_values.nh3 = 120;
	me->calib_values.co = 300;
	me->calib_values.no2 = 710;

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
void mics6814_get_gases_values(mics6814_t *const me) {
	/* Calculate the ratio for all the sensor channels */
	float ratio0 = calculate_ratio(me, MICS6814_CH_NH3);
	float ratio1 = calculate_ratio(me, MICS6814_CH_CO);
	float ratio2 = calculate_ratio(me, MICS6814_CH_NO2);

	/* Calculate the gases values */
	me->gases_values.co = pow(ratio1, -1.179) * 4.385;
	me->gases_values.no2 = pow(ratio2, 1.007) / 6.855;
	me->gases_values.nh3 = pow(ratio0, -1.67) / 1.47;
	me->gases_values.c3h8 = pow(ratio0, -2.518) * 570.164;
	me->gases_values.c4h10 = pow(ratio0, -2.138) * 398.107;
	me->gases_values.ch4 = pow(ratio1, -4.363) * 630.957;
	me->gases_values.h2 = pow(ratio1, -1.8) * 0.73;
	me->gases_values.c2h5oh = pow(ratio1, -1.552) * 1.622;
}

/**
  * @brief Get a value for a specific gas
  */
void mics6814_calibrate(mics6814_t *const me, uint8_t seconds, uint8_t delta) {
	/* Measurement buffers */
	uint16_t nh3_buf[seconds];
	uint16_t co_buf[seconds];
	uint16_t no2_buf[seconds];

	/* Pointers for the next item in the buffer */
	uint8_t nh3_ptr = 0;
	uint8_t co_ptr = 0;
	uint8_t no2_ptr = 0;

	/* The current floating amount in the buffer */
	uint32_t nh3_sum = 0;
	uint32_t co_sum = 0;
	uint32_t no2_sum = 0;

	/* Current measurement */
	uint16_t nh3_curr;
	uint16_t co_curr;
	uint16_t no2_curr;

	/* Flag of stability of indications */
	bool nh3_is_stable = false;
	bool co_is_stable = false;
	bool no2_is_stable = false;

	/* Clear the measurements buffers */
	for (uint8_t i = 0; i < seconds; i++) {
		nh3_buf[i] = 0;
		co_buf[i] = 0;
		no2_buf[i] = 0;
	}

	/* Perform the calibration process */
	do {
		vTaskDelay(pdMS_TO_TICKS(1000));

		/* Get current values */
		nh3_curr = adc_get_value(me, MICS6814_CH_NH3);
		co_curr = adc_get_value(me, MICS6814_CH_CO);
		no2_curr = adc_get_value(me, MICS6814_CH_NO2);

		printf("nh3_curr: %d\r\n", nh3_curr);
		printf("co_curr: %d\r\n", co_curr);
		printf("no2_curr: %d\r\n", no2_curr);

		/* Store new values in the buffer */
		nh3_sum += nh3_curr - nh3_buf[nh3_ptr];
		co_sum += co_curr - co_buf[co_ptr];
		no2_sum += no2_curr - no2_buf[no2_ptr];

		printf("nh3_sum: %ld\r\n", nh3_sum);
		printf("co_sum: %ld\r\n", co_sum);
		printf("no2_sum: %ld\r\n\n", no2_sum);

		/* Store new values in the buffer */
		nh3_buf[nh3_ptr] = nh3_curr;
		co_buf[co_ptr] = co_curr;
		no2_buf[no2_ptr] = no2_curr;

		/* Set flag states */
		nh3_is_stable = abs(nh3_sum / seconds - nh3_curr) < delta;
		co_is_stable = abs(co_sum / seconds - co_curr) < delta;
		no2_is_stable = abs(no2_sum / seconds - no2_curr) < delta;

		/* Pointer to a buffer */
		nh3_ptr = (nh3_ptr + 1) % seconds;
		co_ptr = (co_ptr + 1) % seconds;
		no2_ptr = (no2_ptr + 1) % seconds;
	} while (!nh3_is_stable || !co_is_stable || !no2_is_stable);

	printf("nh3 calib: %d\r\n", me->calib_values.nh3);
	printf("co calib: %d\r\n", me->calib_values.co);
	printf("no2 calib: %d\r\n", me->calib_values.no2);

	/* Assign the new calibration values */
	me->calib_values.nh3 = nh3_sum / seconds;
	me->calib_values.co = co_sum / seconds;
	me->calib_values.no2 = no2_sum / seconds;

	printf("nh3 calib: %d\r\n", me->calib_values.nh3);
	printf("co calib: %d\r\n", me->calib_values.co);
	printf("no2 calib: %d\r\n", me->calib_values.no2);
}

/* Private functions ---------------------------------------------------------*/
static int16_t adc_get_value(mics6814_t *const me, mics6814_channel_e channel) {
	int16_t adc_value;
	int32_t adc_sum = 0;

	for (uint8_t i = 0;  i < 32; i++) {
		ads101x_read_single_ended(&me->adc, channel, &adc_value);
		adc_sum += adc_value;
	}

	return (adc_sum >> 5);
}

static float calculate_ratio(mics6814_t *const me, mics6814_channel_e channel) {
	float base_resistance, current_resistance;

	switch (channel) {
		case MICS6814_CH_NH3:
			base_resistance = (float)me->calib_values.nh3;
			break;
		case MICS6814_CH_CO:
			base_resistance = (float)me->calib_values.co;
			break;
		case MICS6814_CH_NO2:
			base_resistance = (float)me->calib_values.no2;
			break;
		default:
			/* Base and current resistance are 0 if the channel is invalid */
			base_resistance = 0;
			current_resistance = 0;
			break;
	}

	current_resistance = (float)adc_get_value(me, channel);

	return current_resistance / base_resistance * (2047.0 - base_resistance) / (2047.0 - current_resistance);
}

/***************************** END OF FILE ************************************/
