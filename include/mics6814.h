/**
  ******************************************************************************
  * @file           : mics6814.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MICS6814_H_
#define MICS6814_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "ads101x.h"
#include "driver/i2c_master.h"

/* Exported macro ------------------------------------------------------------*/
#define NH3_DEFAULT_CALIB_VALUE	580
#define CO_DEFAULT_CALIB_VALUE	790
#define NO2_DEFAULT_CALIB_VALUE	900

/* Exported typedef ----------------------------------------------------------*/
/* todo: write descriptions */
typedef enum {
	MICS6814_CH_NO2 = ADS101X_CHANNEL_0,
	MICS6814_CH_NH3 = ADS101X_CHANNEL_1,
	MICS6814_CH_CO = ADS101X_CHANNEL_2
} mics6814_channel_e;

/* todo: write descriptions */
typedef struct {
	uint16_t nh3;
	uint16_t co;
	uint16_t no2;
} mics6814_calib_values_t;

/* todo: write descriptions */
typedef struct {
	float co;
	float no2;
	float nh3;
	float c3h8;
	float c4h10;
	float ch4;
	float h2;
	float c2h5oh;
} mics6814_gases_values_t;

/* todo: write descriptions */
typedef struct {
	ads101x_t adc;
	mics6814_calib_values_t calib_values;
	mics6814_gases_values_t gases_values;
} mics6814_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Initialize a MiCS-6814 sensor instance
  *
  * @param me             : Pointer to a mics6814_t structure
  * @param i2c_bus_handle :
  * @param int_pin        :
  *
  * @retval
  * 	- ESP_OK on success
  */
esp_err_t mics6814_init(mics6814_t *const me, i2c_master_bus_handle_t i2c_bus_handle, gpio_num_t int_pin);

/**
  * @brief Load calibration values for a MICS-6814 sensor instance
  *
  * @param me Pointer to a mics6814_t structure
  * @param nh3_value Value for the NH3 sensor
  * @param co_value Value for the CO sensor
  * @param co2_value Value for the NO2 sensor
  */
void mics6814_load_calibration_data(mics6814_t *const me, uint16_t nh3_value,
		uint16_t co_value, uint16_t no2_value);

/**
  * @brief Get a value for a specific gas
  *
  * @param me Pointer to a mics6814_t structure
  */
void mics6814_get_gases_values(mics6814_t *const me);

/**
  * @brief Get a value for a specific gas
  *
  * @param me      : Pointer to a mics6814_t structure
  * @param seconds : The number of seconds that must pass before than we will
  *                  assume that the calibration is complete (less than 64
  *                  seconds to avoid overflow)
  * @param delta   : Tolerance for the average of the current value
  */
void mics6814_calibrate(mics6814_t *const me, uint8_t seconds, uint8_t delta);

#ifdef __cplusplus
}
#endif

#endif /* MICS6814_H_ */

/***************************** END OF FILE ************************************/
