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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/
/* todo: write descriptions */
typedef enum {
	NH3 = 0,
	CO,
	NO2
} gas_e;

/* todo: write descriptions */
typedef struct {
	adc_oneshot_unit_handle_t adc_handle;
	adc_unit_t adc_unit;
	adc_channel_t adc_channel;
	adc_cali_handle_t adc_cali_handle;
} adc_conf_t;

/* todo: write descriptions */
typedef struct {
	adc_conf_t nh3;
	adc_conf_t co;
	adc_conf_t no2;
} mics6814_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Initialize a MICS-6814 sensor instance
  *
  * @param me Pointer to a mics6814_t structure
  *
  * @retval
  * 	- ESP_OK on success
  */
esp_err_t mics6814_init (mics6814_t * const me);

/* todo: write description */
/**
  * @brief Initialize a MICS-6814 sensor instance
  *
  * @param me Pointer to a mics6814_t structure
  *
  * @retval
  * 	- ESP_OK on success
  */
int mics6814_get_gas(mics6814_t * const me, gas_e gas);

#ifdef __cplusplus
}
#endif

#endif /* MICS6814_H_ */

/***************************** END OF FILE ************************************/
