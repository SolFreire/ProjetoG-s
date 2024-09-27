#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include <esp_err.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

const static char *TAG = "EXAMPLE";

#define BUZZER_GPIO GPIO_NUM_15
#define ADC1_CHAN3 ADC_CHANNEL_3
#define ADC_ATTEN ADC_ATTEN_DB_12
#define NUM_SAMPLES 10
#define MAX_ADC 4095    // Resolução de 12 bits (0-4095)
#define V_REF 4380      // Referência de tensão 3.3V em mV
#define SENSOR_MIN_VOLTAGE 200 // Tensão mínima em mV (0.2V)
#define SENSOR_MAX_VOLTAGE 4700 // Tensão máxima em mV (4.7V)
#define MAX_PRESSURE 10  // Faixa máxima de pressão do MPX5010 (10 kPa)




static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void app_main(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN3, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC1_CHAN3, ADC_ATTEN, &adc1_cali_chan0_handle);


    ESP_ERROR_CHECK(gpio_reset_pin(BUZZER_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT));

    for (;;)
    {

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN3, &adc_raw[0][0]));

        int adc_sum = 0; // Variável para acumular leituras
        int adc_avg = 0; // Variável para armazenar a média

        for (int i = 0; i < NUM_SAMPLES; i++)
        {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN3, &adc_raw[0][0]));
        adc_sum += adc_raw[0][0];
        vTaskDelay(pdMS_TO_TICKS(50)); 
        }

        adc_avg = adc_sum / NUM_SAMPLES; //Média
        int voltage_mV = (adc_avg * V_REF) / MAX_ADC;
        ESP_LOGI(TAG, "voltage_mV: %d", voltage_mV);

        if (do_calibration1_chan0)
        {
             ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_avg, &voltage[0][0]));
             ESP_LOGI(TAG, "Average Voltage: %d mV", voltage[0][0] * 2);
        }

        float Vout = voltage[0][0] * 2; // Tensão medida em mV
        float Vs = 5000; 

        if (Vout >= SENSOR_MIN_VOLTAGE)
        {
            float pressure_kPa = ((Vout / Vs) - 0.04) / 0.09;
            ESP_LOGI(TAG, "Pressure: %.2f kPa", pressure_kPa);
        } else 
        {
                ESP_LOGW(TAG, "Voltage below minimum. Invalid reading.");
        }
        //if (pressure_kPa < 0) pressure_kPa = 0;//PressureRange
        //if (pressure_kPa > MAX_PRESSURE) pressure_kPa = MAX_PRESSURE;
        

        //gpio_set_level(BUZZER_GPIO,1);//Buzzer
        //vTaskDelay(pdMS_TO_TICKS(200));
        //gpio_set_level(BUZZER_GPIO,0);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0)
    {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}