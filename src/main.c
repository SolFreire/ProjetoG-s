#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "ble.h"
#include <os/os_mbuf.h>

#define BUZZER_GPIO GPIO_NUM_15
#define SET_BUTTON GPIO_NUM_2
#define ADC1_CHAN3 ADC_CHANNEL_3
#define ADC_ATTEN ADC_ATTEN_DB_12
#define NUM_SAMPLES 10
#define MAX_ADC 4095    // Resolução de 12 bits (0-4095)
#define V_REF 4380      // Referência de tensão 3.3V em mV
#define SENSOR_MIN_VOLTAGE 200 // Tensão mínima em mV (0.2V)
#define SENSOR_MAX_VOLTAGE 4700 // Tensão máxima em mV (4.7V)
#define MAX_PRESSURE 10  // Faixa máxima de pressão do MPX5010 (10 kPa)
#define PRESSURE_THRESHOLD 20 // Limite de peso percentual para ativar o buzzer

const static char *TAG = "GasOn:";

static int adc_raw[2][10];
static int voltage[2][10];
float pressao_maxima = -1;
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

float calcular_peso_percentual(float pressao_kPa) {
    float pressao_minima = 0.3;  
    float peso_percentual = (pressao_kPa - pressao_minima) / (pressao_maxima - pressao_minima) * 100;
    if (peso_percentual < 0) peso_percentual = 0;
    else if (peso_percentual > 100) peso_percentual = 100;
    return peso_percentual;
}

void app_main(void) {
    // Inicialização do BLE
    ble_init();

    // Configuração do ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN3, &config));

    // Configuração do buzzer e botão
    ESP_ERROR_CHECK(gpio_reset_pin(BUZZER_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(gpio_reset_pin(SET_BUTTON));
    ESP_ERROR_CHECK(gpio_set_direction(SET_BUTTON, GPIO_MODE_INPUT));

    // Inicialização da calibração do ADC
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC1_CHAN3, ADC_ATTEN, &adc1_cali_chan0_handle);

    for (;;) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN3, &adc_raw[0][0]));

        int adc_sum = 0;
        int adc_avg = 0;

        // Leitura do ADC
        for (int i = 0; i < NUM_SAMPLES; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN3, &adc_raw[0][0]));
            adc_sum += adc_raw[0][0];
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }

        adc_avg = adc_sum / NUM_SAMPLES;
        int voltage_mV = (adc_avg * V_REF) / MAX_ADC;

        ESP_LOGI(TAG, "voltage_mV: %d", voltage_mV);

        if (voltage_mV >= SENSOR_MIN_VOLTAGE) {
            // Calcular a pressão
            float Vout = voltage_mV * 2; // Tensão medida em mV
            float Vs = 5000; // Tensão de referência do sensor
            float pressure_kPa = ((Vout / Vs) - 0.04) / 0.09;
            ESP_LOGI(TAG, "Pressure: %.2f kPa", pressure_kPa);

            // Verifica o botão e atualiza a pressão máxima, se pressionado
            if (gpio_get_level(SET_BUTTON) == 0) {
                pressao_maxima = pressure_kPa;
                ESP_LOGI(TAG, "Nova pressão máxima capturada: %.2f kPa", pressao_maxima);
            }

            float peso_percentual = calcular_peso_percentual(pressure_kPa);
            ESP_LOGI(TAG, "Peso percentual: %.2f%%", peso_percentual);

            // Ativa o buzzer se o peso percentual estiver abaixo do limite
            if (peso_percentual < PRESSURE_THRESHOLD) {
                gpio_set_level(BUZZER_GPIO, 1); // Liga o buzzer
                ESP_LOGI(TAG, "Alerta: Peso abaixo de 20%%");
            } else {
                gpio_set_level(BUZZER_GPIO, 0); // Desliga o buzzer
            }

            // Envia a notificação BLE apenas com o valor numérico
            char ble_data[10]; // Buffer para os dados BLE
            snprintf(ble_data, sizeof(ble_data), "%.2f", peso_percentual);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(ble_data, strlen(ble_data));
            if (om == NULL) {
                ESP_LOGE(TAG, "Falha ao alocar buffer os_mbuf");
            } else {
                int rc = ble_gatts_notify_custom(conn_handle, encoder_handle, om);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Falha ao enviar notificação BLE: %d", rc);
                } else {
                    ESP_LOGI(TAG, "Notificação enviada via BLE: %s", ble_data);
                }
            }
        } else {
            ESP_LOGI(TAG, "Voltage below minimum. Invalid reading.");
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Atraso de 0,5 segundos
    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
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
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(handle);
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (handle != NULL) {
        adc_cali_delete_scheme_line_fitting(handle);
    }
#endif
}
