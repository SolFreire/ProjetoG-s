#include <stdio.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

#define DEVICE_NAME "ESP32_BLE_Weight"
#define GATTS_TAG "GATTS_DEMO"
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

#define GATT_SERVICE_UUID 0x00FF
#define GATT_CHAR_UUID    0xFF01

static float calcular_peso_percentual(float pressao_kPa);
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static uint16_t peso_percentual_handle_table;
static esp_gatt_if_t gatts_if;

float calcular_peso_percentual(float pressao_kPa) {
    float pressao_minima = 1.1;  // Pressão sem peso
    float pressao_maxima = 4.3;  // Média entre 4.2 e 4.4 kPa (5 kg)

    float peso_percentual = (pressao_kPa - pressao_minima) / (pressao_maxima - pressao_minima) * 100;

    if (peso_percentual < 0) {
        peso_percentual = 0;
    } else if (peso_percentual > 100) {
        peso_percentual = 100;
    }

    return peso_percentual;
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_WRITE_EVT) {
        // Exemplo: realizar ações quando um dado for recebido via BLE
    }
}

void create_gatt_service() {
    esp_gatt_srvc_id_t service_id;
    service_id.is_primary = true;
    service_id.id.inst_id = 0x00;
    service_id.id.uuid.len = ESP_UUID_LEN_16;
    service_id.id.uuid.uuid.uuid16 = GATT_SERVICE_UUID;

    esp_ble_gatts_create_service(gatts_if, &service_id, 4);
}

void add_gatt_characteristic() {
    esp_gatt_char_prop_t property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_attr_value_t gatts_demo_char_val = {
        .attr_max_len = sizeof(uint16_t),
        .attr_len = sizeof(uint16_t),
        .attr_value = (uint8_t*)&peso_percentual_handle_table,
    };
    esp_ble_gatts_add_char(peso_percentual_handle_table, &(esp_bt_uuid_t){.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATT_CHAR_UUID}, 
                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, property, &gatts_demo_char_val, NULL);
}

void enviar_peso_percentual_ble(float peso_percentual) {
    uint16_t peso_value = (uint16_t)peso_percentual;
    esp_ble_gatts_set_attr_value(peso_percentual_handle_table, sizeof(uint16_t), (uint8_t *)&peso_value);
    esp_ble_gatts_send_indicate(gatts_if, 0, peso_percentual_handle_table, sizeof(uint16_t), (uint8_t *)&peso_value, false);
}

void app_main(void)
{   
    // BLE initialization
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_profile_event_handler);
    esp_ble_gap_set_device_name(DEVICE_NAME);
    esp_ble_gatts_app_register(0);
    create_gatt_service();
    add_gatt_characteristic();

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

        float Vout = voltage_mV * 2; // Tensão medida em mV
        float Vs = 5000; 

        if (Vout >= SENSOR_MIN_VOLTAGE)
        {
            float pressure_kPa = ((Vout / Vs) - 0.04) / 0.09;
            ESP_LOGI(TAG, "Pressure: %.2f kPa", pressure_kPa);
            
            float peso_percentual = calcular_peso_percentual(pressure_kPa);
            ESP_LOGI(TAG, "Peso percentual: %.2f%%", peso_percentual);

            // Envia o valor do peso percentual via BLE
            enviar_peso_percentual_ble(peso_percentual);

        } else 
        {
                ESP_LOGW(TAG, "Voltage below minimum. Invalid reading.");
        }

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
