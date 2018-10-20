/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
//added for beacon

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "play_mp3_example.h"




//include "blink.h"
#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
//#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "DataManager.h"
#include "adc.h"
#include "i2c_Accelerometer.h"

#include "ibeacon_demo.h"

#define BT_BLE_COEX_TAG             "BT_BLE_COEX"
#define BT_DEVICE_NAME              "ESP_COEX_A2DP_DEMO"
#define BLE_ADV_NAME                "ESP_COEX_BLE_DEMO"

#include "sdkconfig.h"

//try removing
#define GATTS_TAG "GATTS_AS"
/* Automation IO characteristic */
#define ESP_GATT_UUID_AUTOMATION_IO_SVC             0x1815          /* DIGITAL and ANALOG Service*/
#define ESP_GATT_UUID_AUTOMATION_IO_SV6             0x1816          /* DIGITAL and ANALOG Service*/

#define ESP_GATTS_CHARACTERISTIC_UUID_DIGITAL       0x2A56           //digital
#define ESP_GATTS_CHARACTERISTIC_UUID_ANALOG        0x2A58           //analog
#define ESP_GATTS_CHARACTERISTIC_UUID_AGGREGATE     0x2A5A           //not accurate

#define TEST_DEVICE_NAME            "AUDIBLE BLE"
#define DESCRIPTOR_NAME             0x00
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_AS_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024
#define stack_size     2048
//sets GPIO02 for on board LED
#define BLINK_GPIO 2

//Byte length for sensor notification data
#define ADC_5SEC_BLE_PACKET 11
#define ACCELEROMETER_1MIN_BLE_PACKET 8



void initializeBlink()//void *pvParameter)
{

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


//LED task handler
TaskHandle_t ledHandle;
static uint8_t initialTask = 1;

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
}
/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x60,
    .adv_int_max        = 0x60,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//Profile Structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

//------------------------------requirements for service creation-------------------------------------//
///Declare the static function
static void gatts_profile_accelerometer_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_decimeter_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_NUM_HANDLE_TEST_ACCELEROMETER 6
#define GATTS_NUM_HANDLE_TEST_DECIMETER     6

esp_gatt_char_prop_t accelerometer_property = 0;
esp_gatt_char_prop_t decimeter_property = 1;

uint8_t char1_str[] = {0x11,0x22,0x33};
uint8_t char2_str[] = {0x44,0x55,0x66};

#define PROFILE_NUM 2
#define PROFILE_ACCELEROMETER_APP_ID 0
#define PROFILE_DECIMETER_APP_ID 1

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;
static prepare_type_env_t c_prepare_write_env;
static prepare_type_env_t d_prepare_write_env;

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_ACCELEROMETER_APP_ID] = {        
        .gatts_cb = gatts_profile_accelerometer_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_DECIMETER_APP_ID] = {
        .gatts_cb = gatts_profile_decimeter_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    
};
//----------------------------------------------------------------------------------------------------//
    
/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};
    
esp_attr_value_t GATTS_AS_char1_val =
{
    .attr_max_len = GATTS_AS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

esp_attr_value_t GATTS_AS_char2_val =
{
    .attr_max_len = GATTS_AS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char2_str),
    .attr_value   = char2_str,
};



void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

/*static void ble_init_adv_data(const char *name)
{
    int len = strlen(name);
    uint8_t raw_adv_data[len+5];
    //flag
    raw_adv_data[0] = 2;
    raw_adv_data[1] = ESP_BT_EIR_TYPE_FLAGS;
    raw_adv_data[2] = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    //adv name
    raw_adv_data[3] = len + 1;
    raw_adv_data[4] = ESP_BLE_AD_TYPE_NAME_CMPL;
    for (int i = 0;i < len;i++)
    {
        raw_adv_data[i+5] = *(name++);
    }
    //The length of adv data must be less than 31 bytes
    esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_adv_ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "config raw adv data failed, error code = 0x%x ", raw_adv_ret);
    }
    esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_scan_ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "config raw scan rsp data failed, error code = 0x%x", raw_scan_ret);
    }
}*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:

            
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            #if (IBEACON_MODE == IBEACON_SENDER)
                esp_ble_gap_start_advertising(&ble_adv_params);
            #endif
            break;
    }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
            }
            else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
            

        default:
            break;
    }
}
//TODO: Look into this code for writing event
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }
            
            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
                ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
            
        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_accelerometer_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_id.id.uuid.uuid.uuid16 = ESP_GATT_UUID_AUTOMATION_IO_SVC;
            
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;
#else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= adv_config_flag;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= scan_rsp_config_flag;
            
#endif
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_ACCELEROMETER);
            break;
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            //rsp.attr_value.value[0] = SetSingleBleData;
            //rsp.attr_value.value[1] = bleReadData1;
            //rsp.attr_value.value[2] = bleReadData2;
            //rsp.attr_value.value[3] = bleReadData3;
            
            //SetSingleBleData( &rsp.attr_value.value[0], &rsp.attr_value.value[1], &rsp.attr_value.value[2], &rsp.attr_value.value[3] );
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                
                // param = write value
                //print buffer
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                
                //CODE TO TURN LED ON AND OFF USING BLE WRITE COMMANDS
                //-------------------------------------------------------//
                //if 0x1F  is written, turn on LED
                if ((unsigned int) *param->write.value == 31)
                {
                    gpio_set_level(BLINK_GPIO, 1);
                    initialTask = 0;
                    audio();
                }
                //if 0x00 is written, turn off LED
                else if ((unsigned int) *param->write.value == 0)
                    gpio_set_level(BLINK_GPIO, 0);
                //-------------------------------------------------------//
                
                if (gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        if (accelerometer_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                            ESP_LOGI(GATTS_TAG, "notify enable");
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            //the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_handle,
                                                        sizeof(notify_data), notify_data, false);
                        }
                    }else if (descr_value == 0x0002){
                        if (accelerometer_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(GATTS_TAG, "indicate enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            //the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_handle,
                                                        sizeof(indicate_data), indicate_data, true);
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }
                    
                }
            }
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            example_exec_write_event_env(&a_prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_uuid.uuid.uuid16 = ESP_GATTS_CHARACTERISTIC_UUID_DIGITAL;
            
            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_handle);
            accelerometer_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_err_t add_char1_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_handle, &gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            accelerometer_property,
                                                            &GATTS_AS_char1_val, NULL);
            if (add_char1_ret){
                ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char1_ret);
            }
            break;
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            const uint8_t *prf_char;
            
            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
            if (get_attr_ret == ESP_FAIL){
                ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
            }
            
            ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].service_handle, &gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].descr_uuid,
                                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, DESCRIPTOR_NAME, NULL);
            if (add_descr_ret){
                ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                     param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                     param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
             conn_params.latency = 0;
             conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
             conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
             conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
             ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
             param->connect.conn_id,
             param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
             param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
             gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].conn_id = param->connect.conn_id;
             //start sent the update connection parameters to the peer device.
             esp_ble_gap_update_conn_params(&conn_params);
            
             // test pairing then broadcast
             if (IBEACON_MODE == IBEACON_SENDER)
                beacon();
             break;
             }
             case ESP_GATTS_DISCONNECT_EVT:
                ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
                esp_ble_gap_start_advertising(&adv_params);
                break;
             case ESP_GATTS_CONF_EVT:
                ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
                if (param->conf.status != ESP_GATT_OK){
                    esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
                }
                break;
             case ESP_GATTS_OPEN_EVT:
             case ESP_GATTS_CANCEL_OPEN_EVT:
             case ESP_GATTS_CLOSE_EVT:
             case ESP_GATTS_LISTEN_EVT:
             case ESP_GATTS_CONGEST_EVT:
             default:
                break;
             }
             }

static void gatts_profile_decimeter_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    //static uint8_t initialtask = 1;
    
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_id.id.inst_id = 0x01;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_id.id.uuid.uuid.uuid16 = ESP_GATT_UUID_AUTOMATION_IO_SV6;
            
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;
#else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            
            adv_config_done |= adv_config_flag;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= scan_rsp_config_flag;
            
#endif
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_DECIMETER);
            break;
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            //rsp.attr_value.value[0] = SetSingleBleData;
            //rsp.attr_value.value[1] = bleReadData1;
            //rsp.attr_value.value[2] = bleReadData2;
            //rsp.attr_value.value[3] = bleReadData3;
            
            //SetSingleBleData( &rsp.attr_value.value[0], &rsp.attr_value.value[1], &rsp.attr_value.value[2], &rsp.attr_value.value[3] );
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                
                // param = write value
                //print buffer
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                
                //CODE TO TURN LED ON AND OFF USING WRITE COMMANDS
                //-------------------------------------------------------//
                //if 0x1F  is written, turn on LED
                if ((unsigned int) *param->write.value == 31)
                {
                    gpio_set_level(BLINK_GPIO, 1);
                    initialTask = 0;
                }
                //if 0x00 is written, turn off LED
                else if ((unsigned int) *param->write.value == 0)
                    gpio_set_level(BLINK_GPIO, 0);
                //-------------------------------------------------------//
                
                if (gl_profile_tab[PROFILE_DECIMETER_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        if (decimeter_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                            ESP_LOGI(GATTS_TAG, "notify enable");
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            //the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_handle,
                                                        sizeof(notify_data), notify_data, false);
                        }
                    }else if (descr_value == 0x0002){
                        if (decimeter_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(GATTS_TAG, "indicate enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            //the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_handle,
                                                        sizeof(indicate_data), indicate_data, true);
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }
                    
                }
            }
            example_write_event_env(gatts_if, &c_prepare_write_env, param);
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            example_exec_write_event_env(&c_prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_uuid.uuid.uuid16 = ESP_GATTS_CHARACTERISTIC_UUID_ANALOG;
            
            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_handle);
            decimeter_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_err_t add_char2_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_handle, &gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            decimeter_property,
                                                            &GATTS_AS_char2_val, NULL);
            if (add_char2_ret){
                ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char2_ret);
            }
            break;
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            const uint8_t *prf_char;
            
            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
            if (get_attr_ret == ESP_FAIL){
                ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
            }
            
            ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_DECIMETER_APP_ID].service_handle, &gl_profile_tab[PROFILE_DECIMETER_APP_ID].descr_uuid,
                                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, DESCRIPTOR_NAME, NULL);
            if (add_descr_ret){
                ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            gl_profile_tab[PROFILE_DECIMETER_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                     param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                     param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                     param->connect.conn_id,
                     param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                     param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
           gl_profile_tab[PROFILE_DECIMETER_APP_ID].conn_id = param->connect.conn_id;
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            
            // test pairing then broadcast
            if (IBEACON_MODE == IBEACON_SENDER)
                beacon();
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
            if (param->conf.status != ESP_GATT_OK){
                esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
            }
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void bt_controller_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
}

static void ble_gatts_init(void)
{
    esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    
    ret = esp_ble_gatts_app_register(PROFILE_DECIMETER_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_ACCELEROMETER_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}


//----------------------------------------------------------------------------//
// Function:    getAccelerometerPacketTask()
// Author:      S. Corrales
// Date:        07/30/18
// Inputs:      N/A
// Outputs:     N/A
// Description: task to send Accelerometer data over using Notify capabilities
// Modified:
//----------------------------------------------------------------------------//
//
static void getAccelerometerPacketTask()
{
    uint16_t attr_handle = 0x002a;
    while(1)
    {
        // Delay 1s
        vTaskDelay(100 / portTICK_RATE_MS);
        
        //Getting Data Collector Data
        DATA_COLLECTOR dataCollector;
        memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
        DataManagerGetDataCollector( &dataCollector );
        
        //Accelerometer notify data
        if(dataCollector.arrayFilled0 == DATA_MANAGER_TRUE)
        {
            // Notify client (Note : put last arg to true to send indication)
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].conn_id,
                                        attr_handle,
                                        1, &dataCollector.accelerometerBleData[0],
                                        true);
            dataCollector.arrayFilled0 = DATA_MANAGER_FALSE;
            printf("\nEARMUFF:accel data hex: %x\n\n",dataCollector.accelerometerBleData[0]);
            dataCollector.accelerometerBleData[0]=0;
            
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[0]);
        }
        else if (dataCollector.arrayFilled1 == DATA_MANAGER_TRUE)
        {
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].conn_id,
                                        attr_handle,
                                        1, &dataCollector.accelerometerBleData[1],
                                        true);
            dataCollector.arrayFilled1 = DATA_MANAGER_FALSE;
            printf("\nEARMUFF: accel data hex: %x\n\n",dataCollector.accelerometerBleData[1]);
            dataCollector.accelerometerBleData[1]=0;
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[1]);
        }
         DataManagerSetDataCollector( &dataCollector );
    }
//----------------------------------------------------------------------------//
// Function:    getAccelerometerMinutePacketTask()
// Author:      S. Corrales
}

// Date:        07/30/18
// Inputs:      N/A
// Outputs:     N/A
// Description: task to send Accelerometer data over using Notify capabilities
// Modified:
//----------------------------------------------------------------------------//
//
static void getAccelerometerMinutePacketTask()
{
    uint16_t attr_handle  = gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].char_handle;
    while(1)
    {
        // Delay 1s
        vTaskDelay(100 / portTICK_RATE_MS);
        
        //Getting Data Collector Data
        DATA_COLLECTOR dataCollector;
        memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
        DataManagerGetDataCollector( &dataCollector );
        
        if(dataCollector.arrayFilled0 == DATA_MANAGER_TRUE)
        {
            // Notify client (Note : put last arg to true to send indication)
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].conn_id,
                                        attr_handle,
                                        ACCELEROMETER_1MIN_BLE_PACKET,
                                        &dataCollector.accelerometerBleDataMinute[0],
                                        true);
            dataCollector.arrayFilled0 = DATA_MANAGER_FALSE;
            printf("\nEARMUFF:accel data hex: %" PRIu64 "\n\n",dataCollector.accelerometerBleDataMinute[0]);
            dataCollector.accelerometerBleDataMinute[0]=0;
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[0]);
        }
        else if (dataCollector.arrayFilled1 == DATA_MANAGER_TRUE)
        {
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_ACCELEROMETER_APP_ID].conn_id,
                                        attr_handle,
                                        ACCELEROMETER_1MIN_BLE_PACKET,
                                        &dataCollector.accelerometerBleDataMinute[1],
                                        true);
            dataCollector.arrayFilled1 = DATA_MANAGER_FALSE;
            printf("\nEARMUFF: accel data hex: %" PRIu64 "\n\n",dataCollector.accelerometerBleDataMinute[1]);
            dataCollector.accelerometerBleDataMinute[1]=0;
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[1]);
        }
        DataManagerSetDataCollector( &dataCollector );
    }
}

//----------------------------------------------------------------------------//
// Function:    getAccelerometerMinutePacketTask()
// Author:      S. Corrales
// Date:        07/30/18
// Inputs:      N/A
// Outputs:     N/A
// Description: task to send Accelerometer data over using Notify capabilities
// Modified:
//----------------------------------------------------------------------------//
//
static void getDecimeter5SecPacketTask()
{
    uint16_t attr_handle = gl_profile_tab[PROFILE_DECIMETER_APP_ID].char_handle;
    while(1)
    {
        // Delay 1
        vTaskDelay(100 / portTICK_RATE_MS);
        
        //Getting Data Collector Data
        DATA_COLLECTOR dataCollector;
        memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
        DataManagerGetDataCollector( &dataCollector );

        if(dataCollector.adcArrayFilled0 == DATA_MANAGER_TRUE)
        {
            // Notify client (Note : put last arg to true to send indication)
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_DECIMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_DECIMETER_APP_ID].conn_id,
                                        attr_handle,
                                        ADC_5SEC_BLE_PACKET,
                                        &dataCollector.adcBleData0,
                                        true);
            dataCollector.adcArrayFilled0 = DATA_MANAGER_FALSE;
            //printf("\nEARMUFF:accel data hex: %s \n\n", &dataCollector.adcBleData0);
            memset(dataCollector.adcBleData0, 0, sizeof dataCollector.adcBleData0);
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[0]);
            
        }
        else if (dataCollector.adcArrayFilled1 == DATA_MANAGER_TRUE)
        {
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_DECIMETER_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_DECIMETER_APP_ID].conn_id,
                                        attr_handle,
                                        ADC_5SEC_BLE_PACKET,
                                        &dataCollector.adcBleData4,
                                        true);
            dataCollector.adcArrayFilled1 = DATA_MANAGER_FALSE;
            //printf("\nEARMUFF: accel data hex: %c\n\n",dataCollector.adcBleData4);
            memset(dataCollector.adcBleData4, 0, sizeof(dataCollector.adcBleData4));
            //printf("\n\naccel data; %x\n\n",dataCollector.accelerometerBleData[1]);
        }
        
        DataManagerSetDataCollector( &dataCollector );
    }
}

/* handler for bluetooth stack enabled events */
//static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
//{
//    ESP_LOGD(BT_BLE_COEX_TAG, "%s evt %d", __func__, event);
//    switch (event) {
//        case BT_APP_EVT_STACK_UP: {
//            /* set up bt device name */
//            esp_bt_dev_set_device_name(BT_DEVICE_NAME);
//
//            /* initialize A2DP sink */
//            esp_a2d_register_callback(&bt_app_a2d_cb);
//            esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
//            esp_a2d_sink_init();
//
//            /* initialize AVRCP controller */
//            esp_avrc_ct_init();
//            esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
//
//            /* set discoverable and connectable mode, wait to be connected */
//            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
//            break;
//        }
//        default:
//            ESP_LOGE(BT_BLE_COEX_TAG, "%s unhandled evt %d", __func__, event);
//            break;
//    }
//}

void app_main()
{
   

    //disabled for Zeljko testing
    //---------------------Accelerometer-------------------------------------------//
    //Initialize Data Manager. Creates Data Collector
    DataManagerInit();

   // i2c_example_master_init();
    //xTaskCreate(i2c_accelerometer_task, "i2c_accelerometer_task", 1024 * 4, (void* ) 0, 10, NULL);

    // Start counting task
    static TaskHandle_t accelerometer_task;
    static TaskHandle_t decimeter_task;

    xTaskCreate(&adc,   "adc", stack_size * 4, NULL, 5, NULL);  //configMINIMAL_STACK_SIZE
   // xTaskCreate(&getAccelerometerPacketTask, "counterTask", 2048, NULL, 10, &ct_task);

    //-----------------------------------------------------------------------------//

    
    esp_err_t ret;
    initializeBlink();
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    //---------------------A2DP Sink-------------------------------------------//
//    i2s_config_t i2s_config = {
//#ifdef CONFIG_A2DP_SINK_OUTPUT_INTERNAL_DAC
//        .mode = I2S_MODE_DAC_BUILT_IN,
//#else
//        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
//#endif
//        .sample_rate = 44100,
//        .bits_per_sample = 16,
//        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
//        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
//        .dma_buf_count = 6,
//        .dma_buf_len = 60,                                                      //
//        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
//    };
//
//
//    i2s_driver_install(0, &i2s_config, 0, NULL);
//#ifdef CONFIG_A2DP_SINK_OUTPUT_INTERNAL_DAC
//    i2s_set_pin(0, NULL);
//#else
//    i2s_pin_config_t pin_config = {
//        .bck_io_num = CONFIG_I2S_BCK_PIN,
//        .ws_io_num = CONFIG_I2S_LRCK_PIN,
//        .data_out_num = CONFIG_I2S_DATA_PIN,     // 25
//        .data_in_num = -1                                                       //Not used
//    };
//
//    i2s_set_pin(0, &pin_config);
//#endif

    //-----------------------------------------------------------------------------//
    
    
    // Initialize ble controller
    bt_controller_init();
    
    // create application task
    bt_app_task_start_up();
    
    // Bluetooth device name, connection mode and profile set up
    //bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
   
    //gatt server init
    ble_gatts_init();

    //Set Gap Advertising as receiver while paired
//    if (IBEACON_MODE == IBEACON_RECEIVER)
//        beacon();
    
    //Code for collecting sensor data
   // xTaskCreate(&getAccelerometerMinutePacketTask, "accelerometerTask", 2048*2, NULL, 10, &accelerometer_task);
    xTaskCreate(&getDecimeter5SecPacketTask, "deciemeterTask", 2048*2, NULL, 10, &decimeter_task);

    return;
    
    
}
