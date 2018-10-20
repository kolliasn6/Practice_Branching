/* ADC1 Example
 
 This example code is in the Public Domain (or CC0 licensed, at your option.)
 
 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "DataManager.h"
#include "timer_group_example_main.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_7;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void adc(void *pvParameter)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    
    //timer code
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    timer_event_t evt;
    uint32_t adc_reading = 0;
    uint8_t counter= 0 ;
    //Continuously sample ADC1
    while (1)
    {
        
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
       // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);

        /* Print information that the timer reported an event */
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        if (evt.type == TEST_WITHOUT_RELOAD)
        {
            
            
            /* Print the timer values passed by event */
            printf("------- EVENT TIME --------\n");
            print_timer_counter(evt.timer_counter_value);
            //uint8_t adc_reading1 = (adc_reading & 0xff000000UL) >> 24;
            //uint8_t adc_reading2 = (adc_reading & 0x00ff0000UL) >> 16;
           
            //hardcode line used to test initial data transmittion
            adc_reading = 0x00000ABCUL;
            
            uint8_t adc_reading3 = (adc_reading & 0x0000ff00UL) >>  8;
            uint8_t adc_reading4 = (adc_reading & 0x000000ffUL)      ;
            uint8_t adc_reading1 = (adc_reading & 0x0000ff00UL) >>8;
            uint8_t adc_reading2 = (adc_reading & 0x000000ffUL)      ;
            
            printf("32 bit: %x \n 1st 8bits: %x \n 2nd 8bits: %x \n 3nd 8bits: %x \n 4nd 8bits: %x \n counter: %x  \n ", adc_reading, adc_reading1,adc_reading2, adc_reading3, adc_reading4,counter );
            
            GetADCBleData( adc_reading, adc_reading1, adc_reading2, ((int) evt.timer_counter_value / TIMER_SCALE), adc_reading4, counter );
            if (counter == 20)
            {
                counter=0;
            }
            else
            {
                counter+=2;
            }
        }
       // printf("32 bit: %x \n 1st 8bits: %x \n 2nd 8bits: %x \n 3nd 8bits: %x \n 4nd 8bits: %x \n ", adc_reading, adc_reading1,adc_reading2, adc_reading3, adc_reading4 );
    }
}



