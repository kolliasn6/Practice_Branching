#include <stdio.h>
#include "driver/i2c.h"
#include <math.h>
#include "i2c_Accelerometer.h"
#include "DataManager.h"
#include "timer_group_example_main.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"





SemaphoreHandle_t print_mux = NULL;
//----------------------------------------------------------------------------//
// Function:    getCoordinateValue
// Author:      S. Corrales
// Date:        6/06/2018
// Inputs:      X,Y,Z coordinates used to determine overall position
// Outputs:     Coordinate location
// Description: This function gets the value of the overall coordinate location
// Modified:
//----------------------------------------------------------------------------//
int8_t getCoordinateValue(int8_t x,int8_t y,int8_t z)
{
    int8_t coordinateValue;
    
    //|a|= Root( x^2 + y^2 + z^2)
    coordinateValue = sqrt((x*x)+(y*y)+(z*z));
    coordinateValue = abs(coordinateValue);

    return coordinateValue;

}


//----------------------------------------------------------------------------//
// Function:    twos_comp
// Author:      D. Gubernat
// Date:        05/29/18
// Inputs:      Accelerometer data register
// Outputs:     2s comp
// Description: This function returns the 2s compliment of the register value
// Modified:
//----------------------------------------------------------------------------//
int8_t twos_comp(int8_t val, int8_t bits)
{
    uint8_t buf=0;

    if ((val & (1 << (bits - 1))) != 0) //# if sign bit is set e.g., 8bit: 128-255
    {
        buf = val - (1 << bits);
    }
    return buf;
}


/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_example_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MMA7660_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_example_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MMA7660_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_example_master_sensor_test(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
   // printf("START MODE SET\n");
    i2c_master_write_byte(cmd, MMA7660_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);  //CHANGED SENSOR ADDRESS
    i2c_master_write_byte(cmd, MMA7660_MODE, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, MMA7660_ACTIVE, ACK_CHECK_DIS);
    //printf("MODE SET\n");
    
    i2c_master_write_byte(cmd, MMA7660_SR, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, AUTO_SLEEP_32, ACK_CHECK_DIS);
   // printf("SAMPLE RATE SET\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    i2c_master_write_byte(cmd, MMA7660_X, ACK_CHECK_DIS);
   // printf("X READ\n");
    
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_master_write_byte(cmd, MMA7660_Y, ACK_CHECK_DIS);
    //printf("X READ\n");
    
    
    i2c_master_read_byte(cmd, data_l, NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave initialization
 */
void i2c_example_slave_init()
{
    int i2c_slave_port = I2C_EXAMPLE_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_EXAMPLE_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_EXAMPLE_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                       I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

void i2c_accelerometer_task(void* arg)
{
    int i = 0;
    int ret;
    
    print_mux = xSemaphoreCreateMutex();
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* coordinateArray = (uint8_t*) malloc(DATA_LENGTH);
    int8_t xAxis, yAxis, zAxis;
    uint8_t sensor_data_h, sensor_data_l;
    double acceleration;
    int cnt = 0;
    int coordinateCounter = 0;
    uint8_t fallDetectCounter=0;
    
    //timer code
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    
    while (1) {
        
        //Initializwe MMA7660fc
        ret = i2c_example_master_sensor_test( I2C_EXAMPLE_MASTER_NUM, &sensor_data_h, &sensor_data_l);

        xSemaphoreTake(print_mux, portMAX_DELAY);
        
        xSemaphoreGive(print_mux);
        //removed for timer testing
       // vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
        
        
        //---------------------------------------------------
        for (i = 0; i < DATA_LENGTH; i++) {
            data[i] = i;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);

        //reads all sensor data
        ret = i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, DATA_LENGTH);

        //MASTER READ FROM SLAVE
        if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
            printf("*********\n");
        } else if (ret == ESP_OK) {
            timer_event_t evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);
            
            /* Print information that the timer reported an event */
            if (evt.type == TEST_WITHOUT_RELOAD) {

                xAxis = twos_comp((int8_t)data_rd[0], 6);
                yAxis = twos_comp((int8_t)data_rd[1], 6);
                zAxis = twos_comp((int8_t)data_rd[2], 6);
                
                /* Print the timer values passed by event */
                printf("------- EVENT TIME --------\n");
                print_timer_counter(evt.timer_counter_value);
           
                //printf("xAxis: %i\n yAxis: %i\n zAxis: %i\n\n",xAxis,yAxis,zAxis);
                
     /*--------------------acceleration detection 2bytes----------------------------------//
                //add values to array
                coordinateArray[coordinateCounter] = getCoordinateValue( xAxis , yAxis, zAxis );
                //printf("\ncoordinate: %i\n", coordinateArray[coordinateCounter]);
      
                //check to see if there has been a big change in acceleration
                if (coordinateCounter!= 0)
                {
                    //printf("coordarray[1]: %i  coordarray[0]: %i\n\n",coordinateArray[coordinateCounter], coordinateArray[coordinateCounter-1]);
                    acceleration = abs(coordinateArray[coordinateCounter] - coordinateArray[coordinateCounter-1]);
                    //printf("acceleration: %f\n velocity Threshold: %f\n\n",acceleration,velocityFallingThreshold );
                    if (acceleration > velocityFallingThreshold)
                    {
                        printf("%i, ******FALLING******\n",fallDetectCounter);
                        GetAccelerometerBleData2Byte( FALL_DETECT, fallDetectCounter);
                    }
                    else
                    {
                        printf("%i, STANDING\n",fallDetectCounter);
                        GetAccelerometerBleData2Byte( STAND_DETECT, fallDetectCounter);
                    }
      
                    if (fallDetectCounter !=15)
                    {
                        fallDetectCounter++;
                    }
                    else
                    {
                        fallDetectCounter=0;
                    }
                }
                coordinateCounter+=1;
    //------------------------------------------------------------------------------*/
    //--------------------acceleration detection 1 minute of data----------------------------------//
            //add values to array
            coordinateArray[coordinateCounter] = getCoordinateValue( xAxis , yAxis, zAxis );
            //printf("\ncoordinate: %i\n", coordinateArray[coordinateCounter]);
            
                //check to see if there has been a big change in acceleration
                if (coordinateCounter!= 0)
                {
                    //printf("coordarray[1]: %i  coordarray[0]: %i\n\n",coordinateArray[coordinateCounter], coordinateArray[coordinateCounter-1]);
                    acceleration = abs(coordinateArray[coordinateCounter] - coordinateArray[coordinateCounter-1]);
                    //printf("acceleration: %f\n velocity Threshold: %f\n\n",acceleration,velocityFallingThreshold );
                    if (acceleration > velocityFallingThreshold)
                    {
                        printf("%i, ******FALLING******\n",fallDetectCounter);
                        GetAccelerometerBleDataMinute( FALL_DETECT, fallDetectCounter);
                    }
                    else
                    {
                        printf("%i, STANDING\n",fallDetectCounter);
                        GetAccelerometerBleDataMinute( STAND_DETECT, fallDetectCounter);
                    }
                    
                    if (fallDetectCounter !=119)
                    {
                        fallDetectCounter++;
                    }
                    else
                    {
                        fallDetectCounter=0;
                    }
                }
                coordinateCounter+=1;
    //------------------------------------------------------------------------------//
            }
        } else {
            printf("%s: Master read slave error, IO not connected...\n", esp_err_to_name(ret));
        }

        xSemaphoreGive(print_mux);
        //vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
        //---------------------------------------------------
        int size;
        for (i = 0; i < DATA_LENGTH; i++) {
            data_wr[i] = i + 10;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        
        
        
        xSemaphoreGive(print_mux);
       // vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }
}

//Code to Run I2c on its own
/*
void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    i2c_example_slave_init();
    i2c_example_master_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void* ) 1, 10, NULL);

}
*/
