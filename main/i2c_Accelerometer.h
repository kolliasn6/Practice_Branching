
#include <stdio.h>
#include "driver/i2c.h"
#include <math.h>

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1000             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           26               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           25               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define MMA7660_ADDR                       0x4c
#define MMA7660_TIMEOUT c                  500

#define MMA7660_X                          0x00
#define MMA7660_Y                          0x01
#define MMA7660_Z                          0x02
#define MMA7660_TILT                       0x03
#define MMA7660_SRST                       0x04
#define MMA7660_SPCNT                      0x05
#define MMA7660_INTSU                      0x06
#define MMA7660_SHINTX                     0x80
#define MMA7660_SHINTY                     0x40
#define MMA7660_SHINTZ                     0x20
#define MMA7660_GINT                       0x10
#define MMA7660_ASINT                      0x08
#define MMA7660_PDINT                      0x04
#define MMA7660_PLINT                      0x02
#define MMA7660_FBINT                      0x01
#define MMA7660_MODE                       0x07
#define MMA7660_STAND_BY                   0x00 //SET
#define MMA7660_ACTIVE                     0x01
#define MMA7660_AUTO_SLEEP                 0X09
#define MMA7660_SR                         0x08    //  # sample rate register
#define AUTO_SLEEP_120                     0X00    // # 120 sample per second
#define AUTO_SLEEP_64                      0X01
#define AUTO_SLEEP_32                      0X02 //SET
#define AUTO_SLEEP_16                      0X03
#define AUTO_SLEEP_8                       0X04
#define AUTO_SLEEP_4                       0X05
#define AUTO_SLEEP_2                       0X06
#define AUTO_SLEEP_1                       0X07
#define MMA7660_PDET                       0x09
#define MMA7660_PD                         0x0A
#define velocityFallingThreshold           11.0
#define velocityPuttingOn                  3.0
#define velocityStandingThreshold          1.5

#define FALL_DETECT                        1
#define STAND_DETECT                       0



int8_t getCoordinateValue(int8_t x,int8_t y,int8_t z);

int8_t twos_comp(int8_t val, int8_t bits);
static esp_err_t i2c_example_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
static esp_err_t i2c_example_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size);
static esp_err_t i2c_example_master_sensor_test(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l);
void i2c_example_master_init();
// @brief i2c slave initialization
 void i2c_example_slave_init();
// @brief test function to show buffer
static void disp_buf(uint8_t* buf, int len);
void i2c_accelerometer_task(void* arg);




