//
//  data_collector.h
//  
//
//  Created by steven corrales on 7/24/18.
//

#ifndef data_collector_h
#define data_collector_h

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

// Constants
#define DATA_MANAGER_TRUE                                   1
#define DATA_MANAGER_FALSE                                  0
#define DATA_MANAGER_NULL                                   0
#define BLE_PACKET_LENGTH_MAX                               16

typedef struct{

    
    uint8_t bleADCData0[BLE_PACKET_LENGTH_MAX];
    uint8_t bleADCData1[BLE_PACKET_LENGTH_MAX];
//    uint8_t bleADCData2[BLE_PACKET_LENGTH_MAX];
//    uint8_t bleADCData3[BLE_PACKET_LENGTH_MAX];
    
    uint8_t accelerometerBleData[2];
    uint64_t accelerometerBleDataMinute[2];
    
    uint8_t adcBleData0[BLE_PACKET_LENGTH_MAX] ;
    uint8_t adcBleData4[BLE_PACKET_LENGTH_MAX] ;
   
    uint8_t adcArrayFilled0;
    uint8_t adcArrayFilled1;
    
    uint8_t arrayFilled0;
    uint8_t arrayFilled1;
    
    
}DATA_COLLECTOR;

// DataManager Database object
typedef struct
{
    // BLE Object
    struct
    {
        // This is the Extended Mode module data
        DATA_COLLECTOR dataCollector;
    } ble;
// This flag tells you if the data in this structure is valid.
uint8_t isInitialized : 1;
} DATA_MANAGER_DATABASE;


void DataManagerInit( void );

// Public APIs
void SetArrayBleData( uint8_t **Data0, uint8_t **Data1, uint8_t **Data2, uint8_t **Data3, uint16_t startData );
//uint16_t GetArrayBleData( uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint16_t startData );
                         
void SetADCBleData( uint8_t **Data0, uint8_t **Data1, uint8_t **Data2, uint8_t **Data3 );

void GetADCBleData( uint32_t OGData, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3 ,uint8_t counter);

void DataManagerGetDataCollector( DATA_COLLECTOR* value );
void DataManagerSetDataCollector( DATA_COLLECTOR* value );

void GetAccelerometerBleData2Bytes( uint8_t Data0, uint8_t variable);
void SetAccelerometerBleData2Bytes( uint8_t **Data0, uint8_t **Data1,uint8_t **Data2, uint8_t **Data3);

void GetAccelerometerBleDataMinute( uint8_t Data0, uint8_t variable);
void SetAccelerometerBleDataMinute( uint64_t **Data0, uint64_t **Data1,uint64_t **Data2, uint64_t **Data3);

#endif /* data_collector_h */
