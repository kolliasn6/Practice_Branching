//
//  data_collector.c
//  
//
//  Created by steven corrales on 7/24/18.
//

#include "DataManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


static DATA_MANAGER_DATABASE* dataManagerDatabase = DATA_MANAGER_NULL;

//----------------------------------------------------------------------------//
// Function:    DataManagerInit
// Author:      S. Corrales
// Date:        7/23/2018
// Inputs:
// Outputs:
// Description: This function initializes the data Base
// Modified:
//----------------------------------------------------------------------------//
void DataManagerInit( void )
{
    // Malloc the database memory only if we haven't already
    if( dataManagerDatabase == DATA_MANAGER_NULL )
    {
        // Allocate the memory for the database
        dataManagerDatabase = (DATA_MANAGER_DATABASE*)malloc( sizeof(DATA_MANAGER_DATABASE) );
        
        // Initialize the entire database to zero
        memset(dataManagerDatabase, 0, sizeof(DATA_MANAGER_DATABASE));

        // Indicate that the database has been initialized...
        dataManagerDatabase->isInitialized = DATA_MANAGER_TRUE;
    }
}

//----------------------------------------------------------------------------//
// Function:    SetADCBleData
// Author:      S. Corrales
// Date:        7/23/2018
// Inputs:
// Outputs:
// Description: This function will send data to BLE controller to broadcast to central devices
// Modified:
//----------------------------------------------------------------------------//

void SetADCBleData( uint8_t **Data0, uint8_t **Data1, uint8_t **Data2, uint8_t **Data3 )
{
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    DataManagerGetDataCollector( &dataCollector );
    

    *Data0 = dataCollector.adcBleData0;
    *Data1 = dataCollector.adcBleData4;
   // *Data2 = dataCollector.adcBleData2;
   // *Data3 = dataCollector.adcBleData3;
    
}

/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    GetADCBleData
 * Author:      S. Corrales
 * Date:        7/23/2018
 * Inputs:
 * Outputs:
 * Description: This function will get data to send to BLE controller to broadcast to central devices
 --------------------------------------------------------------------------------------------------------------------------------/
 */
void GetADCBleData( uint32_t OGData, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3,  uint8_t counter )
{
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    DataManagerGetDataCollector( &dataCollector );

    

    if (counter < 9)
    {
        dataCollector.adcBleData0[counter] = Data0;
        dataCollector.adcBleData0[counter+1] = Data1;

        if (counter == 8)
        {
            dataCollector.adcArrayFilled0 = DATA_MANAGER_TRUE;
            dataCollector.adcBleData0[counter+2] = Data2;
            //printf("\nArray 0, accel data int; %i\n\n",dataCollector.accelerometerBleData[0]);
        }
    }
    else if(counter >= 10)
    {
        if (counter == 18)
        {
            dataCollector.adcArrayFilled1 = DATA_MANAGER_TRUE;
            dataCollector.adcBleData4[counter-8] = Data2;
        }
        counter-=10;
        dataCollector.adcBleData4[counter] = Data0;
        dataCollector.adcBleData4[counter+1] = Data1;

        

    }

    DataManagerSetDataCollector( &dataCollector );
}

/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    GetAccelerometerBleData
 * Author:      S. Corrales
 * Date:        7/28/2018
 * Inputs:
 * Outputs:
 * Description: This function will get accelerometer data to send to BLE controller to broadcast to central devices
 --------------------------------------------------------------------------------------------------------------------------------/
 */
void GetAccelerometerBleData2Bytes( uint8_t Data0, uint8_t variable)
{
    
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );

    if (variable < 7)
    {
        if(variable == 0)
        {
            dataCollector.accelerometerBleData[0] =0;
        }
        
        dataCollector.accelerometerBleData[0] = (dataCollector.accelerometerBleData[0] |(Data0<<variable));
    }
    else if (variable == 7)
    {
        dataCollector.arrayFilled0 = DATA_MANAGER_TRUE;
        dataCollector.accelerometerBleData[0] = (dataCollector.accelerometerBleData[0] |(Data0<<variable));
        //printf("\nArray 0, accel data int; %i\n\n",dataCollector.accelerometerBleData[0]);
    }
    else if (variable == 15)
    {
        variable-=8;
        dataCollector.arrayFilled1 = DATA_MANAGER_TRUE;
        dataCollector.accelerometerBleData[1] = (dataCollector.accelerometerBleData[1] |(Data0<<variable));
        //printf("\n Array 1, accel data int; %i\n\n",dataCollector.accelerometerBleData[1]);
    }
    else if(variable > 7)
    {
        variable-=8;
        dataCollector.accelerometerBleData[1] = (dataCollector.accelerometerBleData[1] |(Data0<<variable));
    
    }
    DataManagerSetDataCollector( &dataCollector );

}

//----------------------------------------------------------------------------//
// Function:    SetAccelerometerBleData
// Author:      S. Corrales
// Date:        7/28/2018
// Inputs:
// Outputs:
// Description: This function will send accelerometer data to BLE controller to broadcast to central devices
// Modified:
//----------------------------------------------------------------------------//

void SetAccelerometerBleData2Bytes( uint8_t **Data0, uint8_t **Data1,uint8_t **Data2, uint8_t **Data3)
{
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );
    
    if(dataCollector.arrayFilled1 == DATA_MANAGER_TRUE)
    {
        *Data0 = dataCollector.accelerometerBleData[0];
        *Data1 = dataCollector.accelerometerBleData[1];
        *Data2 = dataCollector.accelerometerBleData[0];
        *Data3 = dataCollector.accelerometerBleData[1];
    }

}

/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    GetAccelerometerBleDataMinute
 * Author:      S. Corrales
 * Date:        7/28/2018
 * Inputs:
 * Outputs:
 * Description: This function will get accelerometer data to send to BLE controller to broadcast to central devices
 --------------------------------------------------------------------------------------------------------------------------------/
 */
void GetAccelerometerBleDataMinute( uint8_t Data0, uint8_t variable)
{
    
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );
    
    if (variable < 59)
    {
        if(variable == 0)
        {
            dataCollector.accelerometerBleDataMinute[0] =0;
        }
        
        dataCollector.accelerometerBleDataMinute[0] = (dataCollector.accelerometerBleDataMinute[0] |(Data0<<variable));
    }
    else if (variable == 59)
    {
        dataCollector.arrayFilled0 = DATA_MANAGER_TRUE;
        dataCollector.accelerometerBleDataMinute[0] = (dataCollector.accelerometerBleDataMinute[0] |(Data0<<variable));
        //printf("\nArray 0, accel data int; %i\n\n",dataCollector.accelerometerBleData[0]);
    }
    else if (variable == 119)
    {
        variable-=60;
        dataCollector.arrayFilled1 = DATA_MANAGER_TRUE;
        dataCollector.accelerometerBleDataMinute[1] = (dataCollector.accelerometerBleDataMinute[1] |(Data0<<variable));
        //printf("\n Array 1, accel data int; %i\n\n",dataCollector.accelerometerBleData[1]);
    }
    else if(variable > 59)
    {
        variable-=60;
        dataCollector.accelerometerBleDataMinute[1] = (dataCollector.accelerometerBleDataMinute[1] |(Data0<<variable));   
    }
    DataManagerSetDataCollector( &dataCollector );
    
}

//----------------------------------------------------------------------------//
// Function:    SetAccelerometerBleDataMinute
// Author:      S. Corrales
// Date:        7/28/2018
// Inputs:
// Outputs:
// Description: This function will send accelerometer data every minute to BLE controller to broadcast to central devices
// Modified:
//----------------------------------------------------------------------------//

void SetAccelerometerBleDataMinute( uint64_t **Data0, uint64_t **Data1, uint64_t **Data2, uint64_t **Data3)
{
    // Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );
    
    if(dataCollector.arrayFilled1 == DATA_MANAGER_TRUE)
    {
        *Data0 = dataCollector.accelerometerBleData[0];
        *Data1 = dataCollector.accelerometerBleData[1];
        *Data2 = dataCollector.accelerometerBleData[0];
        *Data3 = dataCollector.accelerometerBleData[1];
    }
    
    //DataManagerSetDataCollector( &dataCollector );
    //returnValue= FALSE;
    
    //return returnValue;
}


/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    SetArrayBleData
 * Author:      S. Corrales
 * Date:        7/23/2018
 * Inputs:      indexOfBattery - The index of the battery to set requested data
 * Outputs:
 * Description: This function will send data to BLE controller to broadcast to central devices
 --------------------------------------------------------------------------------------------------------------------------------*/
void SetArrayBleData( uint8_t **Data0, uint8_t **Data1, uint8_t **Data2, uint8_t **Data3, uint16_t startData )
{
    
    //Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );
    
   // uint8_t returnValue;
    
    *Data0 = dataCollector.bleADCData0[startData];
    *Data1 = dataCollector.bleADCData0[startData+1];
    *Data2 = dataCollector.bleADCData0[startData+2];
    *Data3 = dataCollector.bleADCData0[startData+3];
    
    
    DataManagerSetDataCollector( &dataCollector );
    //returnValue= FALSE;
    
    //return returnValue;
}

/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    GetArrayBleData
 * Author:      S. Corrales
 * Date:        7/23/2018
 * Inputs:      indexOfBattery - The index of the battery to set requested data
 * Outputs:
 * Description: This function will send data to BLE controller to broadcast to central devices
 --------------------------------------------------------------------------------------------------------------------------------/
uint16_t GetArrayBleData( uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint16_t startData )
{
    
    //Initialize all volatile data from data collector
    DATA_COLLECTOR dataCollector;
    memset( &dataCollector, 0, sizeof(DATA_COLLECTOR) );
    
    DataManagerGetDataCollector( &dataCollector );
    
    // uint8_t returnValue;
    
    *Data0 = dataCollector.bleADCData0[startData];
    *Data1 = dataCollector.bleADCData0[startData+1];
    *Data2 = dataCollector.bleADCData0[startData+2];
    *Data3 = dataCollector.bleADCData0[startData+3];
    
    
    DataManagerSetDataCollector( &dataCollector );
    //returnValue= FALSE;
    if (startData == 12)
    {
        startData == 0;
        
    }
    else
    {
        startData+=4;
    }
    return startData;
}*/


/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    DataManagerGetChargerExtendedModeModuleData
 * Author:      S. Corrales
 * Date:        3/05/2018
 * Inputs:      value         - The value to use
 * Outputs:     TRUE/FALSE for success or failure
 * Description: This function gets the extended mode module data for the charger object.
 --------------------------------------------------------------------------------------------------------------------------------*/
/*uint8_t*/void DataManagerGetDataCollector( DATA_COLLECTOR* value )
{
    // There is nothing to check so return TRUE
    //uint8_t returnValue = DATA_MANAGER_TRUE;
    
    memcpy( value, &dataManagerDatabase->ble.dataCollector, sizeof(DATA_COLLECTOR) );
    
   // return returnValue;
}

/*--------------------------------------------------------------------------------------------------------------------------------
 * Function:    DataManagerSetChargerExtendedModeModuleData
 * Author:      S. Corrales
 * Date:        03/05/2018
 * Inputs:      value         - The value to use
 * Outputs:     TRUE/FALSE for success or failure
 * Description: This function sets the extended mode module data for the charger object.
 --------------------------------------------------------------------------------------------------------------------------------*/
/*uint8_t*/void DataManagerSetDataCollector( DATA_COLLECTOR* value )
{
    // There is nothing to check so return TRUE
    //uint8_t returnValue = DATA_MANAGER_TRUE;
    
    memcpy( &dataManagerDatabase->ble.dataCollector, value, sizeof(DATA_COLLECTOR) );
    
   // return returnValue;
}
