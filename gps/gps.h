/*******************************************************************************
*
* FILE: 
* 		gps.h
*
* DESCRIPTION: 
* 		Contains API functions to access data from the GPS module MAX-M8
*
*******************************************************************************/

#ifndef GPS_H
#define GPS_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Defines 
------------------------------------------------------------------------------*/
#define HAL_GPS_DEFAULT_TIMEOUT     1000    /* DEFAULT TIMEOUT FOR GPS UART */
#define GPS_ID_UPC_BYTE             17      /* GPS_ID UBX UPCOMING BYTES */

/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
static uint8_t configUBX[]= {
    #include "bin/configUBX.tbin"
};

static uint8_t configNMEA[] = {
    #include "bin/configNMEA.tbin"
};

//Activation of navigation system: Galileo, Glonass, GPS, SBAS, IMES
static uint8_t setGNSS[]={
    #include "bin/setGNSS.tbin"    
};

static uint8_t getDeviceID[]={
    #include "bin/deviceID.tbin"
};

static uint8_t getNavigatorData[]={
    #include "bin/navigatorData.tbin"
};

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

// GPS Status
typedef enum GPS_STATUS
	{
    GPS_OK          = 0,
    GPS_FAIL           ,
    GPS_UNSUPPORTED_OP ,
    GPS_UNRECOGNIZED_OP,
    GPS_TIMEOUT    
	} GPS_STATUS;



/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

GPS_STATUS GPS_Load_Config();

GPS_STATUS GPS_Transmit
    (
    uint8_t*    data_ptr,
    size_t      data_size,
    uint32_t    timeout
    );

GPS_STATUS GPS_Receive
    (
    uint8_t*    data_ptr,
    size_t      data_size,
    uint32_t    timeout
    );

GPS_STATUS GPS_Get_ID
    (
    uint8_t* data_ptr,
    uint8_t upc_byte
    );

GPS_STATUS GPS_Get_NavigatorData
    (
    uint8_t* data_ptr,
    uint8_t upc_byte
    );

#endif /* GPS_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
