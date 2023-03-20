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



/*------------------------------------------------------------------------------
 Defines subcommand codes
------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
static const uint8_t configUBX[]= {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x9A,0x79};

static const uint8_t configNMEA[] = {0xB5,0x62,0x06,0x17,0x14,0x00,0x00,0x41,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x75,0x57};

//Activation of navigation system: Galileo, Glonass, GPS, SBAS, IMES
static const uint8_t setGNSS[]={0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x20,0x04,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,0xDF,0xFB};

static const uint8_t getDeviceID[]={0xB5,0x62,0x27,0x03,0x00,0x00,0x2A,0xA5};

static const uint8_t getNavigatorData[]={0xB5,0x62,0x01,0x21,0x00,0x00,0x22,0x67};

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
	} IMU_STATUS;



/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

void GPS_Load_Config();

GPS_STATUS GPS_Transmit
    (
    void*       data_ptr,
    size_t      data_size,
    uint32_t    timeout
    );

GPS_STATUS GPS_Receive
    (
    void*       data_ptr,
    size_t      data_size,
    uint32_t    timeout
    );

GPS_STATUS GPS_Get_ID
    (
    uint8_t* data_ptr;
    );

GPS_STATUS GPS_Get_NavigatorData
    (
    uint8_t* data_ptr
    );

#endif /* GPS_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
