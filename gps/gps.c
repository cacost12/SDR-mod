/*******************************************************************************
*
* FILE: 
* 		gps.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the GPS module MAX-M8
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "gps.h"
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		GPS_Load_Config                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initialize UBX protocol, NMEA protocol and activate navigation         *
*       system (Galileo, Glonass, GPS, SBAS, IMES)                             *
*                                                                              *
*******************************************************************************/

void GPS_Load_Config()
{
    HAL_UART_Transmit_DMA( &( GPS_HUART )   ,
                            configUBX,
                            sizeof(configUBX) / sizeof(uint8_t) );
    HAL_Delay(200);

    HAL_UART_Transmit_DMA( &( GPS_HUART )   ,
                            configNMEA,
                            sizeof(configNMEA) / sizeof(uint8_t) );
    HAL_Delay(200);

    HAL_UART_Transmit_DMA( &( GPS_HUART )   ,
                            setGNSS,
                            sizeof(setGNSS) / sizeof(uint8_t) );
    HAL_Delay(200);
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		GPS_Get_ID                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return Unique ID of GPS                                                *
*                                                                              *
*******************************************************************************/

GPS_STATUS GPS_Get_ID()
{
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(   &( GPS_HUART ),
                                                            getDeviceID,
                                                            sizeof( getDeviceID ) / sizeof( uint8_t ));
    if ( hal_status != HAL_TIMEOUT )
        {
        return GPS_OK;
        }
    else
        {
        return GPS_TIMEOUT
        }
}