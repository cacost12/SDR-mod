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
* 		GPS_Transmit                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Transmits a specified number of bytes over GPS                         *
*                                                                              *
*******************************************************************************/

GPS_STATUS GPS_Transmit
    (
    void*       data_ptr,
    size_t      data_size,
    uint32_t    timeout
    )
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Transmit( &( GPS_HUART ),
                                data_ptr,
                                data_size,
                                timeout );

/* Return HAL status */
if ( gps_status != HAL_TIMEOUT )
    {
    return GPS_OK;
    }
else
    {
    return GPS_TIMEOUT;
    }
    
} /* GPS_Transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		GPS_Receive                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Receive bytes from the GPS                                             *
*                                                                              *
*******************************************************************************/

GPS_STATUS GPS_Receive
    (
    void*       data_ptr,
    size_t      data_size,
    uint32_t    timeout
    )
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Receive byte */
gps_status = HAL_UART_Receive( &( GPS_HUART ),
                                data_ptr,
                                data_size,
                                timeout );

/* Return HAL status */
if ( gps_status != HAL_TIMEOUT )
    {
    return GPS_OK;
    }
else
    {
    return GPS_TIMEOUT;
    }

} /* GPS_Receive */

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
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

GPS_Transmit( configUBX, sizeof(configUBX) / sizeof(uint8_t), HAL_DEFAULT_TIMEOUT );
HAL_Delay(200);

GPS_Transmit( configNMEA, sizeof(configNMEA) / sizeof(uint8_t), HAL_DEFAULT_TIMEOUT );
HAL_Delay(200);

GPS_Transmit( setGNSS, sizeof(setGNSS) / sizeof(uint8_t), HAL_DEFAULT_TIMEOUT );
HAL_Delay(200);

} /* GPS_Load_Config */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		GPS_Get_ID                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return Unique ID of GPS                                                *
*                                                                              *
*******************************************************************************/

GPS_STATUS GPS_Get_ID
    (
    uint8_t* data_ptr;
    )
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
GPS_STATUS gps_status;

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

gps_status = GPS_Transmit( getDeviceID, sizeof( getDeviceID ) / sizeof( uint8_t ), HAL_DEFAULT_TIMEOUT );
gps_status = GPS_Receive( data_ptr, sizeof(data_ptr), HAL_DEFAULT_TIMEOUT );

return gps_status;

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		GPS_Get_NavigatorData                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return GPS Navigator Data                                              *
*                                                                              *
*******************************************************************************/

GPS_STATUS GPS_Get_NavigatorData
    (
    uint8_t* data_ptr
    )
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
GPS_STATUS gps_status;

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

gps_status = GPS_Transmit( getNavigatorData, sizeof( getNavigatorData ) / sizeof( uint8_t ), HAL_DEFAULT_TIMEOUT );
gps_status = GPS_Receive( data_ptr, sizeof(data_ptr), HAL_DEFAULT_TIMEOUT );

return gps_status;

}