/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                               
------------------------------------------------------------------------------*/
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#include "usb.h"
#ifdef VALVE_CONTROLLER
    #include "valve.h"
#endif


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ping                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sends a 1 byte response back to host PC to signal a functioning        * 
*       serial connection                                                      *
*                                                                              *
*******************************************************************************/
void ping
    (
    #ifndef VALVE_CONTROLLER
        void
    #else
        CMD_SOURCE cmd_source
    #endif
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t    response;   /* A0002 Response Code */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
response = PING_RESPONSE_CODE; /* Code specific to board and revision */


/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
#ifndef VALVE_CONTROLLER 
    usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );
#else
    if ( cmd_source == CMD_SOURCE_USB )
        {
        usb_transmit( &response         , 
                      sizeof( response ), 
                      HAL_DEFAULT_TIMEOUT );
        }
    else
        {
        valve_transmit( &response         , 
                        sizeof( response ), 
                        HAL_DEFAULT_TIMEOUT );
        }
#endif

} /* ping */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/