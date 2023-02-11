/*******************************************************************************
*
* FILE: 
* 		sd_card.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER  )
	#include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION    )
	#include "sdr_pin_defines_A0005.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sd_card.h"
#include "fatfs.h"
#include <string.h>


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		write_to_sd_card                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		write a file to SD card						                           *
*                                                                              *
*******************************************************************************/

SD_CARD_STATUS write_to_sd_card(char* filename , char* buffer_string_ptr)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FATFS FatFs_Handle;
FIL file_handle;
FRESULT file_result;
uint32_t bytes_written;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Mount the SD Card */
file_result = f_mount(&FatFs_Handle, "", 1);
if ( file_result != FR_OK )
	{
	return SD_CARD_MOUNT_FAIL;
	}
/*Create a file in SD Card with name given*/
file_result = f_open(&file_handle, strcat(filename, ".txt"), FA_OPEN_ALWAYS | FA_WRITE);
if ( file_result != FR_OK )
	{
	return SD_CARD_OPEN_FAIL;
	}
/*Write to the file*/
file_result = f_write(&file_handle, buffer_string_ptr, strlen(buffer_string_ptr), &bytes_written);
if ( (bytes_written == 0) || (file_result != FR_OK) )
	{
	return SD_CARD_WRITE_FAIL;
	}

/*Close file*/
f_close(&file_handle);
return SD_CARD_OK;
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
