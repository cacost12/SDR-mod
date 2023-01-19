/*******************************************************************************
*
* FILE: 
* 		sd_card.h
*
* DESCRIPTION: 
* 		Contains API functions to operate the SD card 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */

typedef enum SD_CARD_STATUS{
	SD_CARD_OK 	= 0	  ,
	SD_CARD_OPEN_FAIL ,
	SD_CARD_MOUNT_FAIL,
	SD_CARD_WRITE_FAIL,	
	SD_CARD_TIMEOUT
} SD_CARD_STATUS;

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
SD_CARD_STATUS write_to_sd_card
	(
	char* name,
	char* buffer
	);


#endif /* SD_CARD_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
