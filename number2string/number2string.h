/*******************************************************************************
*
* FILE: 
* 		number2string.h
*
* DESCRIPTION: 
* 		Contains API functions for converting numbers to string
*
*******************************************************************************/


// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NUMBER2STRING_H
#define NUMBER2STRING_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Defines 
------------------------------------------------------------------------------*/

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/*Reverse string created by String converter functions to be in the correct order*/
void reverseString(char* num_ptr, int len);

/*Function that converts integer to string and returns the size length of the string*/
int int2str(int num, char num_str[]);

/*Function that converts float to string*/
void float2str(float f, char* pnum_str, int decimal);

#endif /* NUMBER2STRING_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
