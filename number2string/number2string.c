/*******************************************************************************
*
* FILE: 
* 		number2string.c
*
* DESCRIPTION: 
* 		Contains API functions for converting float numbers to string
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "number2string.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		reverseString                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Reverse string created by String converter functions to be in          *
*       the correct order                                                      *
*                                                                              *
*******************************************************************************/

void reverseString
    (
    char* num_ptr, 
    int len
    )
{
for(int i = 0; i<len/2; i++)
    {
    char temp = num_ptr[i];
    num_ptr[i] = num_ptr[len-i-1];
    num_ptr[len-i-1] = temp;
    }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		int2str                                                                *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Function that converts integer to string and returns the size          *
*       length of the string                                                   *
*                                                                              *
*******************************************************************************/

int int2str
    (
    int num, 
    char num_str[]
    )
{
int i = 0;
int i_size = 0;
while ( num/10 != 0 )
    {
    num_str[i] = ( num%10 ) + '0';
    num = num / 10;
    i++;
    i_size++;
    }
num_str[i] = ( num%10 ) + '0';
i_size++;
reverseString( num_str, i_size );
return i_size;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		float2str                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Function that converts float to string                                 *
*                                                                              *
*******************************************************************************/

void float2str
    (
    float f, 
    char* pnum_str, 
    int decimal
    )
{
memset(pnum_str, 0, sizeof(pnum_str));
int integer_f = (int) f;
float fpart = f - (float) integer_f;

fpart = fpart * pow( 10, decimal );

int i = int2str( integer_f, pnum_str );
pnum_str[i] = '.';

int2str( (int) fpart, pnum_str + i + 1 );
}