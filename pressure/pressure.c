/*******************************************************************************
*
* FILE: 
* 		pressure.c
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's pressure
*       transducers 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_L0002.h"
#include "pressure.h"
#include "led.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Amplifier gain settings */
static uint8_t           pt_gains[] = { 0, 0, 0, 0, 0, 0, 0, 0 } ; 


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* PT number to GPIO pin butmask mapping */
static inline uint16_t mux_map
	(
    PRESSURE_PT_NUM    pt_num    
    );

/* Sample ADC in polling mode */
static PRESSURE_STATUS sample_adc_poll
	(
    uint16_t     num_samples,
    uint32_t*    psample_buffer
    );

/* Mapping from PT number to amplifer gain GPIO pin bitmask */
static uint16_t amplifier_gain_map
	(
    uint8_t gain_setting 
    );


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_get_pt_reading                                                *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Get a single pressure transducer reading                               *
*                                                                              *
*******************************************************************************/
PRESSURE_STATUS pressure_get_pt_reading 
	(
    PRESSURE_PT_NUM pt_num,          /* PT number to pull         */
	uint32_t*       pt_readout_ptr   /* Pointer to PT output data */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
uint16_t        gain_GPIO_pins_bitmask; /* GPIO pins for amplifier gain       */
uint16_t        mux_GPIO_pins_bitmask;  /* GPIO pins for multiplexor          */
PRESSURE_STATUS pt_status;              /* Status code from adc conversion    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
gain_GPIO_pins_bitmask = amplifier_gain_map( pt_gains[ pt_num ] );
mux_GPIO_pins_bitmask  = mux_map( pt_num );
pt_status              = PRESSURE_OK;


/*------------------------------------------------------------------------------
 Setup mux and gain circuits  
------------------------------------------------------------------------------*/

/* Reset all GPIO */
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    , 
                   PRESSURE_MUX_ALL_PINS |
                   PRESSURE_GAIN_ALL_PINS,
                   GPIO_PIN_RESET );

/* Set MUX */
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT   , 
                   mux_GPIO_pins_bitmask,
                   GPIO_PIN_SET );

/* Set gain */
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    ,
                   gain_GPIO_pins_bitmask,
                   GPIO_PIN_SET );

/*------------------------------------------------------------------------------
 Poll ADC once 
------------------------------------------------------------------------------*/
pt_status = sample_adc_poll( 1, pt_readout_ptr );
if ( pt_status != PRESSURE_OK )
	{
    return pt_status;
    }


/*------------------------------------------------------------------------------
 Reset gain cicuitry and return  
------------------------------------------------------------------------------*/

/* Reset all GPIO */
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    , 
                   PRESSURE_MUX_ALL_PINS |
                   PRESSURE_GAIN_ALL_PINS,
                   GPIO_PIN_RESET );

/* Return pt reading */
return PRESSURE_OK;

} /* pressure_get_pt_reading */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_poll_pts                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Get readings from all pressure transducers                             *
*                                                                              *
*******************************************************************************/
PRESSURE_STATUS pressure_poll_pts
	(
    uint32_t* pPT_readings /* array of size NUM_PTS */ 
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
uint16_t        gain_GPIO_pins_bitmask; /* GPIO pins for amplifier gain       */
uint16_t        mux_GPIO_pins_bitmask;  /* GPIO pins for multiplexor          */
PRESSURE_STATUS pt_status;              /* Status code from adc conversion    */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/

/* Reset all GPIO */
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    , 
				   PRESSURE_MUX_ALL_PINS |
				   PRESSURE_GAIN_ALL_PINS,
				   GPIO_PIN_RESET );

/*------------------------------------------------------------------------------
 Get a reading from each PT 
------------------------------------------------------------------------------*/

for ( uint8_t i = 0; i < NUM_PTS; ++i )
	{
	/*--------------------------------------------------------------------------
	 Setup mux and gain circuits  
	--------------------------------------------------------------------------*/

	/* GPIO pin mapping */
	gain_GPIO_pins_bitmask = amplifier_gain_map( pt_gains[ i ] );
	mux_GPIO_pins_bitmask  = mux_map( i );

	/* Set MUX */
	HAL_GPIO_WritePin( PRESSURE_GPIO_PORT   , 
					   mux_GPIO_pins_bitmask,
					   GPIO_PIN_SET );

	/* Set gain */
	HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    ,
					   gain_GPIO_pins_bitmask,
					   GPIO_PIN_SET );
    HAL_Delay( 1 );

	/*--------------------------------------------------------------------------
	 Poll ADC once 
	--------------------------------------------------------------------------*/
	pt_status = sample_adc_poll( 1, (pPT_readings + i) );
	if ( pt_status != PRESSURE_OK )
		{
		return pt_status;
		}

	/*--------------------------------------------------------------------------
	 Reset gain cicuitry 
	--------------------------------------------------------------------------*/

	/* Reset all GPIO */
	HAL_GPIO_WritePin( PRESSURE_GPIO_PORT    , 
					   PRESSURE_MUX_ALL_PINS |
					   PRESSURE_GAIN_ALL_PINS,
					   GPIO_PIN_RESET );

    }

/* Conversions successful */
return PRESSURE_OK;

} /* pressure_poll_pts */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_set_gain                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set the amplifier gain for a single pressure transducer                *
*                                                                              *
*******************************************************************************/
void pressure_set_gain
	(
    PRESSURE_PT_NUM pt_num,
    uint8_t         gain
    )
{

// TODO: Throw an assert if pt_num is out of range

/* Update gain within global pt_gains array */
pt_gains[ pt_num ] = gain;

} /* pressure_set_gain */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_set_all_gains                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set the amplifier gain for all pressure transducers                    *
*                                                                              *
*******************************************************************************/
void pressure_set_all_gains
	(
    uint8_t gains[] 
    )
{

/* iterate over pressure transducers */
for ( int i = 0; i < NUM_PTS; ++i )
	{
	/* Update gain within global pt_gains array */
    pt_gains[i] = gains[i];
    }

} /* pressure_set_gains */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_get_gain                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Get the gain for a single pressure transducer                          *
*                                                                              *
*******************************************************************************/
uint8_t pressure_get_gain
	(
    PRESSURE_PT_NUM pt_num
    )
{

// TODO: Throw an assert if pt_num is out of range

return pt_gains[ pt_num ];

} /* pressure_get_gain */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pressure_get_all_gains                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Get the gain for all pressure transducers                              *
*                                                                              *
*******************************************************************************/
void pressure_get_all_gains
	(
    uint8_t* pgains 
    )
{

/* Loop over all pressure transducer gains */
for ( int i = 0; i < NUM_PTS; ++i )
	{
    /* Read gain from global gains array */
    *( pgains + i ) = pt_gains[i];
    }

} /* pressure_get_all_gains */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		amplifer_gain_map                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Mapping from pressure transducer number to amplifer gain GPIO pin      *
*       bitmask. ex. gain setting 128 -> PRESSURE_GAIN7_PIN                    * 
*                                                                              *
*******************************************************************************/
static uint16_t amplifier_gain_map
	(
    uint8_t gain_setting 
    )
{
/* PT gain GPIO pins are consecutively numbered, except GPIO_pin 2. Shift bits
   2-7 up one */
uint16_t gain_setting_16b       = (uint16_t) gain_setting;
uint16_t gain_setting_low_bits  = gain_setting_16b &   0x0003;
uint16_t gain_setting_high_bits = gain_setting_16b & (~0x0003);

/* Shift and recombine bits */
gain_setting_high_bits = gain_setting_high_bits << 1; 
return ( gain_setting_high_bits | gain_setting_low_bits );

} /* amplifier_gain_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		mux_map                                                                *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Mapping from pressure transducer number to mutliplexor GPIO pin        *
*       bitmask. ex. PTNUM5 -> 101 -> GPIO_PIN_C | GPIO_PIN_A                  * 
*                                                                              *
*******************************************************************************/
static inline uint16_t mux_map
	(
    PRESSURE_PT_NUM    pt_num    
    )
{
/* Mux pins are adjacent and from the same port. Just shift the ptnum bits up
   to create the bitmask */
return ( (uint16_t) pt_num) << PT_MUX_BITMASK_SHIFT; 

} /* mux_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sample_adc_poll                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Sample the pressure transducer ADC a specified number of times in      *
*       polling mode                                                           *
*                                                                              *
*******************************************************************************/
static PRESSURE_STATUS sample_adc_poll
	(
    uint16_t    num_samples,
    uint32_t*   psample_buffer
    )
{
/* Conversion status indicator */
HAL_StatusTypeDef adc_status;

/* Start the ADC */
HAL_ADC_Start( &( PRESS_ADC ) );

/* Poll ADC */
for ( int i = 0; i < num_samples; ++i )
	{
	/* Wait for end of conversion */
    adc_status = HAL_ADC_PollForConversion( &( PRESS_ADC ), ADC_TIMEOUT );
    if      ( adc_status == HAL_TIMEOUT )
		{
        return PRESSURE_ADC_TIMEOUT;
        }
	else if ( adc_status != HAL_OK      )
		{
        return PRESSURE_ADC_POLL_ERROR;
        }
	else /* No error */
		{
		/* Read the ADC value */
		*(psample_buffer + i) = HAL_ADC_GetValue( &( PRESS_ADC ) ); 
        }
    }

/* Stop the ADC */
HAL_ADC_Stop( &( PRESS_ADC ) );

/* Conversion successful */
return PRESSURE_OK;

} /* sample_adc_poll */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/