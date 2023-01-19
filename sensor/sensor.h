/*******************************************************************************
*
* FILE: 
* 		sensor.h
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/

// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#include "stm32h7xx_hal.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
#endif

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif

/* Project includes */
#if defined( ENGINE_CONTROLLER )
	#include "pressure.h"
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE        ( 0x01 )
#define SENSOR_POLL_CODE        ( 0x02 )

/* Max allowed number of sensors for polling */
#define SENSOR_MAX_NUM_POLL     ( 5    )

#if   defined( FLIGHT_COMPUTER   )
	/* General */
	#define NUM_SENSORS         ( 12   )
	#define IMU_DATA_SIZE       ( 20   )
	#define SENSOR_DATA_SIZE	( 28   )
#elif defined( ENGINE_CONTROLLER )
	/* General */
	#define NUM_SENSORS         ( 10   )
	#define SENSOR_DATA_SIZE    ( 40   )
#else
	#error Board is not compatible with SENSOR module
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Sensor status return codes */
typedef enum 
	{
    SENSOR_OK = 0                ,
	SENSOR_UNRECOGNIZED_OP       ,
	SENSOR_UNSUPPORTED_OP        ,
	SENSOR_IMU_FAIL              ,
	SENSOR_PT_ERROR              ,
	SENSOR_TC_ERROR              ,
	SENSOR_ACCEL_ERROR           ,
    SENSOR_GYRO_ERROR            ,
	SENSOR_MAG_ERROR             ,
	SENSOR_BARO_ERROR            ,
	SENSOR_USB_FAIL              ,
	SENSOR_UNRECOGNIZED_SENSOR_ID,
	SENSOR_POLL_FAIL_TO_START    ,
	SENSOR_POLL_FAIL             ,
    SENSOR_FAIL   
    } SENSOR_STATUS;

/* Sensor poll command codes */
typedef enum
	{
	SENSOR_POLL_START   = 0xF3,
	SENSOR_POLL_REQUEST = 0x51,
	SENSOR_POLL_WAIT    = 0x44,
	SENSOR_POLL_RESUME  = 0xEF,
	SENSOR_POLL_STOP    = 0x74
	} SENSOR_POLL_CMD;

/* Sensor idenification code instance*/
typedef uint8_t SENSOR_ID;

/* Sensor Names/codes */
typedef enum
	{
	#if defined( FLIGHT_COMPUTER )
		SENSOR_ACCX  = 0x00,
		SENSOR_ACCY  = 0x01,
		SENSOR_ACCZ  = 0x02,
		SENSOR_GYROX = 0x03,
		SENSOR_GYROY = 0x04,
		SENSOR_GYROZ = 0x05,
		SENSOR_MAGX  = 0x06,
		SENSOR_MAGY  = 0x07,
		SENSOR_MAGZ  = 0x08,
		SENSOR_IMUT  = 0x09,
		SENSOR_PRES  = 0x0A,
		SENSOR_TEMP  = 0x0B
	#elif defined( ENGINE_CONTROLLER )
		SENSOR_PT0   = 0x00,
		SENSOR_PT1   = 0x01,
		SENSOR_PT2   = 0x02,
		SENSOR_PT3   = 0x03,
		SENSOR_PT4   = 0x04,
		SENSOR_PT5   = 0x05,
		SENSOR_PT6   = 0x06,
		SENSOR_PT7   = 0x07,
		SENSOR_LC    = 0x09,
		SENSOR_TC    = 0x08
	#endif
	} SENSOR_IDS;

/* Sensor Data in integer format */
typedef struct SENSOR_DATA 
	{
	#if defined( FLIGHT_COMPUTER )
		IMU_DATA imu_data;
		float    baro_pressure;
		float    baro_temp;	
	#elif defined( ENGINE_CONTROLLER )
		uint32_t pt_pressures[ NUM_PTS ];
		uint32_t load_cell_force;
		uint32_t tc_temp;
	#endif /* #elif defined( ENGINE_CONTROLLER ) */
	} SENSOR_DATA;

/* Sensor Data sizes and offsets */
typedef struct SENSOR_DATA_SIZE_OFFSETS
	{
	uint8_t offset;  /* Offset of sensor readout in SENSOR_DATA struct  */
	size_t  size;    /* Size of readout in bytes                        */
	} SENSOR_DATA_SIZE_OFFSETS;


/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

#if defined( TERMINAL ) 
/* Initialize the sensor module */
void sensor_init 
	(
	void
	);

/* Execute a sensor subcommand */
SENSOR_STATUS sensor_cmd_execute
	(
	uint8_t subcommand
    );

/* Poll specific sensors on the board */
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID* sensor_ids_ptr  ,
	uint8_t    num_sensors
	);

#endif /* #if defined( TERMINAL )  */

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );


#endif /* SENSOR_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/