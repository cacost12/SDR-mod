/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
	#include "baro.h"
#endif
#include "usb.h"
#include "sensor.h"
#if defined( ENGINE_CONTROLLER )
	#include "pressure.h"
	#include "loadcell.h"
	#include "temp.h"
#endif


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

#if defined( TERMINAL )
/* Hash table of sensor readout sizes and offsets */
static SENSOR_DATA_SIZE_OFFSETS sensor_size_offsets_table[ NUM_SENSORS ];
#endif


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

#if defined( TERMINAL )
/* Sensor ID to size and pointer mapping */
void static sensor_map
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID    sensor_id      ,
	uint8_t**    sensorid_pptr  ,
	size_t*      sensor_size
	);

/* Extract bytes for export from SENSOR_ID struct */
void static extract_sensor_bytes 
	(
	SENSOR_DATA* sensor_data_ptr      ,
	SENSOR_ID*   sensor_ids_ptr       ,
	uint8_t      num_sensors          ,
	uint8_t*     sensor_data_bytes_ptr,
	uint8_t*     num_sensor_bytes
	);
#endif


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

#if defined( TERMINAL )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_init                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the sensor module                                           *
*                                                                              *
*******************************************************************************/
void sensor_init 
	(
	void
	)
{
/* Setup the sensor id hash table */
#if defined( FLIGHT_COMPUTER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].offset = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].offset = 4;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].offset = 6;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].offset = 8;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].offset = 10; /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].offset = 12; /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].offset = 14; /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].offset = 16; /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].offset = 18; /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].offset = 20; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].offset = 24; /* SENSOR_TEMP  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 2;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].size   = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].size   = 2;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].size   = 2;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].size   = 2;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].size   = 2;  /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].size   = 2;  /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].size   = 2;  /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].size   = 2;  /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].size   = 2;  /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].size   = 4;  /* SENSOR_TEMP  */
#elif defined( ENGINE_CONTROLLER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_PT0  */
	sensor_size_offsets_table[ 1  ].offset = 4;  /* SENSOR_PT1  */
	sensor_size_offsets_table[ 2  ].offset = 8;  /* SENSOR_PT2  */
	sensor_size_offsets_table[ 3  ].offset = 12; /* SENSOR_PT3  */
	sensor_size_offsets_table[ 4  ].offset = 16; /* SENSOR_PT4  */
	sensor_size_offsets_table[ 5  ].offset = 20; /* SENSOR_PT5  */
	sensor_size_offsets_table[ 6  ].offset = 24; /* SENSOR_PT6  */
	sensor_size_offsets_table[ 7  ].offset = 28; /* SENSOR_PT7  */
	sensor_size_offsets_table[ 8  ].offset = 36; /* SENSOR_TC   */
	sensor_size_offsets_table[ 9  ].offset = 32; /* SENSOR_LC   */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 4;  /* SENSOR_PT0  */
	sensor_size_offsets_table[ 1  ].size   = 4;  /* SENSOR_PT1  */
	sensor_size_offsets_table[ 2  ].size   = 4;  /* SENSOR_PT2  */
	sensor_size_offsets_table[ 3  ].size   = 4;  /* SENSOR_PT3  */
	sensor_size_offsets_table[ 4  ].size   = 4;  /* SENSOR_PT4  */
	sensor_size_offsets_table[ 5  ].size   = 4;  /* SENSOR_PT5  */
	sensor_size_offsets_table[ 6  ].size   = 4;  /* SENSOR_PT6  */
	sensor_size_offsets_table[ 7  ].size   = 4;  /* SENSOR_PT7  */
	sensor_size_offsets_table[ 8  ].size   = 4;  /* SENSOR_TC   */
	sensor_size_offsets_table[ 9  ].size   = 4;  /* SENSOR_LC   */
#endif

} /* sensor_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
    uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_status;                         /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
USB_STATUS    usb_status;                            /* USB return codes      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
uint8_t       num_sensor_bytes = SENSOR_DATA_SIZE;   /* Size of data in bytes */
uint8_t       num_sensors;                           /* Number of sensors to 
                                                        use for polling       */
uint8_t       poll_sensors[ SENSOR_MAX_NUM_POLL ];   /* Codes for sensors to
                                                        be polled             */
uint8_t       sensor_poll_cmd;                       /* Command codes used by 
                                                        sensor poll           */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
usb_status      = USB_OK;
sensor_status   = SENSOR_OK;
num_sensors     = 0;
sensor_poll_cmd = 0;
memset( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset( &sensor_data         , 0, sizeof( sensor_data       ) );
memset( &poll_sensors[0]     , 0, sizeof( poll_sensors      ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{
	/*--------------------------------------------------------------------------
	 SENSOR POLL 
	--------------------------------------------------------------------------*/
    case SENSOR_POLL_CODE:
		{
		/* Determine the number of sensors to poll */
		usb_status = usb_receive( &num_sensors, 
		                          sizeof( num_sensors ), 
								  HAL_DEFAULT_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Determine which sensors to poll */
		usb_status = usb_receive( &poll_sensors[0],
		                          num_sensors     , 
                                  HAL_SENSOR_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Receive initiating command code  */
		usb_status = usb_receive( &sensor_poll_cmd,
		                           sizeof( sensor_poll_cmd ),
								   HAL_DEFAULT_TIMEOUT );
		if      ( usb_status      != USB_OK            )
			{
			return SENSOR_USB_FAIL; /* USB error */
			}
		else if ( sensor_poll_cmd != SENSOR_POLL_START )
			{
			/* SDEC fails to initiate sensor poll */
			return SENSOR_POLL_FAIL_TO_START;
			}

		/* Start polling sensors */
		while ( sensor_poll_cmd != SENSOR_POLL_STOP )
			{
			/* Get command code */
			usb_status = usb_receive( &sensor_poll_cmd         ,
			                          sizeof( sensor_poll_cmd ),
									  HAL_DEFAULT_TIMEOUT );
			if ( usb_status != USB_OK ) 
				{
				return SENSOR_USB_FAIL;
				}
			
			/* Execute command */
			switch ( sensor_poll_cmd )
				{

				/* Poll Sensors */
				case SENSOR_POLL_REQUEST:
					{
					sensor_status = sensor_poll( &sensor_data    , 
												 &poll_sensors[0],
												 num_sensors );
					if ( sensor_status != SENSOR_OK )
						{
						return SENSOR_POLL_FAIL;
						}
					else
						{
						/* Copy over sensor data into buffer */
						extract_sensor_bytes( &sensor_data, 
						                      &poll_sensors[0],
											  num_sensors     ,
											  &sensor_data_bytes[0],
											  &num_sensor_bytes );

						/* Transmit sensor bytes back to SDEC */
						usb_transmit( &sensor_data_bytes[0],
						              num_sensor_bytes     ,
									  HAL_SENSOR_TIMEOUT );
								
						break;
						}
					} /* case SENSOR_POLL_REQUEST */

				/* STOP Executtion */
				case SENSOR_POLL_STOP:
					{
					/* Do nothing */
					break;
					} /* case SENSOR_POLL_STOP */

				/* WAIT, Pause execution */
				case SENSOR_POLL_WAIT:
					{
					/* Poll USB port until resume signal arrives */
					while( sensor_poll_cmd != SENSOR_POLL_RESUME )
						{
						usb_receive( &sensor_poll_cmd, 
						             sizeof( sensor_poll_cmd ),
									 HAL_DEFAULT_TIMEOUT );
						}
					break;
					} /* case SENSOR_POLL_WAIT */

				/* Erroneous Command*/
				default:
					{
					Error_Handler();
					break;
					}
				} /* switch( sensor_poll_cmd ) */

			} /* while( sensor_poll_cmd != SENSOR_POLL_STOP ) */
		
		return sensor_status ;
        } /* SENSOR_POLL_CODE */ 

	/*--------------------------------------------------------------------------
	 SENSOR DUMP 
	--------------------------------------------------------------------------*/
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		usb_transmit( &num_sensor_bytes,
                      sizeof( num_sensor_bytes ), 
                      HAL_DEFAULT_TIMEOUT );

		/* Get the sensor readings */
	    sensor_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_status == SENSOR_OK )
			{
			usb_transmit( &sensor_data_bytes[0]      , 
                          sizeof( sensor_data_bytes ), 
                          HAL_SENSOR_TIMEOUT );
			return ( sensor_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/*--------------------------------------------------------------------------
	 UNRECOGNIZED SUBCOMMAND 
	--------------------------------------------------------------------------*/
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */

#endif /* #if  defined( TERMINAL ) */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
#if defined( FLIGHT_COMPUTER )
	IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
	IMU_STATUS      gyro_status;
	IMU_STATUS      mag_status; 
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;
#elif defined( ENGINE_CONTROLLER )
	PRESSURE_STATUS pt_status;              /* Pressure status codes       */
	THERMO_STATUS   tc_status;              /* Thermocouple status codes   */
#endif

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
#if defined( FLIGHT_COMPUTER )
	accel_status = IMU_OK;         
	gyro_status  = IMU_OK;
	mag_status   = IMU_OK; 
	press_status = BARO_OK;           
	temp_status  = BARO_OK;
#elif defined( ENGINE_CONTROLLER )
	pt_status    = PRESSURE_OK;          
	tc_status    = THERMO_OK;        
#endif

/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

/* Poll Sensors  */
#if defined( FLIGHT_COMPUTER )
	/* IMU sensors */
	accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
	gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );
	mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );
	sensor_data_ptr -> imu_data.temp = 0;     // Figure out what to do with this 
											  // readout, temporarily being used 
											  // as struct padding

	/* Baro sensors */
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#elif defined( ENGINE_CONTROLLER )
	/* Pressure Transducers */
	pt_status    = pressure_poll_pts( &( sensor_data_ptr -> pt_pressures[0] ) );

	/* Load cell */
	sensor_data_ptr -> load_cell_force = loadcell_get_reading();

	/* Thermocouple */
	tc_status    = temp_get_temp( &( sensor_data_ptr -> tc_temp ), 
	                              THERMO_HOT_JUNCTION );
#endif


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
#if defined( FLIGHT_COMPUTER )
	if      ( accel_status != IMU_OK )
		{
		return SENSOR_ACCEL_ERROR;
		}
	else if ( gyro_status  != IMU_OK )
		{
		return SENSOR_GYRO_ERROR;
		}
	else if ( mag_status   != IMU_OK )
		{
		return SENSOR_MAG_ERROR;	
		}
	else if ( press_status != BARO_OK ||
			temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}
	else
		{
		return SENSOR_OK;
		}
#elif defined( ENGINE_CONTROLLER )
	if      ( pt_status != PRESSURE_OK )
		{
		return SENSOR_PT_ERROR;
		}
	else if ( tc_status != THERMO_OK   )
		{
		return SENSOR_TC_ERROR;
		}
	else
		{
		return SENSOR_OK;
		}
#endif /* #elif defined( ENGINE_CONTROLLER )*/

} /* sensor_dump */

#if defined( TERMINAL )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_poll                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Poll specific sensors on the board                                     *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr, /* Data Export target               */
	SENSOR_ID*   sensor_ids_ptr , /* Array containing sensor IDS      */
	uint8_t      num_sensors      /* Number of sensors to poll        */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_ID  sensor_id;        /* ID of sensor currently being polled */
SENSOR_ID* sensor_id_ptr;    /* Pointer to sensor id                */

/* Module return codes */
#if   defined( FLIGHT_COMPUTER   )
	IMU_STATUS    imu_status;      /* IMU Module return codes   */ 
	BARO_STATUS   baro_status;     /* Baro module return codes  */
#elif defined( ENGINE_CONTROLLER )
	THERMO_STATUS thermo_status;   /* Thermocouple return codes */
#endif

/* Sensor poll memory to prevent multiple calls to same API function */
#if defined( FLIGHT_COMPUTER )
	bool imu_accel_read;
	bool imu_gyro_read;
	bool imu_mag_read;
#endif

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
sensor_id_ptr     = sensor_ids_ptr;
sensor_id         = *(sensor_id_ptr   );

/* Module return codes */
#if   defined( FLIGHT_COMPUTER   )
	imu_status    = IMU_OK;
	baro_status   = BARO_OK;
#elif defined( ENGINE_CONTROLLER )
	thermo_status = THERMO_OK;
#endif

/* Sensor poll memory */
#if defined( FLIGHT_COMPUTER )
	imu_accel_read = false;
	imu_gyro_read  = false;
	imu_mag_read   = false;
#endif


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Iterate over each sensor readout */
for ( int i = 0; i < num_sensors; ++i )
	{
	
	/* Poll sensor */
	switch ( sensor_id )
		{
		#if defined( FLIGHT_COMPUTER )
			case SENSOR_ACCX:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCY:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCZ:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_GYROX:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROY:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROZ:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_MAGX:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGY:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGZ:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_IMUT:
				{
				sensor_data_ptr -> imu_data.temp = 0;
				break;
				}

			case SENSOR_PRES:
				{
				baro_status = baro_get_temp(     &( sensor_data_ptr -> baro_temp     ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				baro_status = baro_get_pressure( &( sensor_data_ptr -> baro_pressure ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				break;
				}

			case SENSOR_TEMP:
				{
				baro_status = baro_get_temp( &( sensor_data_ptr -> baro_temp ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				break;
				}

		#elif defined( ENGINE_CONTROLLER )
			case SENSOR_PT0:
				{
				sensor_data_ptr -> pt_pressures[0] = pressure_get_pt_reading( PT_NUM0 );
				break;
				}

			case SENSOR_PT1:
				{
				sensor_data_ptr -> pt_pressures[1] = pressure_get_pt_reading( PT_NUM1 );
				break;
				}

			case SENSOR_PT2:
				{
				sensor_data_ptr -> pt_pressures[2] = pressure_get_pt_reading( PT_NUM2 );
				break;
				}

			case SENSOR_PT3:
				{
				sensor_data_ptr -> pt_pressures[3] = pressure_get_pt_reading( PT_NUM3 );
				break;
				}

			case SENSOR_PT4:
				{
				sensor_data_ptr -> pt_pressures[4] = pressure_get_pt_reading( PT_NUM4 );
				break;
				}

			case SENSOR_PT5:
				{
				sensor_data_ptr -> pt_pressures[5] = pressure_get_pt_reading( PT_NUM5 );
				break;
				}

			case SENSOR_PT6:
				{
				sensor_data_ptr -> pt_pressures[6] = pressure_get_pt_reading( PT_NUM6 );
				break;
				}

			case SENSOR_PT7:
				{
				sensor_data_ptr -> pt_pressures[7] = pressure_get_pt_reading( PT_NUM7 );
				break;
				}

			case SENSOR_TC:
				{
				thermo_status = temp_get_temp( &( sensor_data_ptr -> tc_temp ),
				                               THERMO_HOT_JUNCTION );
				if ( thermo_status != THERMO_OK )
					{
					return SENSOR_TC_ERROR;
					}
				break;
				}

			case SENSOR_LC:
				{
				sensor_data_ptr -> load_cell_force = loadcell_get_reading();
				break;
				}

		#endif /* #elif defined( ENGINE_CONTROLLER ) */

		default:
			{
			/* Unrecognized sensor id */
			return SENSOR_UNRECOGNIZED_SENSOR_ID; 
			}
		} /* switch( sensor_id ) */

		/* Go to next sensor */
		sensor_id_ptr++;
		sensor_id        = *(sensor_id_ptr   );

	} /*  while( i < num_sensors ) */

return SENSOR_OK;
} /* sensor_poll */
#endif /* #if defined( TERMINAL ) */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

#if defined( TERMINAL )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_map                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Sensor ID to size and pointer mapping                                  *
*                                                                              *
*******************************************************************************/
void static sensor_map
	(
	SENSOR_DATA* sensor_data_ptr, /* In:  Base pointer to sensor data   */
	SENSOR_ID    sensor_id      , /* In:  Sensor id                    */
	uint8_t**    sensorid_pptr  , /* Out: Pointer to sensor readout in 
	                                      sensor_data_ptr              */
	size_t*      sensor_size_ptr  /* Out: Size of readout in bytes     */
	)
{
/* Lookup sensor offset and size from table */
*sensor_size_ptr = sensor_size_offsets_table[ sensor_id ].size;
*sensorid_pptr   = ( (uint8_t*) sensor_data_ptr ) + 
                   sensor_size_offsets_table[ sensor_id ].offset;

} /*  sensor_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		extract_sensor_bytes                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Extract bytes for export from SENSOR_ID struct                         *
*                                                                              *
*******************************************************************************/
void static extract_sensor_bytes 
	(
	SENSOR_DATA* sensor_data_ptr      , /* In:  Sensor data in struct         */
	SENSOR_ID*   sensor_ids_ptr       , /* In:  Sensor ids                    */
	uint8_t      num_sensors          , /* In:  Number of sensors polled      */
	uint8_t*     sensor_data_bytes_ptr, /* Out: Sensor data in bytes          */
	uint8_t*     num_sensor_bytes       /* Out: Size of output data           */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
uint8_t*   output_ptr;    /* Pointer to data export output                    */
uint8_t*   input_ptr;     /* Pointer to data within SENSOR_ID struct          */
size_t     sensor_size;   /* Size in bytes of current sensor readout          */
SENSOR_ID  sensor_id;     /* Current Sensor ID                                */
SENSOR_ID* sensor_id_ptr; /* Pointer to current sensor ID                     */


/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
output_ptr        = sensor_data_bytes_ptr;
sensor_id_ptr     = sensor_ids_ptr;
sensor_id         = *(sensor_id_ptr);
*num_sensor_bytes = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
for ( uint8_t i = 0; i < num_sensors; ++i )
	{
	/* Get position info of sensor readout */
	sensor_map( sensor_data_ptr, 
	            sensor_id      ,
				&input_ptr      ,
				&sensor_size );

	/* Copy data into output buffer */
	memcpy( output_ptr, input_ptr, sensor_size );

	/* Update size of output */
	*num_sensor_bytes += (uint8_t) sensor_size;

	/* Go to next sensor */ 
	if ( i != ( num_sensors-1) )
		{
		sensor_id_ptr++;
		sensor_id = *(sensor_id_ptr);
		output_ptr += sensor_size;
		}
	}

} /* extract_sensor_bytes */
#endif /* #if defined( TERMINAL )*/


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/