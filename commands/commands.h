/*******************************************************************************
*
* FILE: 
* 		commands.h
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMANDS_H
#define COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* sdec command codes */
#define PING_OP 	   0x01    /* ping command opcode        */
#define CONNECT_OP	   0x02    /* connect command opcode     */
#define IGNITE_OP	   0x20    /* ignition command opcode    */
#define POWER_OP       0x21    /* Power command opcode       */
#define FLASH_OP       0x22    /* flash command opcode       */
#define SENSOR_OP      0x03    /* sensor command opcode      */
#define SOL_OP         0x51    /* solenoid command opcode    */
#define VALVE_OP       0x52    /* Valve command opcode       */
#define DUAL_DEPLOY_OP 0xA0    /* dual-deploy command opcode */

/* Board identifier code */
#if   defined( A0002_REV1 )
	/* Rev 1 */
	#define PING_RESPONSE_CODE    ( 0x04 )
#elif defined( L0002_REV4 )
	/* Rev 4 */
	#define PING_RESPONSE_CODE	  ( 0x03 )
#elif defined( L0005_REV2 )
	/* Rev 2 */
	#define PING_RESPONSE_CODE    ( 0x02 )
#elif defined( A0002_REV2 )
	/* Rev 2 */
	#define PING_RESPONSE_CODE    ( 0x05 )
#elif defined( A0007_REV1 )
	/* Rev 1 */
	#define PING_RESPONSE_CODE    ( 0x06 )
#elif defined ( L0005_REV3 )
	/* Rev 3 */
	#define PING_RESPONSE_CODE    ( 0x07 )
#endif

/* Firmware Identifier Code */
#define FIRMWARE_TERMINAL       ( 0x01 ) /* Terminal Firmware    */
#define FIRMWARE_DATA_LOGGER    ( 0x02 ) /* Data Logger Firmware */
#define FIRMWARE_DUAL_DEPLOY    ( 0x03 ) /* Dual Deploy Firmware */


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Sends a single response byte back to sender */
void ping
	(
	#ifndef VALVE_CONTROLLER
		void
	#else
		CMD_SOURCE cmd_source
	#endif
	);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/