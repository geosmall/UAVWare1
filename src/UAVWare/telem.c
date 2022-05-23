#include "telem.h"
#include "oav_defines.h"
#include "../uvos/uvos_time.h"
#include "../uvos/uvos_usart.h"
#include "../libraries/mavlink/common/mavlink.h"
#include "../libraries/lwrb/lwrb.h"
#include "storage.h"

#include "stm32yyxx_ll.h"

/* USART telem reception setup */
#define RX_BUFFER_SIZE  128
// static uint8_t circular_buffer_storage[RX_BUFFER_SIZE] = {0};
// static cbuf_handle_t _buf_handle = NULL;
static uint8_t usart_rx_rb_data[RX_BUFFER_SIZE] = {0};
static lwrb_t usart_rx_rb;
__IO uint32_t uwNbReceivedChars = 0;

#define TX_BUFFER_SIZE  512
static uint8_t tx_byte_buffer[TX_BUFFER_SIZE] = {0}; //A byte buffer that will be sent from the serial port.

static uint8_t usart_tx_rb_data[TX_BUFFER_SIZE] = {0};
static lwrb_t usart_tx_rb;

mavlink_message_t mvl_tx_message; //A special MAVLink message data structure.
mavlink_message_t mvl_rx_message;
mavlink_status_t mvl_rx_status;
const uint8_t MAV_COMPONENT_ID = 1; //Component ID and System ID identify us to QGroundControl
const uint8_t MAV_SYSTEM_ID = 1;
const uint8_t MAV_CHANNEL = MAVLINK_COMM_0;  //MAVLink channel 1 appears to be required at least for Blue Robotics QGC

const uint32_t hb_interval = 1000;					// heartbeat interval in milliseconds - 1 second
uint32_t t_last_hb = 0;
const uint32_t sys_stat_interval = 100;			// 10 system status messages per second
uint32_t t_last_sys_stat = 0;
int16_t sys_stat_count = 0;
uint8_t mvl_armed = 0;
uint8_t mvl_packet_received = 0;

static const char * config_base_addr = &config;
int paramListPartIndicator = -1;

mavlink_param_set_t set;
char * parameter_id;
mavlink_message_t msg;


void MVL_Send_Param_UINT8( const uint8_t paramValue, const char * parameterName, const int listsize, const int index );
void MVL_Send_Param_INT32( const int32_t paramValue, const char * parameterName, const int listsize, const int index );
void MVL_Send_Param_UINT32( const uint32_t paramValue, const char * parameterName, const int listsize, const int index );
void MVL_Send_Param_FLOAT( const float paramValue_flt, const char * parameterName, const int listsize, const int index );


void telem_byte_received_callback( uint8_t c )
{
	// Put received char from usart driver callback in our buffer
	// circular_buf_putc( _buf_handle, c );
	lwrb_write( &usart_rx_rb, &c, 1 );
}

void telem_init( void )
{

	// Initialize rx circular buffer
	// _buf_handle = circular_buf_init( circular_buffer_storage , RX_BUFFER_SIZE );
	lwrb_init( &usart_rx_rb, usart_rx_rb_data, sizeof( usart_rx_rb_data ) );

	// Initialize tx lw ring buffer
	lwrb_init( &usart_tx_rb, usart_tx_rb_data, sizeof( usart_tx_rb_data ) );

	// Register USART RXNE callback
	UVOS_TELEM_RegisterTelemUSARTCallback( telem_byte_received_callback );

	// Initialize UVOS telem, passing tx lw structure and ring buffer
	UVOS_TELEM_init( &usart_tx_rb, usart_tx_rb_data );

	// Enable telem RXNE interrupt
	UVOS_TELEM_enable_rxne_it();

}

void telem_update( void )
{

	uint8_t rxbyte;

	while ( lwrb_get_full( &usart_rx_rb ) ) {
		lwrb_read( &usart_rx_rb, &rxbyte, 1 );
		mvl_packet_received = mavlink_parse_char( MAV_CHANNEL, rxbyte,
		                      &mvl_rx_message,
		                      &mvl_rx_status );
		if ( mvl_packet_received ) {
			break;
		}
	}

	/* ====================== received MAVLink Message Handling ==================
	 *  If a full incoming MAVLink message is received, AND the message
	 *  came from the GCS (System ID 255), we handle it here.
	 *
	 *  In this code we:
	 *  -Respond to initial messages sent from QGC to the vehicle when it first connects, including:
	 *    -A parameter list request. We send an arbitary float parameter.
	 *    -A request for "mission items." We say we have none.
	 *    -A request for autopilot capabilities and version. We make some things up.
	 *  -Take action on manual control joystick messages -- retransmitted back to QGC for viewing in MAVLink inspector
	 *  -Arm or disarm the vehicle and acknowledge arm/disarm status based on incoming command messages.
	 *  There may be more messages you want to handle.
	 *  The Message IDs below are defined in individual message's .h files.
	 *  For example: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_manual_control.h#L4
	 *  You can find message numbers and field descriptions at https://mavlink.io/en/messages/common.html
	 * ==================================================================== */
	if ( ( mvl_packet_received ) && ( 255 == mvl_rx_message.sysid ) ) {

		LL_GPIO_SetOutputPin( testPin2_GPIO_Port, testPin2_Pin );

		mvl_packet_received = 0; //reset the "packet received" flag
		switch ( mvl_rx_message.msgid ) {
		case MAVLINK_MSG_ID_MANUAL_CONTROL: //#69 https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
			MVL_Handle_Manual_Control( &mvl_rx_message );
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21 https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
			MVL_Handle_Param_Request_List( &mvl_rx_message );
			break;
		case MAVLINK_MSG_ID_PARAM_SET: //#23 https://mavlink.io/en/messages/common.html#PARAM_SET
			mavlink_msg_param_set_decode( &mvl_rx_message, &set );
			uint32_t index = get_param_index_from_id( ( char * ) set.param_id );
			telem_set_parameter( index, &set );
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG: //#76 https://mavlink.io/en/messages/common.html#COMMAND_LONG
			MVL_Handle_Command_Long( &mvl_rx_message );
			break;
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //#43 https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST
			MVL_Handle_Mission_Request_List( &mvl_rx_message );
			break;
		default:
			break;
		}

		LL_GPIO_ResetOutputPin( testPin2_GPIO_Port, testPin2_Pin );

	}

}

void telem_set_parameter( uint32_t index, mavlink_param_set_t * param_set )
{
	MAV_PARAM_TYPE param_type = get_param_mav_type( index );
	mavlink_param_union_t u_param;
	u_param.param_float = param_set->param_value;
	if ( index >= 0 ) {
		// For read-only  respond with new val followed by original val
		// https://ardupilot.org/dev/docs/mavlink-get-set-params.html
		if ( param_type == MAV_PARAM_TYPE_UINT8 ) {
			uint8_t * val_ptr_U8 = ( uint8_t * )( config_base_addr + get_param_offset( index ) );
			if ( get_param_read_write( index ) ) {
				*val_ptr_U8 = u_param.param_uint8; // update the config struct member
			} else {
				MVL_Send_Param_UINT8( u_param.param_uint8, get_param_name( index ), get_sizeof_param_index(), index  );
			}
			MVL_Send_Param_UINT8( *val_ptr_U8, get_param_name( index ), get_sizeof_param_index(), index  );
		}	else if ( param_type == MAV_PARAM_TYPE_INT32 ) {
			int32_t * val_ptr_32 = ( uint32_t * )( config_base_addr + get_param_offset( index ) );
			if ( get_param_read_write( index ) ) {
				*val_ptr_32 = u_param.param_int32; // update the config struct member
			} else {
				MVL_Send_Param_INT32( u_param.param_int32, get_param_name( index ), get_sizeof_param_index(), index  );
			}
			MVL_Send_Param_INT32( *val_ptr_32, get_param_name( index ), get_sizeof_param_index(), index  );
		}	else if ( param_type == MAV_PARAM_TYPE_UINT32 ) {
			uint32_t * val_ptr_U32 = ( uint32_t * )( config_base_addr + get_param_offset( index ) );
			if ( get_param_read_write( index ) ) {
				*val_ptr_U32 = u_param.param_uint32; // update the config struct member
			} else {
				MVL_Send_Param_UINT32( u_param.param_uint32, get_param_name( index ), get_sizeof_param_index(), index  );
			}
			MVL_Send_Param_UINT32( *val_ptr_U32, get_param_name( index ), get_sizeof_param_index(), index  );
		} else if ( param_type == MAV_PARAM_TYPE_REAL32 ) {
			float * val_ptr_F = ( uint32_t * )( config_base_addr + get_param_offset( index ) );
			if ( get_param_read_write( index ) ) {
				*val_ptr_F = u_param.param_float; // update the config struct member
			} else {
				MVL_Send_Param_FLOAT( u_param.param_float, get_param_name( index ), get_sizeof_param_index(), index  );
			}
			MVL_Send_Param_FLOAT( *val_ptr_F, get_param_name( index ), get_sizeof_param_index(), index  );
		}
	}
}

/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Onboard parameter value
 * @param param_type  Onboard parameter type.
 * @param param_count  Total number of onboard parameters
 * @param param_index  Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 *
static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
*/

// uint8_t parameterType = MAV_PARAM_TYPE_REAL32;

void telem_send_parameter_float( float param_val, const char * param_id, uint32_t listsize, int index )
{
	mavlink_msg_param_value_pack( MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, param_id, param_val, MAV_PARAM_TYPE_REAL32, listsize, index );
	uint16_t tx_buflen = mavlink_msg_to_send_buffer( tx_byte_buffer, &msg );
	UVOS_TELEM_puts( tx_byte_buffer, tx_buflen );
}

/* Let QGroundControl know a vehicle is present, we send a heartbeat ~ every 1s */
void telem_heartbeat( void )
{
	//#0 HEARTBEAT https://mavlink.io/en/messages/common.html#HEARTBEAT
	mavlink_heartbeat_t mvl_hb; //struct with user fields: uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status;
	mvl_hb.type = MAV_TYPE_SUBMARINE; //My vehicle is an underwater ROV. Change as appropriate. See: https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L69
	mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC; //See https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L40
	mvl_hb.system_status = MAV_STATE_ACTIVE;
	if ( mvl_armed ) {
		mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
	} else {
		mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
	}
	mvl_hb.base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; //I always use CUSTOM_MODE_ENABLED
	mvl_hb.custom_mode = 0xdeadbeef; //custom mode, can be anything, I guess
	mavlink_msg_heartbeat_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
	                                   &mvl_tx_message, &mvl_hb );
	MVL_Transmit_Message( &mvl_tx_message );
}

void telem_system_stats( void )
{
	//We'll make up some fake periodic data to feed to the QGroundControl widget
	float pfreq = 0.2; //slowly varying
	float phaserad = 2 * PI_F * pfreq * millis() / 1000.0;

	//We'll send a varying voltage signal that ramps from 10000 to 16,000 mV over 60 sys_stat_intervals... 10-16V on the QGC widget
	int16_t angle_as_mV = sys_stat_count * 100 + 10000;
	sys_stat_count += 1;
	sys_stat_count %= 60;
	//I often use the mavlink_<message name>_pack_chan() functions that
	//accept each field as an argument instead of the mavlink_<message name>_encode() that
	//accepts a struct. They should save some memory to skip the extra
	//message struct, but I think setting each field by name in a demo code is easier to follow.

	mavlink_sys_status_t mvl_sys_stat; //#1 SYS_STATUS https://mavlink.io/en/messages/common.html#SYS_STATUS
	mvl_sys_stat.onboard_control_sensors_present = 0; //To set these, consult https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR
	mvl_sys_stat.onboard_control_sensors_enabled = 0;
	mvl_sys_stat.onboard_control_sensors_health = 0;
	mvl_sys_stat.load = 0;
	mvl_sys_stat.voltage_battery = angle_as_mV; //the only non-trivial telemetry we're sending, shows up several places in QGC
	mvl_sys_stat.current_battery = -1;
	mvl_sys_stat.battery_remaining = -1;
	mvl_sys_stat.drop_rate_comm = 0;
	mvl_sys_stat.errors_comm = 0;
	mvl_sys_stat.errors_count1 = 0;
	mvl_sys_stat.errors_count2 = 0;
	mvl_sys_stat.errors_count3 = 0;
	mvl_sys_stat.errors_count4 = 0;

	//We'll also send an attitude quaternion to display something in the QGC
	//roll/pitch widget and in the compass.
	//The code below results in a gentle spherical rocking in the QGC roll/pitch widget
	//and a continuous rotation of the displayed heading.

	float maxang = 0.0873; // about five degrees
	float roll = maxang * sin( phaserad );
	float pitch = maxang * cos( phaserad );
	float yaw = phaserad;

	//Quaternion conversion taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
	float cy = cos( yaw * 0.5 );
	float sy = sin( yaw * 0.5 );
	float cr = cos( roll * 0.5 );
	float sr = sin( roll * 0.5 );
	float cp = cos( pitch * 0.5 );
	float sp = sin( pitch * 0.5 );

	mavlink_attitude_quaternion_t mvl_att_quat; //#31 ATTITUDE_QUATERNION https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION
	mvl_att_quat.time_boot_ms = millis();
	mvl_att_quat.q1 = cy * cr * cp + sy * sr * sp; //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
	mvl_att_quat.q2 = cy * sr * cp - sy * cr * sp;
	mvl_att_quat.q3 = cy * cr * sp + sy * sr * cp;
	mvl_att_quat.q4 = sy * cr * cp - cy * sr * sp;

	mvl_att_quat.rollspeed = 2 * PI_F * pfreq * maxang * cos( phaserad ); //d/dt A*sin(2*pi*f*t) = 2*pi*f*A*cos(2*pi*f*t)
	mvl_att_quat.pitchspeed = -2 * PI_F * pfreq * maxang * sin( phaserad );
	mvl_att_quat.yawspeed = 2 * PI_F * pfreq;

	mavlink_msg_sys_status_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
	                                    &mvl_tx_message, &mvl_sys_stat );
	MVL_Transmit_Message( &mvl_tx_message );
	mavlink_msg_attitude_quaternion_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
	    &mvl_tx_message, &mvl_att_quat );
	MVL_Transmit_Message( &mvl_tx_message );
}

#define PARAM_MODULUS (6)

void telem_send_queued_params( void )
{

#if 0

	uint32_t num_params, param_list_part_count_max, mod;
	volatile uint32_t remain;

	num_params = get_sizeof_param_index();
	mod = PARAM_MODULUS;

	param_list_part_count_max = num_params / mod;
	remain = num_params - mod * ( num_params / mod );

	if ( remain > 0 ) param_list_part_count_max++;

	if ( paramListPartIndicator >= 0 && paramListPartIndicator <= param_list_part_count_max ) {

		uint32_t index_start, index_end;

		index_start = paramListPartIndicator * PARAM_MODULUS;
		if ( paramListPartIndicator <= param_list_part_count_max ) {
			index_end = index_start + ( PARAM_MODULUS - 1 );
		} else {
			index_end = index_start + ( remain - 1 );
		}

		uintptr_t config_base = &config;
		for ( uint32_t i = index_start; i <= index_end; i++ ) {
			MAV_PARAM_TYPE mav_param_type = get_param_mav_type( i );
			uintptr_t addr = config_base + get_param_offset( i );
			if ( mav_param_type == MAV_PARAM_TYPE_UINT8 ) {
				// void MVL_Send_Param_UINT8( const uint8_t paramValue, const char * parameterName, const int listsize, const int index );
				uint8_t paramValue8 = *( uint8_t * ) addr;
				MVL_Send_Param_UINT8( paramValue8, get_param_name( i ), num_params, i );
			} else if  ( mav_param_type == MAV_PARAM_TYPE_UINT32 ) {
				uint32_t paramValue32 = *( uint32_t * ) addr;
				MVL_Send_Param_UINT32( paramValue32, get_param_name( i ), num_params, i );
			} else {
				float paramValueF = *( float * ) addr;
				MVL_Send_Param_FLOAT( paramValueF, get_param_name( i ), num_params, i );
			}
		}
		paramListPartIndicator++;
	} else {
		paramListPartIndicator = -1;
	}

#else

	uint32_t num_params = get_sizeof_param_index();
	uint32_t i;
	uintptr_t base = &config;
	uintptr_t addr;

	if ( paramListPartIndicator >= 0 && paramListPartIndicator <= 6 ) {
		if ( paramListPartIndicator == 0 ) {
			telem_send_queued_param_by_index( 0 );
			telem_send_queued_param_by_index( 1 );
			telem_send_queued_param_by_index( 2 );
			telem_send_queued_param_by_index( 3 );
			telem_send_queued_param_by_index( 4 );
			telem_send_queued_param_by_index( 5 );
			telem_send_queued_param_by_index( 6 );
		} else if ( paramListPartIndicator == 1 ) {
			telem_send_queued_param_by_index( 7 );
			telem_send_queued_param_by_index( 8 );
			telem_send_queued_param_by_index( 9 );
			telem_send_queued_param_by_index( 10 );
		} else if ( paramListPartIndicator == 2 ) {
			telem_send_queued_param_by_index( 11 );
			telem_send_queued_param_by_index( 12 );
			telem_send_queued_param_by_index( 13 );
			telem_send_queued_param_by_index( 14 );
			telem_send_queued_param_by_index( 15 );
			telem_send_queued_param_by_index( 16 );
		} else if ( paramListPartIndicator == 3 ) {
			telem_send_queued_param_by_index( 17 );
			telem_send_queued_param_by_index( 18 );
			telem_send_queued_param_by_index( 19 );
			telem_send_queued_param_by_index( 20 );
		} else if ( paramListPartIndicator == 4 ) {
			telem_send_queued_param_by_index( 21 );
			telem_send_queued_param_by_index( 22 );
			telem_send_queued_param_by_index( 23 );
			telem_send_queued_param_by_index( 24 );
			telem_send_queued_param_by_index( 25 );
			telem_send_queued_param_by_index( 26 );
			telem_send_queued_param_by_index( 27 );
			telem_send_queued_param_by_index( 28 );
		} else if ( paramListPartIndicator == 5 ) {
			telem_send_queued_param_by_index( 29 );
			telem_send_queued_param_by_index( 30 );
			telem_send_queued_param_by_index( 31 );
			telem_send_queued_param_by_index( 32 );
			telem_send_queued_param_by_index( 33 );
			telem_send_queued_param_by_index( 34 );
		} else if ( paramListPartIndicator == 6 ) {
			telem_send_queued_param_by_index( 35 );
			telem_send_queued_param_by_index( 36 );
			telem_send_queued_param_by_index( 37 );
			telem_send_queued_param_by_index( 38 );
		}
		paramListPartIndicator++;
	} else {
		paramListPartIndicator = -1;
	}

#endif
}

void telem_send_queued_param_by_index( uint32_t index )
{
	MAV_PARAM_TYPE param_type = get_param_mav_type( index );
	uintptr_t base = &config;
	uintptr_t addr = base + get_param_offset( index );

	if ( param_type == MAV_PARAM_TYPE_UINT8 ) {
		MVL_Send_Param_UINT8( *( uint8_t * ) addr, get_param_name( index ), get_sizeof_param_index(), index  );
	}	else if ( param_type == MAV_PARAM_TYPE_INT32 ) {
		MVL_Send_Param_INT32( *( int32_t * ) addr, get_param_name( index ), get_sizeof_param_index(), index  );
	}	else if ( param_type == MAV_PARAM_TYPE_UINT32 ) {
		MVL_Send_Param_UINT32( *( uint32_t * ) addr, get_param_name( index ), get_sizeof_param_index(), index  );
	} else if ( param_type == MAV_PARAM_TYPE_REAL32 ) {
		MVL_Send_Param_FLOAT( *( float * ) addr, get_param_name( index ), get_sizeof_param_index(), index  );
	}
}

/* MAVLink message handlers ------------------- */

void MVL_Transmit_Message( mavlink_message_t * mvl_msg_ptr )
{
	LL_GPIO_SetOutputPin( testPin1_GPIO_Port, testPin1_Pin );

	uint16_t tx_buflen = mavlink_msg_to_send_buffer( tx_byte_buffer, mvl_msg_ptr );
	UVOS_TELEM_puts( tx_byte_buffer, tx_buflen );

	LL_GPIO_ResetOutputPin( testPin1_GPIO_Port, testPin1_Pin );
}

void MVL_Handle_Manual_Control( mavlink_message_t * mvl_msg_ptr )
{
	mavlink_manual_control_t mvl_joy; //manual control data structure into which we decode the message
	mavlink_msg_manual_control_decode( mvl_msg_ptr, &mvl_joy );
	//For now, let's just retransmit the manual control message to see it in MAVLink Inspector
	mavlink_msg_manual_control_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
	                                        &mvl_tx_message, &mvl_joy );
	MVL_Transmit_Message( &mvl_tx_message );
}

void MVL_Handle_Param_Request_List( mavlink_message_t * mvl_msg_ptr )
{
	paramListPartIndicator = 0;
}

void MVL_Handle_Command_Long( mavlink_message_t * mvl_msg_ptr )
{
	mavlink_command_long_t mvl_cmd;
	mavlink_msg_command_long_decode( mvl_msg_ptr, &mvl_cmd );

	// debug_put_hex16( mvl_cmd.command );

	switch ( mvl_cmd.command ) {
	case ( MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ): {
		if ( 1 == mvl_cmd.param1 ) {
			mavlink_autopilot_version_t mvl_apv; //See: https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
			mvl_apv.flight_sw_version = 2;
			mvl_apv.middleware_sw_version = 1;
			mvl_apv.board_version = 1;
			mvl_apv.vendor_id = 10101;
			mvl_apv.product_id = 20202;
			mvl_apv.uid = 0;
			mvl_apv.capabilities = 0; //See: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY
			mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET; //Just as an example, code does not support! https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
			mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
			mavlink_msg_autopilot_version_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
			    &mvl_tx_message, &mvl_apv );
			MVL_Transmit_Message( &mvl_tx_message );
		}
		break;
	}//end handling of autopilot capabilities request
	case ( MAV_CMD_COMPONENT_ARM_DISARM ): {
		if ( 1 == mvl_cmd.param1 ) {
			mvl_armed = 1;
		} else {
			mvl_armed = 0;
		}
		//Acknowledge the arm/disarm command.
		mavlink_command_ack_t mvl_ack; //https://mavlink.io/en/messages/common.html#COMMAND_ACK
		mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM; //https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
		mvl_ack.result = MAV_RESULT_ACCEPTED; //https://mavlink.io/en/messages/common.html#MAV_RESULT
		mavlink_msg_command_ack_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
		                                     &mvl_tx_message, &mvl_ack );
		//skipped setting several fields here, with unknown consequences.
		MVL_Transmit_Message( &mvl_tx_message );
		break;
	}//end handling of arm/disarm command
	}//end switch/case
}//end MVL_Handle_Command_Long()

void MVL_Handle_Mission_Request_List( mavlink_message_t * mvl_msg_ptr )
{
	mavlink_mission_count_t mvl_mc;
	mvl_mc.target_system = MAV_SYSTEM_ID;
	mvl_mc.target_component = MAV_COMPONENT_ID;
	mvl_mc.count = 0;
	mavlink_msg_mission_count_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL,
	                                       &mvl_tx_message, &mvl_mc );
	MVL_Transmit_Message( &mvl_tx_message );
}

void MVL_Send_Param_UINT8( const uint8_t paramValue, const char * parameterName, const int listsize, const int index )
{
	mavlink_param_value_t mvl_param;
	mavlink_param_union_t u_param;
	mavlink_message_t msg;

	u_param.param_uint8 = paramValue;

	strncpy( mvl_param.param_id, parameterName, 16 );
	mvl_param.param_value = u_param.param_float;
	mvl_param.param_type =    MAV_PARAM_TYPE_UINT8; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
	mvl_param.param_count = listsize; //We have just one parameter to send.
	mvl_param.param_index = index;
	mavlink_msg_param_value_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL, &msg, &mvl_param );
	MVL_Transmit_Message( &msg );
}

void MVL_Send_Param_INT32( const int32_t paramValue, const char * parameterName, const int listsize, const int index )
{
	mavlink_param_value_t mvl_param;
	mavlink_param_union_t u_param;
	mavlink_message_t msg;

	u_param.param_int32 = paramValue;
	// u_param.type = MAV_PARAM_TYPE_INT32;

	strcpy( mvl_param.param_id, parameterName );
	mvl_param.param_value = u_param.param_float;
	mvl_param.param_type =  MAV_PARAM_TYPE_INT32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
	mvl_param.param_count = listsize; //We have just one parameter to send.
	mvl_param.param_index = index;
	mavlink_msg_param_value_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL, &msg, &mvl_param );
	MVL_Transmit_Message( &msg );
}

void MVL_Send_Param_UINT32( const uint32_t paramValue, const char * parameterName, const int listsize, const int index )
{
	mavlink_param_value_t mvl_param;
	mavlink_param_union_t u_param;
	mavlink_message_t msg;

	u_param.param_uint32 = paramValue;

	strcpy( mvl_param.param_id, parameterName );
	mvl_param.param_value = u_param.param_float;
	mvl_param.param_type =  MAV_PARAM_TYPE_UINT32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
	mvl_param.param_count = listsize; //We have just one parameter to send.
	mvl_param.param_index = index;
	mavlink_msg_param_value_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL, &msg, &mvl_param );
	MVL_Transmit_Message( &msg );
}

void MVL_Send_Param_FLOAT( const float paramValue_flt, const char * parameterName, const int listsize, const int index )
{
	mavlink_param_value_t mvl_param;
	// mavlink_param_union_t u_param;
	mavlink_message_t msg;

	// u_param.param_float = paramValue;

	strcpy( mvl_param.param_id, parameterName );
	// mvl_param.param_value = 123.456;
	mvl_param.param_value = paramValue_flt;
	mvl_param.param_type =  MAV_PARAM_TYPE_REAL32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
	mvl_param.param_count = listsize; //We have just one parameter to send.
	mvl_param.param_index = index;
	mavlink_msg_param_value_encode_chan( MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_CHANNEL, &msg, &mvl_param );
	MVL_Transmit_Message( &msg );
}