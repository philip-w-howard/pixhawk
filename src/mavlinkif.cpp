/*
 * mavlinkif.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>

#include "mavlink/ardupilotmega/mavlink.h"
#include "mavlinkif.h"

static const uint8_t MY_SYSID = 0xFF;
static const uint8_t MY_COMPID = 0xBE;

static inline void byte_swap_8(void *dst, void *src)
{
	char *c_dst = (char *)dst;
	char *c_src = (char *)src;

	c_dst[0] = c_src[7];
	c_dst[1] = c_src[6];
	c_dst[2] = c_src[5];
	c_dst[3] = c_src[4];
	c_dst[4] = c_src[3];
	c_dst[5] = c_src[2];
	c_dst[6] = c_src[1];
	c_dst[7] = c_src[0];
}

static uint64_t microsSinceEpoch()
{

	struct timeval tv;
 	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	return micros;
}


static void send_msg(int pixhawk, mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int bytes;

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	// Send the message to the pixhawk and to the log file
	bytes = write(pixhawk, buf, len);
	if (bytes != len) printf("Failed to write msg to pixhawk: %d %d\n", bytes, len);
	else printf("Wrote %d bytes to pixhawk\n", len);
	if (bytes != len) printf("Failed to write msg to log: %d %d\n", bytes, len);
}

void send_param_request_list(int pixhawk)
{
	mavlink_system_t mavlink_system;

	mavlink_system.sysid = MY_SYSID;                   ///< ID this ground control
	mavlink_system.compid = MY_COMPID;     			   ///< component sending this msg

	// Initialize the required buffers
	mavlink_message_t msg;

	// Pack the message
	// Ignoring return code
	mavlink_msg_param_request_list_pack(
		mavlink_system.sysid, mavlink_system.compid, &msg,
		1,1);

	send_msg(pixhawk, &msg);
}

void send_ping(int pixhawk)
{
    static uint32_t seq = 0;

	// Initialize the required buffers
	mavlink_message_t msg;

    uint8_t target_system = 0;
    uint8_t target_component = 0;
    seq++;

		// Pack the message
	mavlink_msg_ping_pack(MY_SYSID, MY_COMPID, &msg,
			microsSinceEpoch(), seq, target_system, target_component);

	send_msg(pixhawk, &msg);
}

void send_change_operator_control(int pixhawk)
{
 	// Initialize the required buffers
	mavlink_message_t msg;

    uint8_t target_system = 1;
    uint8_t release_control = 0;		// zero means request control
    uint8_t version = 0;

		// Pack the message
    mavlink_msg_change_operator_control_pack(MY_SYSID, MY_COMPID, &msg,
    			target_system, release_control, version, "");

	send_msg(pixhawk, &msg);
}

void send_heartbeat(int pixhawk)
{
	mavlink_system_t mavlink_system;

	mavlink_system.sysid = MY_SYSID;                   ///< ID this ground control
	mavlink_system.compid = MY_COMPID;     			   ///< component sending this msg

	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_GCS;
	uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

	uint8_t system_mode = 0;
	uint32_t custom_mode = 0;
	uint8_t system_state = 0;

	// Initialize the required buffers
	mavlink_message_t msg;

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

	send_msg(pixhawk, &msg);
}

void send_request_data_stream(int pixhawk,
		uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
		uint16_t req_message_rate, uint8_t start_stop)
{
	mavlink_system_t mavlink_system;

	mavlink_system.sysid = MY_SYSID;                   ///< ID this ground control
	mavlink_system.compid = MY_COMPID;     			   ///< component sending this msg

	// Initialize the required buffers
	mavlink_message_t msg;

	// Pack the message
	// Ignoring return code
	mavlink_msg_request_data_stream_pack(
			mavlink_system.sysid, mavlink_system.compid, &msg,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);

	send_msg(pixhawk, &msg);
}

typedef struct
{
	int mav_channel;
	int fd;
	int bytes_at_time;
	void (*proc_msg)(mavlink_message_t *msg, void *param);
	void *proc_param;
	pthread_t thread_id;
	bool stop;
} read_params_t;

static void *read_msgs(void *p)
{
	read_params_t *params = (read_params_t *)p;

	mavlink_message_t msg;
	mavlink_status_t status;
	int num_msgs = 0;
	uint8_t input_buff[params->bytes_at_time];
	int length;

	memset(&msg, 0, sizeof(msg));
	memset(&status, 0, sizeof(status));

	while(!params->stop)
	{
		length = read(params->fd, input_buff, params->bytes_at_time);
		for (int ii=0; ii<length; ii++)
		{
			// Try to get a new message
			if(mavlink_parse_char(params->mav_channel, input_buff[ii], &msg, &status))
			{
				num_msgs++;

				params->proc_msg(&msg, params->proc_param);
			}
		}

		// Update global packet drops counter
		//packet_drops += status.packet_rx_drop_count;
	}

	return NULL;
}

void *start_message_thread(int mav_channel, int fd, int bytes_at_time,
		void (*proc_msg)(mavlink_message_t *msg, void *param), void *proc_param)
{
	read_params_t *params;

	params = (read_params_t *)malloc(sizeof(read_params_t));
	if (params == NULL)
	{
		perror("Unable to malloc read_params");
		return NULL;
	}

	params->mav_channel 		= mav_channel;
	params->fd     			= fd;
	params->bytes_at_time	= bytes_at_time;
	params->proc_msg			= proc_msg;
	params->proc_param		= proc_param;
	params->stop 			= false;

	int result = pthread_create(&params->thread_id, NULL, read_msgs, params);

	if (result != 0)
	{
		perror("Unable to start mavlink read thread");
		free(params);
		return NULL;
	}

	return params;
}

void stop_message_thread(void *p)
{
	read_params_t *params = (read_params_t *)p;

	params->stop = true;
	pthread_join(params->thread_id, NULL);
	free(params);
}

void write_tlog(int fd, mavlink_message_t *msg)
{
	struct timeval  tv;
	if (gettimeofday(&tv, NULL) != 0)
	{
		perror("Unable to get time of day\n");
	}

	uint64_t time_in_mill;

	time_in_mill = tv.tv_sec;
	time_in_mill *= 1000000;
	time_in_mill += tv.tv_usec;

	uint8_t timestamp[sizeof(uint64_t)];

	/*
	memcpy(timestamp, &time_in_mill, 8);

	printf("Timestamp: ");
	for (int ii=0; ii<8; ii++)
	{
		printf("%02X ", timestamp[ii]);
	}
	printf("\n");

	printf("Time: %ld %ld %04lX %04lX\n", tv.tv_sec, tv.tv_usec, tv.tv_sec, tv.tv_usec);
    */

	byte_swap_8(timestamp, &time_in_mill);

	write(fd, timestamp, sizeof(timestamp));

	/*
	printf("Timestamp: ");
	for (int ii=0; ii<8; ii++)
	{
		printf("%02X ", timestamp[ii]);
	}
	printf("\n");
    */

	// Copy the message to the send buffer
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	for (int ii=0; ii<6; ii++)
	{
		printf("%2X ", buf[ii]);
	}
	printf("\n");

	// CRC is at beginning of msg, we need to print it at the end
	write(fd, buf, len);
}
