/*
 * ag.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include<signal.h>

#include <mraa/gpio.hpp>
#include "mavlink/ardupilotmega/mavlink.h"

#include "pixhawk.h"
#include "wifi.h"
#include "mavlinkif.h"

// Ports:
// MFD2 is console
// MFD1 is UART
// USB0 is USBA radio
// ACM0 is USBA hardwire
static char portname[] = "/dev/ttyACM0";

static void signal_handler(int sig)
{
	printf("**************************** Received signal %d\n", sig);
}
int main()
{
	// check that we are running on Galileo or Edison
	mraa_platform_t platform = mraa_get_platform_type();
	if (platform != MRAA_INTEL_EDISON_FAB_C) {
		std::cerr << "Unsupported platform, exiting" << std::endl;
		return MRAA_ERROR_INVALID_PLATFORM;
	}

	std::cout << "pixhawk interface running on " << mraa_get_version() << std::endl;

	if (signal(SIGPIPE, signal_handler) == SIG_ERR)
	{
	    printf("\ncan't catch SIGPIPE\n");
	}

	queue_t *pixhawk_q = queue_create();
	if (pixhawk_q == NULL)
	{
		perror("Error opening pixhawk queue");
		return -1;
	}

	queue_t *mission_q = queue_create();
	if (mission_q == NULL)
	{
		perror("Error opening mission planner queue");
		return -1;
	}

	queue_t *agdrone_q = queue_create();
	if (agdrone_q == NULL)
	{
		perror("Error opening mission planner queue");
		return -1;
	}

	if (listen_to_wifi(2002, mission_q, agdrone_q) != 0)
	{
		perror("Unable to establish WiFi connection");
		return -1;
	}

	int pixhawk = open_pixhawk(portname);
	if (pixhawk < 0)
	{
		perror ("error opening serial port");
		return -1;
	}

	int logfile = open("/media/sdcard/pixhawk.tlog", O_RDWR | O_CREAT | O_TRUNC);
	if (logfile < 0)
	{
		perror("Unable to open log file");
		return -1;
	}

	//start_message_thread(0, wifi, 200, forward_msg, &pixhawk);
	start_message_read_thread(1, pixhawk, 1, agdrone_q);
	start_message_write_thread(pixhawk, pixhawk_q);

	int num_msgs = 0;
	mavlink_message_t *msg;

	while (1)
	{
		msg = (mavlink_message_t *)queue_remove(agdrone_q);
		if (msg != NULL)
		{
			write_tlog(logfile, msg);

			if (msg->sysid == 1)
				queue_msg(mission_q, msg);
			else
				queue_msg(pixhawk_q, msg);

			free(msg);
		}

		if (num_msgs % 1000 == 0) printf("processed %d msgs\n", num_msgs);
		num_msgs++;

		if (num_msgs == 100)
		{
			send_param_request_list(pixhawk_q, EDISON_SYSID, EDISON_COMPID);
		}
		if (num_msgs == 1000 || num_msgs == 1005)
		{
			send_request_data_stream(pixhawk_q, EDISON_SYSID, EDISON_COMPID,
					1, 1, MAV_DATA_STREAM_POSITION, 1, 1);
		}
		if (num_msgs == 2000 || num_msgs == 2005)
		{
			send_request_data_stream(pixhawk_q, EDISON_SYSID, EDISON_COMPID,
					1, 1, MAV_DATA_STREAM_POSITION, 20, 1);
		}
	}
//	send_change_operator_control(pixhawk, logfile);
//	send_change_operator_control(pixhawk, logfile);

//	send_param_request_list(pixhawk, logfile);

/*
	int target_system = 1;
	int target_component = 1;
	int start_stop = 1;
	int req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
	int req_message_rate = 2;
*/

	//send_heartbeat(pixhawk, logfile);

	/*
	req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
	req_message_rate = 2;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	req_stream_id = MAV_DATA_STREAM_POSITION;
	req_message_rate = 3;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	req_stream_id = MAV_DATA_STREAM_EXTRA1;
	req_message_rate = 10;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	req_stream_id = MAV_DATA_STREAM_EXTRA3;
	req_message_rate = 2;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	req_stream_id = MAV_DATA_STREAM_RAW_SENSORS;
	req_message_rate = 2;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	req_stream_id = MAV_DATA_STREAM_RC_CHANNELS;
	req_message_rate = 10;
	start_stop = 0;
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	send_request_data_stream(pixhawk, logfile,
			target_system, target_component, req_stream_id, req_message_rate, start_stop);
	*/

	//close(wifi);
	close(logfile);
	close(pixhawk);
	std::cout << "Exiting\n";
	return MRAA_SUCCESS;
}


