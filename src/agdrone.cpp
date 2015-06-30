/*
 * ag.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

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

/*
void proc_msg(mavlink_message_t *msg, void *param)
{
	switch(msg.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		mavlink_heartbeat_t heartbeat;
		mavlink_msg_heartbeat_decode(&msg, &heartbeat);
		printf("%d Heartbeat: %d %d %d %d %d\n", num_msgs,
				heartbeat.type,
				heartbeat.autopilot,
				heartbeat.base_mode,
				heartbeat.system_status,
				heartbeat.mavlink_version);
		break;
	default:
		printf("%d msg: %02X %02X\n", num_msgs, msg.msgid, mavlink_msg_get_send_buffer_length(&msg)-8);
		break;
	}
	// CRC is at beginning of msg, we need to print it at the end
	write(logfile, &msg.magic,
			mavlink_msg_get_send_buffer_length(&msg) - sizeof(msg.checksum));
	write(logfile, &msg.checksum, sizeof(msg.checksum));

	//				if (num_msgs == 1) send_param_request_list(pixhawk, logfile);
}
*/

static volatile int Total_Msgs = 0;

void forward_msg(mavlink_message_t *msg, void *param)
{
	int fd = *(int *)param;

	// CRC is at beginning of msg, we need to print it at the end
	write(fd, &msg->magic,
			mavlink_msg_get_send_buffer_length(msg) - sizeof(msg->checksum));
	write(fd, &msg->checksum, sizeof(msg->checksum));

	Total_Msgs++;
	printf("Total msgs: %d\n", Total_Msgs);
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

	printf("waiting for connection on port 2002\n");
	int wifi = open_wifi(2002);
	if (wifi <= 0)
	{
	    printf("Failed to open wifi connection\n");
	    return -1;
	}

//	mraa::Gpio* led = new mraa::Gpio(13, true, false);
//	bool led_on = false;


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

	start_message_thread(0, wifi, 200, forward_msg, &pixhawk);
	start_message_thread(1, pixhawk, 1, forward_msg, &wifi);

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

	send_heartbeat(pixhawk, logfile);

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

	while (Total_Msgs < 500)
	{}

	close(wifi);
	close(logfile);
	close(pixhawk);
	std::cout << "Exiting\n";
	return MRAA_SUCCESS;
}


