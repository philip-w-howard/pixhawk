#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include <mraa/gpio.hpp>
#include "mavlink/ardupilotmega/mavlink.h"

#include "pixhawk.h"
#include "wifi.h"

// Ports:
// MFD2 is console
// MFD1 is UART
// USB0 is USBA radio
// ACM0 is USBA hardwire
static char portname[] = "/dev/ttyACM0";

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
	int failure = open_wifi(2002);
	if (failure) 
	{
	    printf("Failed to open wifi connection\n");
	    return failure;
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

//	send_change_operator_control(pixhawk, logfile);
//	send_change_operator_control(pixhawk, logfile);

	send_param_request_list(pixhawk, logfile);

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

	process_messages(pixhawk, logfile, 100);

	/*
		if (led_on)
			led->write(0);
		else
			led->write(1);
		led_on = !led_on;
    */

	close(logfile);
	close(pixhawk);
	/*
	// button connected to D4 (digital in)
	//upm::GroveButton* button = new upm::GroveButton(4);

	// led connected to D3 (digital out)
	//upm::GroveLed* led = new upm::GroveLed(13);

	// LCD connected to the default I2C bus
	upm::Jhd1313m1* lcd = new upm::Jhd1313m1(0);

	lcd->setCursor(0,0);
	lcd->write("Hello world");
	lcd->setCursor(1,0);
	lcd->write(mraa_get_version());

	// loop forever updating the temperature values every second

	for (int ii=0; ii<10; ii++) {
		led->write(1);
		usleep(500000);
		led->write(0);
		usleep(500000);
	}

	sleep(5);
	lcd->clear();
	sleep(5);

	lcd->setColor(0,0,0);
*/
	std::cout << "Exiting\n";
	return MRAA_SUCCESS;
}
