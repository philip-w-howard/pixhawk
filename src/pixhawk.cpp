/*
 * Author: Philip Howard <phil.w.howard@gmail.com>
 */

//#include "grove.h"
//#include "jhd1313m1.h"
//#include <mraa/uart.hpp>
#include <mraa/gpio.hpp>

#include <climits>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#include "mavlink/ardupilotmega/mavlink.h"

static const uint8_t MY_SYSID = 0xFF;
static const uint8_t MY_COMPID = 0xBE;

uint64_t microsSinceEpoch()
{

	struct timeval tv;
 	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	return micros;
}

int open_pixhawk()
{
	int fd;
	char portname[] = "/dev/ttyMFD1";

	fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		perror ("error opening serial port");
		return fd;
	}

	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		perror("error %d from tcgetattr");
		return -1;
	}


	cfsetispeed(&tty, B57600);
	cfsetospeed(&tty, B57600);

	tty.c_cflag &= ~CSIZE; /* Mask the character size bits */
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS ;

	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	tty.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
	tty.c_iflag = 0;  // IGNPAR;
	tty.c_oflag &= ~OPOST;		// turn off output processing

	tty.c_cc[VTIME] = 0;	// block until read is ready
	tty.c_cc[VMIN] = 1;		// wait for one character


	tcflush(fd, TCIFLUSH);
	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		perror ("error %d from tcsetattr");
		return -1;
	}
	return fd;
}
static void send_msg(int pixhawk, int logfile, mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int bytes;

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	// Send the message to the pixhawk and to the log file
	bytes = write(pixhawk, buf, len);
	if (bytes != len) printf("Failed to write msg to pixhawk: %d %d\n", bytes, len);
	write(logfile, buf, len);
	if (bytes != len) printf("Failed to write msg to log: %d %d\n", bytes, len);
}


static void send_ping(int pixhawk, int logfile)
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

	send_msg(pixhawk, logfile, &msg);
}

static void send_change_operator_control(int pixhawk, int logfile)
{
 	// Initialize the required buffers
	mavlink_message_t msg;

    uint8_t target_system = 1;
    uint8_t release_control = 0;		// zero means request control
    uint8_t version = 0;

		// Pack the message
    mavlink_msg_change_operator_control_pack(MY_SYSID, MY_COMPID, &msg,
    			target_system, release_control, version, "");

	send_msg(pixhawk, logfile, &msg);
}

static void send_heartbeat(int pixhawk, int logfile)
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

	send_msg(pixhawk, logfile, &msg);
}

static void send_request_data_stream(int pixhawk, int logfile,
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

	send_msg(pixhawk, logfile, &msg);
}

static void process_messages(int pixhawk, int logfile, int count)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	int num_msgs = 0;
	uint8_t input_char;
	int length;

	memset(&msg, 0, sizeof(msg));
	memset(&status, 0, sizeof(status));

	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)

	while(num_msgs < count)
	{
		length = read(pixhawk, &input_char, 1);
		if (length == 1)
		{
			// Try to get a new message
			if(mavlink_parse_char(MAVLINK_COMM_0, input_char, &msg, &status))
			{
				//if (num_msgs % 100 == 0) send_heartbeat(pixhawk, logfile);
				if (num_msgs % 100 == 0) send_ping(pixhawk, logfile);

				num_msgs++;

				// Handle message
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
			}
		} else {
			printf("Misread %d characters from pixhawk\n", length);
		}

		// Update global packet drops counter
		//packet_drops += status.packet_rx_drop_count;
	}
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

//	mraa::Gpio* led = new mraa::Gpio(13, true, false);
//	bool led_on = false;

	int pixhawk = open_pixhawk();
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

	send_change_operator_control(pixhawk, logfile);
	send_change_operator_control(pixhawk, logfile);

	/*
	int target_system = 1;
	int target_component = 1;
	int start_stop = 1;
	int req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
	int req_message_rate = 2;

	send_heartbeat(pixhawk, logfile);

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

	process_messages(pixhawk, logfile, 400);

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
