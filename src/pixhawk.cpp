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
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#include "mavlink/common/mavlink.h"

// NOTE: This should come from a mavlink include file
static const int MAVLINK_OVERHEAD = 8;

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

	tty.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
	tty.c_iflag = IGNPAR;
	tty.c_oflag = 0;
	tty.c_lflag = 0;		// non-cannonical mode

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

	mraa::Gpio* led = new mraa::Gpio(13, true, false);
	bool led_on = false;

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
