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

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	perror("error %d from tcgetattr");
                return -1;
        }

        /*
        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 1;            // read blocks
        tty.c_cc[VTIME] = 255;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
		*/

        tty.c_cflag = speed | CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_lflag = 0;		// non-cannonical mode

        tcflush(fd, TCIFLUSH);
        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	perror ("error %d from tcsetattr");
                return -1;
        }
        return 0;
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

	char portname[] = "/dev/ttyMFD1";
	int pixhawk = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (pixhawk < 0)
	{
		perror ("error opening serial port");
		return 0;
	}
	else
	{
		set_interface_attribs(pixhawk, B57600, 0);  // 57600 set speed to 115,200 bps, 8n1 (no parity)
		//set_blocking (fu, 100);                // set blocking, with 100 characters to wait for
	}

	int logfile = open("/media/sdcard/pixhawk.tlog", O_RDWR | O_CREAT | O_TRUNC);
	if (logfile < 0)
	{
		perror("Unable to open log file");
		return -1;
	}

	char buff[200];
	int count;
	int written;
	for (int ii=0; ii<500; ii++)
	{
		count = read(pixhawk, buff, 100);
	    written = write(logfile, buff, count);
	    if (written != count)
	    {
	    	printf("Error writing: %d %d\n", written, count);
	    }
		//buff[count] = 0;
		//printf("%d: %s\n", count, buff);
		if (led_on)
			led->write(0);
		else
			led->write(1);
		led_on = !led_on;

	}

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
