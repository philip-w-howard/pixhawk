/*
 * Author: Philip Howard <phil.w.howard@gmail.com>
 */

//#include "grove.h"
//#include "jhd1313m1.h"
//#include <mraa/uart.hpp>
//#include <mraa/gpio.hpp>

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#include "pixhawk.h"
#include "mavlinkif.h"

#define MY_SYSID		0x41
#define MY_COMPID		0x41

int open_pixhawk(char *portname)
{
	int fd;

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

extern volatile int Total_Msgs;

void pixhawk_proc_msg(mavlink_message_t *msg, void *param)
{
	pix_proc_msg_t *pix_msg = (pix_proc_msg_t *)param;

	write_tlog(pix_msg->log_fd, msg);
	pix_msg->num_msgs++;
	if (pix_msg->num_msgs % 50 == 2)
	{
		send_param_request_list(pix_msg->send_q, MY_SYSID, MY_COMPID);
		//send_ping(pix_msg->send_q, MY_SYSID, MY_COMPID);
	}

	Total_Msgs++;
	printf("Total msgs: %d\n", Total_Msgs);
}

