/*
 * wifi.h
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */

#ifndef WIFI_H_
#define WIFI_H_

#include "queue.h"

// start a server connection
int listen_to_wifi(int portno, queue_t *wifi_q, queue_t *agdrone_q);

// start a client connection
int start_wifi(char *host, int portno, queue_t *wifi_q, queue_t *agdrone_q);



#endif /* WIFI_H_ */
