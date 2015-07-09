/*
 * wifi.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include "queue.h"
#include "mavlinkif.h"

typedef struct
{
	queue_t *wifi_q;
	queue_t *agdrone_q;
	int     sockfd;
} wifi_param_t;

static void *wifi_listener(void *param)
{
	wifi_param_t *wifi = (wifi_param_t *)param;

	int wififd;

	socklen_t clilen;
    struct sockaddr_in cli_addr;
    clilen = sizeof(cli_addr);
    printf("Listening for WiFi connections\n");

    while (1)
    {
    	wififd = accept(wifi->sockfd,
    			(struct sockaddr *) &cli_addr,
                &clilen);
    	if (wififd < 0)
    	{
    		perror("ERROR on accept");
    	}

    	printf("processing wifi data on %d\n", wififd);

    	start_message_read_thread(0, wififd, 200, wifi->agdrone_q);
    	start_message_write_thread(wififd, wifi->wifi_q);
    }

    return NULL;
}

int listen_to_wifi(int portno, queue_t *wifi_q, queue_t *agdrone_q)
{
    int sockfd;
    struct sockaddr_in serv_addr;
    pthread_t thread_id;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
       perror("ERROR opening socket");
       return -1;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
    {
        perror("ERROR on binding");
        close(sockfd);
        return -1;
    }

    listen(sockfd,5);

    wifi_param_t *params;
    params = (wifi_param_t *)malloc(sizeof(wifi_param_t));
    params->wifi_q = wifi_q;
    params->agdrone_q = agdrone_q;
    params->sockfd = sockfd;

    pthread_create(&thread_id, NULL, wifi_listener, params);

    return 0;
}


