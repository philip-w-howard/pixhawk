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

int open_wifi(int portno)
{
    int sockfd, newsockfd;
    socklen_t clilen;
//    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
//    int n;

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
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd,
                (struct sockaddr *) &cli_addr,
                &clilen);
    if (newsockfd < 0)
    {
         perror("ERROR on accept");
         close(sockfd);
         return -1;
    }

    return newsockfd;
    /*
    bzero(buffer,256);
    n = read(newsockfd,buffer,sizeof(buffer));
    if (n < 0) perror("ERROR reading from socket");
    printf("Here is the message: \n");
    for (int ii=0; ii<n; ii++)
    {
    	printf("%d %02X '%c'\n", ii, buffer[ii], buffer[ii]);
    }

    close(newsockfd);
    close(sockfd);

    return 0;
    */
}


