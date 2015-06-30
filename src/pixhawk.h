/*
 * pixhawk.h
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */

#ifndef PIXHAWK_H_
#define PIXHAWK_H_

int open_pixhawk(char *portname);

void send_param_request_list(int pixhawk, int logfile);
void send_heartbeat(int pixhawk, int logfile);
void send_ping(int pixhawk, int logfile);
void send_change_operator_control(int pixhawk, int logfile);
void send_request_data_stream(int pixhawk, int logfile,
		uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
		uint16_t req_message_rate, uint8_t start_stop);

void process_messages(int pixhawk, int logfile, int count);

#endif /* PIXHAWK_H_ */
