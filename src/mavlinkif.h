/*
 * mavlinkif.h
 *
 *  Created on: Jun 30, 2015
 *      Author: philhow
 */

#ifndef MAVLINKIF_H_
#define MAVLINKIF_H_

#include "mavlink/ardupilotmega/mavlink.h"

void send_param_request_list(int pixhawk);
void send_heartbeat(int pixhawk);
void send_ping(int pixhawk);
void send_change_operator_control(int pixhawk);
void send_request_data_stream(int pixhawk,
		uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
		uint16_t req_message_rate, uint8_t start_stop);

void *start_message_thread(int mav_channel, int fd, int bytes_at_time,
		void (*proc_msg)(mavlink_message_t *msg, void *param), void *proc_param);

void stop_message_thread(void *p);

void write_tlog(int fd, mavlink_message_t *msg);

#endif /* MAVLINKIF_H_ */
