/*
 * ============================================================================
 *
 *      @file           mavlink_comm.h
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           18/08/2015 17:50:03
 *
 * ============================================================================
 */

#include <mavlink.h>
#ifndef _MAVLINK_COMM_H_
#define _MAVLINK_COMM_H_

struct mavlink_comm_cfg {
	int local_port;
	void (*cb)(mavlink_message_t *msg, void *user_data);
	void *user_data;
	char *remote_addr;
	int remote_port;
};

struct mavlink_comm;

struct mavlink_comm *mavlink_comm_new(struct mavlink_comm_cfg *cfg);
int mavlink_comm_send_msg(struct mavlink_comm *c, mavlink_message_t *msg);
int mavlink_comm_get_sockfd(struct mavlink_comm *c);
int mavlink_comm_read_msg(struct mavlink_comm *c);
void mavlink_comm_destroy(struct mavlink_comm *c);
uint64_t mavlink_comm_get_time_ms(void);
#endif
