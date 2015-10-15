/*
 * ============================================================================
 *
 *      @file           mavlink_comm.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           18/08/2015 17:50:03
 *
 * ============================================================================
 */
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <poll.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <netdb.h>
#include <mavlink.h>
#include "mavlink_comm.h"

#define MAVLINK_COMM_BUFSIZE 4096

struct mavlink_comm {
	int sock;
	struct sockaddr_in remote_addr;
	unsigned char tx_buffer[MAVLINK_COMM_BUFSIZE];
	unsigned int tx_bufidx;
	unsigned char rx_buffer[MAVLINK_COMM_BUFSIZE];
	unsigned int rx_bufidx;
	void (*cb)(mavlink_message_t *msg, void *user_data);
	void *user_data;
};

struct mavlink_comm *mavlink_comm_new(struct mavlink_comm_cfg *cfg)
{
	struct sockaddr_in locAddr;
	struct mavlink_comm *c;
	struct hostent *server;

	if (!cfg)
		return NULL;

	c = calloc(1, sizeof(*c));
	if (!c)
		return NULL;
	c->cb = cfg->cb;
	c->user_data = cfg->user_data;

	c->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(cfg->local_port);

	if (cfg->remote_addr && cfg->remote_addr[0] != '\0') {
		server = gethostbyname(cfg->remote_addr);
		if (server == NULL) {
			fprintf(stderr,"ERROR, no such host %s\n",
					cfg->remote_addr);
			exit(0);
		}
		bzero((char *) &c->remote_addr, sizeof(c->remote_addr));
		c->remote_addr.sin_family = AF_INET;
		bcopy((char *)server->h_addr,
				(char *)&c->remote_addr.sin_addr.s_addr,
				server->h_length);
		c->remote_addr.sin_port = htons(cfg->remote_port);
	}
	/* Bind the socket to port 14551
	 * necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(c->sock,(struct sockaddr *)&locAddr,
				sizeof(struct sockaddr))) {
		perror("error bind failed");
		goto exit_close;
	} 
	return c;

exit_close:
	close(c->sock);
	free(c);
	return NULL;
}

static inline int mavlink_comm_send_data_internal(struct mavlink_comm *c)
{
	return sendto(c->sock, c->tx_buffer, c->tx_bufidx, 0,
			(struct sockaddr*)&c->remote_addr,
			sizeof(struct sockaddr_in));
}

int mavlink_comm_send_msg(struct mavlink_comm *c, mavlink_message_t *msg)
{
	int len;
	if (!c || !msg)
		return -EINVAL;
	len = mavlink_msg_to_send_buffer(c->tx_buffer, msg);

	return sendto(c->sock, c->tx_buffer, len, 0,
			(struct sockaddr*)&c->remote_addr,
			sizeof(struct sockaddr_in));
}

int mavlink_comm_get_sockfd(struct mavlink_comm *c)
{
	if (c)
		return c->sock;
	return -EINVAL;
}

int mavlink_comm_read_msg(struct mavlink_comm *c)
{
	int recsize;
	int i;
	mavlink_message_t msg;
	mavlink_status_t status;
	socklen_t fromlen;

	if (!c)
		return -EINVAL;


	recsize = recvfrom(c->sock, (void *)c->rx_buffer, MAVLINK_COMM_BUFSIZE,
			0, (struct sockaddr *)&c->remote_addr, &fromlen);

	if (recsize > 0) {
		/* Something received - print out all bytes and parse packet */
		/*printf("Bytes Received: %d\nDatagram: ", (int)recsize);*/
		for (i = 0; i < recsize; ++i) {
			/*temp = c->rx_buffer[i];
			printf("%02x ", (unsigned char)temp);*/
			if (mavlink_parse_char(MAVLINK_COMM_0, c->rx_buffer[i],
						&msg, &status) && c->cb) {
				(c->cb)(&msg, c->user_data);
			}
		}
	} else {
		printf("error receiving data %d\n", recsize);
		return recsize;
	}
	return 0;
}

int mavlink_comm_flush_tx_buffer(struct mavlink_comm *c)
{
	if (!c)
		return -EINVAL;
	return mavlink_comm_send_data_internal(c);
}

void mavlink_comm_destroy(struct mavlink_comm *c)
{
	if (!c)
		return;
	close(c->sock);	
	free(c);
}

uint64_t mavlink_comm_get_time_ms(void)
{
	struct timespec ts;
	uint64_t ms = 0;

	clock_gettime(CLOCK_MONOTONIC, &ts);  
	ms = ((uint64_t)ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;

	return ms;
}
