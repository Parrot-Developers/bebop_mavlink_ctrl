/*
 * ============================================================================
 *
 *      @file           mavlink_gcs.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 14:04:52
 *
 * ============================================================================
 */
#include <errno.h>
#include <strings.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <sys/poll.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include "mavlink_gcs.h"
#include "mavlink_comm.h"

struct mavlink_gcs {
	struct mavlink_info_cache cache;
	struct mavlink_comm *comm;
	int epollfd;
	int exitfd;
	int fd_1hz;
	int fd_5hz;
	int controller_fd;
	void (*update_status_cb)(struct mavlink_info_cache *cache);
	void (*controller_cb)(struct mavlink_gcs *c,
			void *controller_userdata);
	void *controller_userdata;
	uint8_t remote_system_id;
	uint8_t remote_component_id;
	mavlink_manual_control_t last_pcmd;
	uint64_t last_pcmd_time;
};

static void callback(mavlink_message_t *msg, void *user_data)
{
	if (!msg)
		return;
	/* Packet received */
	printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n",
			msg->sysid, msg->compid,
			msg->len, msg->msgid);
}

static int timerfd_init(const struct itimerspec *ti)
{
	int fd = -1;
	int ret = -1;
	if (ti == NULL)
		return -1;
	/* create new timer */
	fd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (fd < 0) {
		printf("Failed to create timer\n");
		return -1;
	}

	/* set to non-blocking */
	ret = fcntl(fd, F_SETFL, O_NONBLOCK);
	if (ret) {
		printf("Failed to set to non blocking mode\n");
		close(fd);
		return -1;
	}

	/* set timeout */
	ret = timerfd_settime(fd, 0, ti, NULL);
	if (ret) {
		printf("Failed to set timer duration\n");
		close(fd);
		return -1;
	}
	printf("%s fd %d\n", __func__, fd);
	return fd;
}


struct mavlink_gcs *mavlink_gcs_new(struct mavlink_gcs_cfg *cfg)
{
	struct mavlink_gcs *c;
	struct epoll_event ev;
	struct epoll_event events[5];
	struct itimerspec timeout;
	struct mavlink_comm_cfg comm_cfg = { .local_port = 14550,
		.remote_port = 14551,
		.cb = &callback};

	if (!cfg)
		return NULL;

	comm_cfg.remote_addr = cfg->remote_addr;

	c = calloc(1, sizeof(struct mavlink_gcs));
	if (!c)
		return NULL;
	c->remote_system_id = cfg->remote_system_id;
	c->remote_component_id = cfg->remote_component_id;
	c->update_status_cb = cfg->update_status_cb;
	c->controller_fd = cfg->controller_fd;
	c->controller_cb = cfg->controller_cb;
	c->controller_userdata = cfg->controller_userdata;
	c->comm = mavlink_comm_new(&comm_cfg);
	if (!c->comm) {
		printf("bad init comm\n");
		goto error;
	}

	c->exitfd = eventfd(0,0);
	bzero(&timeout, sizeof(timeout));
	timeout.it_value.tv_sec =  1;
	timeout.it_value.tv_nsec = 0;
	timeout.it_interval.tv_sec = 1;
	timeout.it_interval.tv_nsec = 0;

	c->fd_1hz = timerfd_init(&timeout);
	if (c->fd_1hz < 0) {
		printf("Failed to create timerfd 1hz\n");
		goto error;
	}

	bzero(&timeout, sizeof(timeout));
	timeout.it_value.tv_sec =  0;
	timeout.it_value.tv_nsec = 200000000;
	timeout.it_interval.tv_sec = 0;
	timeout.it_interval.tv_nsec = 200000000;

	c->fd_5hz = timerfd_init(&timeout);
	if (c->fd_5hz < 0) {
		printf("Failed to create timerfd 1hz\n");
		goto error;
	}

	printf("size %lu\n", sizeof(events) / sizeof(events[0]));
	c->epollfd = epoll_create(sizeof(events) / sizeof(events[0]));
	if (c->epollfd < 0) {
		printf("epoll_create failed\n");
		goto error;
	}

	ev.events = EPOLLIN;
	ev.data.fd = c->fd_1hz;
	if (epoll_ctl(c->epollfd, EPOLL_CTL_ADD, c->fd_1hz, &ev) == -1) {
		printf("failed to add fd_1hz\n");
		goto error;
	}

	ev.events = EPOLLIN;
	ev.data.fd = c->fd_5hz;
	if (epoll_ctl(c->epollfd, EPOLL_CTL_ADD, c->fd_5hz, &ev) == -1) {
		printf("failed to add fd_1hz\n");
		goto error;
	}

	ev.events = EPOLLIN;
	ev.data.fd = mavlink_comm_get_sockfd(c->comm);
	if (epoll_ctl(c->epollfd, EPOLL_CTL_ADD, ev.data.fd, &ev) == -1) {
		printf ("failed to add sockfd\n");
		goto error;
	}

	ev.events = EPOLLIN;
	ev.data.fd = c->exitfd;
	if (epoll_ctl(c->epollfd, EPOLL_CTL_ADD, c->exitfd, &ev) == -1) {
		printf("failed to add eventfd\n");
		goto error;
	}
	ev.events = EPOLLIN;
	ev.data.fd = c->controller_fd;
	if (epoll_ctl(c->epollfd, EPOLL_CTL_ADD, c->controller_fd, &ev) == -1) {
		printf("failed to add eventfd\n");
		goto error;
	}

	return c;

error:
	if (c->comm)
		mavlink_comm_destroy(c->comm);
	if (c->fd_1hz > 0)
		close(c->fd_1hz);
	if (c->fd_5hz > 0)
		close(c->fd_5hz);	
	if (c->epollfd > 0)
		close(c->epollfd);
	free(c);
	return NULL;
}

void mavlink_gcs_destroy(struct mavlink_gcs *c)
{
	if (!c)
		return;

	if (c->epollfd > 0)
		close(c->epollfd);
	free(c);
}

void mavlink_gcs_send_takeoff(struct mavlink_gcs *gcs)
{
	mavlink_message_t msg;
	if (!gcs)
		return;
	printf("takeoff\n");
	mavlink_msg_command_long_pack(gcs->remote_system_id,
					gcs->remote_component_id,
					&msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0,
					0,0,0,0,0,0,0);
	mavlink_comm_send_msg(gcs->comm, &msg);
}

void mavlink_gcs_send_landing(struct mavlink_gcs *gcs)
{
	mavlink_message_t msg;
	if (!gcs)
		return;
	printf("landing\n");
	mavlink_msg_command_long_pack(gcs->remote_system_id,
					gcs->remote_component_id,
					&msg, 1, 1, MAV_CMD_NAV_LAND, 0,
					0,0,0,0,0,0,0);
	mavlink_comm_send_msg(gcs->comm, &msg);
}

void mavlink_gcs_send_piloting_cmd(struct mavlink_gcs *gcs,
				int16_t x, int16_t y, int16_t z, int16_t r)
{
	mavlink_message_t msg;
	if (!gcs)
		return;
	uint64_t tsnow = mavlink_comm_get_time_ms();
	if (gcs->last_pcmd_time + 20 < tsnow) {
		printf("pcmd %d %d %d %d\n", x, y , z, r);
		mavlink_msg_manual_control_pack(1, 1, &msg,
					gcs->remote_system_id, x, y ,
					(z + 1000) / 2, r, 0);
		mavlink_comm_send_msg(gcs->comm, &msg);
		gcs->last_pcmd_time = tsnow;
	}
	gcs->last_pcmd.x = x;
	gcs->last_pcmd.y = y;
	gcs->last_pcmd.z = z;
	gcs->last_pcmd.r = r;
}

static void mavlink_gcs_send_status(struct mavlink_gcs *gcs)
{
	mavlink_message_t msg;
	struct mavlink_info_cache *c = &gcs->cache;
	int ret;

	if (gcs->update_status_cb)
		(gcs->update_status_cb)(c);

	mavlink_msg_heartbeat_pack(gcs->remote_system_id,
			gcs->remote_component_id,
			&msg, c->hb.type, c->hb.autopilot, c->hb.base_mode,
			0, c->hb.system_status);

	ret = mavlink_comm_send_msg(gcs->comm, &msg);
	if (ret < 0)
		printf("%d error %d\n", __LINE__, ret);
}

void mavlink_gcs_stop(struct mavlink_gcs *c)
{
	int ret;
	uint64_t exit = 1;
	if (!c || c->exitfd < 0)
		return;
	ret = write(c->exitfd, &exit, sizeof(uint64_t));
	if (ret < 0)
		printf("failed to send exit\n");
}

uint64_t mavlink_gcs_get_time_ms(void)
{
	return mavlink_comm_get_time_ms();
}

void mavlink_gcs_poll_loop(struct mavlink_gcs *c)
{
	int n, nfds;
	uint64_t exit = 0;
	uint64_t time_elapse;
	int sockfd = mavlink_comm_get_sockfd(c->comm);
	struct epoll_event new_events[3];
	while (!exit) {
		do {
			nfds = epoll_wait(c->epollfd, new_events,
					sizeof(new_events) / sizeof(new_events[0]),
					-1);
		} while (nfds < 0 && errno == EINTR);

		if (nfds < 0) {
			printf("epoll_wait failed\n");
			return;
		}

		for (n = 0; n < nfds; n++) {
			if (!(new_events[n].events & EPOLLIN))
				continue;
			if (new_events[n].data.fd == c->fd_1hz) {
				(void) read(new_events[n].data.fd,
						&time_elapse,
						sizeof(uint64_t));
				mavlink_gcs_send_status(c);
			}
			else if (new_events[n].data.fd == c->fd_5hz){
				(void) read(new_events[n].data.fd,
						&time_elapse,
						sizeof(uint64_t));
				c->controller_cb(c, c->controller_userdata);
			}
			else if (new_events[n].data.fd == sockfd) {
				mavlink_comm_read_msg(c->comm);
			}
			else if (new_events[n].data.fd == c->exitfd) {
				(void) read(new_events[n].data.fd, &exit,
						sizeof(uint64_t));
			}
			else if (new_events[n].data.fd == c->controller_fd) {
				c->controller_cb(c, c->controller_userdata);
			}
		}
	}
}
