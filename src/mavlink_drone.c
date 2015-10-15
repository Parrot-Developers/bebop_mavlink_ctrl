/*
 * ============================================================================
 *
 *      @file           mavlink_drone.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           27/08/2015 11:22:12
 *
 * ============================================================================
 */
#include <stdio.h>
#include <errno.h>
#include <strings.h>
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
#include "mavlink_drone.h"
#include "mavlink_comm.h"

struct mavlink_drone {
	struct mavlink_info_cache cache;
	struct mavlink_comm *comm;
	int epollfd;
	int exitfd;
	int fd_1hz;
	void (*update_status_cb)(struct mavlink_info_cache *cache);
	uint8_t remote_system_id;
	uint8_t remote_component_id;
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

struct mavlink_drone *mavlink_drone_new(struct mavlink_drone_cfg *cfg)
{
	struct mavlink_drone *c;
	struct epoll_event ev;
	struct epoll_event events[3];
	struct itimerspec timeout;
	struct mavlink_comm_cfg comm_cfg = { .local_port = 14551,
		.remote_port = 14550,
		.cb = &callback};

	if (!cfg)
		return NULL;

	c = calloc(1, sizeof(struct mavlink_drone));
	if (!c)
		return NULL;
	c->remote_system_id = cfg->remote_system_id;
	c->remote_component_id = cfg->remote_component_id;
	c->update_status_cb = cfg->update_status_cb;
	comm_cfg.remote_addr = NULL;
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
	return c;

error:
	if (c->comm)
		mavlink_comm_destroy(c->comm);
	if (c->fd_1hz > 0)
		close(c->fd_1hz);
	if (c->epollfd > 0)
		close(c->epollfd);
	free(c);
	return NULL;
}

void mavlink_drone_destroy(struct mavlink_drone *c)
{
	if (!c)
		return;

	if (c->epollfd > 0)
		close(c->epollfd);
	free(c);
}

static void mavlink_drone_send_status(struct mavlink_drone *ctrl)
{
	mavlink_message_t msg;
	struct mavlink_info_cache *c = &ctrl->cache;
	int ret;
	if (ctrl->update_status_cb)
		(ctrl->update_status_cb)(c);

	mavlink_msg_heartbeat_pack(ctrl->remote_system_id,
			ctrl->remote_component_id,
			&msg, c->hb.type, c->hb.autopilot, c->hb.base_mode,
			0, c->hb.system_status);
	ret = mavlink_comm_send_msg(ctrl->comm, &msg);
	if (ret < 0)
		printf("%d error %d\n", __LINE__, ret);

	/* Send Status */
	mavlink_msg_sys_status_pack(ctrl->remote_system_id,
			ctrl->remote_component_id,
			&msg, c->sys_stat.onboard_control_sensors_present,
			c->sys_stat.onboard_control_sensors_enabled,
			c->sys_stat.onboard_control_sensors_health,
			c->sys_stat.load,
			c->sys_stat.voltage_battery,
			c->sys_stat.current_battery,
			c->sys_stat.drop_rate_comm,
			c->sys_stat.errors_comm,
			c->sys_stat.errors_count1,
			c->sys_stat.errors_count2,
			c->sys_stat.errors_count3,
			c->sys_stat.errors_count4,
			c->sys_stat.battery_remaining);

	ret = mavlink_comm_send_msg(ctrl->comm, &msg);
	if (ret < 0)
		printf("%d error %d\n", __LINE__, ret);


	/* Send Local Position */
	mavlink_msg_local_position_ned_pack(ctrl->remote_system_id,
			ctrl->remote_component_id, &msg, c->lpn.time_boot_ms,
			c->lpn.x, c->lpn.y, c->lpn.z,
			c->lpn.vx, c->lpn.vy, c->lpn.vz);
	ret = mavlink_comm_send_msg(ctrl->comm, &msg);
	if (ret < 0)
		printf("%d error %d\n", __LINE__, ret);

	/* Send attitude */
	mavlink_msg_attitude_pack(ctrl->remote_system_id,
			ctrl->remote_component_id,
			&msg, c->att.time_boot_ms, c->att.roll, c->att.pitch,
			c->att.yaw, c->att.rollspeed, c->att.pitchspeed,
			c->att.yawspeed);
	ret = mavlink_comm_send_msg(ctrl->comm, &msg);
	if (ret < 0)
		printf("%d error %d\n", __LINE__, ret);

}

void mavlink_drone_stop(struct mavlink_drone *c)
{
	int ret;
	uint64_t exit = 1;
	if (!c || c->exitfd < 0)
		return;
	ret = write(c->exitfd, &exit, sizeof(uint64_t));
	if (ret < 0)
		printf("failed to send exit\n");
}

uint64_t mavlink_drone_get_time_ms(void)
{
	return mavlink_comm_get_time_ms();
}

void mavlink_drone_poll_loop(struct mavlink_drone *c)
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
			if (new_events[n].data.fd == c->fd_1hz){
				(void) read(new_events[n].data.fd,
						&time_elapse,
						sizeof(uint64_t));
				mavlink_drone_send_status(c);
			}
			else if (new_events[n].data.fd == sockfd) {
				mavlink_comm_read_msg(c->comm);
			}
			else if (new_events[n].data.fd == c->exitfd) {
				(void) read(new_events[n].data.fd, &exit,
						sizeof(uint64_t));
			}
		}
	}
}
