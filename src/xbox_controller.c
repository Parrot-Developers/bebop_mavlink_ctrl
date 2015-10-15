/*
 * ============================================================================
 *
 *      @file           xbox_controller.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 14:35:09
 *
 * ============================================================================
 */
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <poll.h>
#include <linux/joystick.h>
#include "xbox_controller.h"

#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_X 2

#define LEFT_AXIS_X 0
#define LEFT_AXIS_Y 1

#define RIGHT_AXIS_X 4
#define RIGHT_AXIS_Y 3
#define XBOX_CONTROLLER_STR_SIZE 128

struct xbox_controller {
	char axes;
	char buttons;
	char name[XBOX_CONTROLLER_STR_SIZE];
	char path[XBOX_CONTROLLER_STR_SIZE];
	int fd;
	struct xbox_controller_cmd last_cmd;
};

static int convertAxisValue(int v)
{
	return (v * 1000) / INT16_MAX;
}

static void handleAxis(struct xbox_controller_cmd *cmd,
			const struct js_event *e)
{
	int value = convertAxisValue(e->value);

	if (-100 < value && value < 100)
		value = 0;

	switch (e->number) {
	case LEFT_AXIS_X:
		cmd->lx = value;
		break;

	case LEFT_AXIS_Y:
		cmd->ly = value;
		break;

	case RIGHT_AXIS_X:
		cmd->rx = value;
		break;

	case RIGHT_AXIS_Y:
		cmd->ry = value;
		break;

	default:
		break;
	}
}

static void handleButton(struct xbox_controller_cmd *cmd,
			const struct js_event *e)
{
	if (e->value == 0)
		return;

	switch (e->number) {
	case BUTTON_X:
		/* landing */
		cmd->button[0] = 1;
		break;

	case BUTTON_A:
		/* takeoff */
		cmd->button[1] = 1;		
		break;

	default:
		break;
	}
}

struct xbox_controller *xbox_controller_new(const char *path)
{
	int ret;

	struct xbox_controller *c = calloc(1, sizeof(*c));
	if (!path)
		return NULL;
	c->fd = open(path, O_RDONLY);
	if (c->fd == -1)
		return NULL;
	// Read infos
	ret = ioctl(c->fd, JSIOCGAXES, &c->axes);
	if (ret == -1) {
		printf("Fail to fetch axis count : %s", strerror(errno));
		ret = -errno;
		goto error;
	}

	ret = ioctl(c->fd, JSIOCGBUTTONS, &c->buttons);
	if (ret == -1) {
		printf("Fail to fetch buttons count : %s", strerror(errno));
		ret = -errno;
		goto error;
	}

	ret = ioctl(c->fd, JSIOCGNAME(sizeof(c->name)), c->name);
	if (ret == -1) {
		printf("Fail to fetch name : %s", strerror(errno));
		ret = -errno;
		goto error;
	}

        fcntl(c->fd, F_SETFL, O_NONBLOCK );   /* use non-blocking mode */
	strncpy(c->path, path, XBOX_CONTROLLER_STR_SIZE);

	return c;

error:
	close(c->fd);

	return NULL;
}

void xbox_controller_destroy(struct xbox_controller *c)
{
	if (c)
		free(c);
}

int xbox_controller_getfd(struct xbox_controller *c)
{
	if (!c)
		return -EINVAL;
	return c->fd;
}

void xbox_controller_handle_command(struct xbox_controller *c,
				struct xbox_controller_cmd *cmd)
{
	struct js_event e;
	int ret;
	if (!cmd || !c)
		return;

	c->last_cmd.button[0] = 0;
	c->last_cmd.button[1] = 0;
	lseek(c->fd, SEEK_SET, 0);
	ret = read(c->fd, &e, sizeof(e));
	if (ret < 0) {
		memcpy(cmd, &c->last_cmd, sizeof(struct xbox_controller_cmd));
		fflush(stdout);
		return;
	}

	e.type &= ~JS_EVENT_INIT;
	switch (e.type) {
	case JS_EVENT_AXIS:
		handleAxis(&c->last_cmd, &e);
		break;

	case JS_EVENT_BUTTON:
		handleButton(&c->last_cmd, &e);
		break;

	default:
		printf("%s : unexpected event type %d\n", c->path, e.type);
		break;
	}
	memcpy(cmd, &c->last_cmd, sizeof(struct xbox_controller_cmd));
}
