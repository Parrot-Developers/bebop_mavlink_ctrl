/*
 * ============================================================================
 *
 *      @file           test_gcs.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 14:28:43
 *
 * ============================================================================
 */

#include <stdio.h>
#include <errno.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include "mavlink_gcs.h"
#include "xbox_controller.h"

static void update_status(struct mavlink_info_cache *c)
{
	c->hb.custom_mode = 0;
	c->hb.type = MAV_TYPE_GCS;
	c->hb.autopilot = MAV_AUTOPILOT_INVALID;
	c->hb.base_mode = MAV_MODE_GUIDED_ARMED;
	c->hb.system_status = MAV_STATE_ACTIVE;
}

static void controller_handle_command(struct mavlink_gcs *mavlink_gcs,
					void *u)
{
	struct xbox_controller *controller = (struct xbox_controller *)u;
	struct xbox_controller_cmd cmd;
	xbox_controller_handle_command(controller, &cmd);
	if (cmd.button[1]) {
		mavlink_gcs_send_takeoff(mavlink_gcs);
	} else if (cmd.button[0]) {
		mavlink_gcs_send_landing(mavlink_gcs);
	}
	mavlink_gcs_send_piloting_cmd(mavlink_gcs, cmd.rx, cmd.ry,
					-cmd.ly, cmd.lx);
}


static void display_help(void)
{
	printf("test_gcs ip_address joystick_name\n");
}

int main(int argc, char *argv[])
{
	struct mavlink_gcs *mavlink_gcs;
	struct xbox_controller *controller;
	struct mavlink_gcs_cfg cfg = {
		.remote_system_id = 1,
		.remote_component_id = MAV_COMP_ID_MISSIONPLANNER,
		.update_status_cb = &update_status,
		.controller_cb = &controller_handle_command,
	};

	if (argc != 3) {
		display_help();
		return -1;
	}

	strncpy(cfg.remote_addr, argv[1], MAVLINK_GCS_ADDRESS_SIZE);
	controller = xbox_controller_new(argv[2]);
	if (!controller) {
		printf("bad init controller\n");
		goto error;
	}
	cfg.controller_fd = xbox_controller_getfd(controller);
	cfg.controller_userdata = controller;
	mavlink_gcs = mavlink_gcs_new(&cfg);
	if (!mavlink_gcs) {
		printf("bad init gcs\n");
		goto error;
	}

	mavlink_gcs_poll_loop(mavlink_gcs);
	mavlink_gcs_destroy(mavlink_gcs);

	return 0;

error:
	if (mavlink_gcs)
		free(mavlink_gcs);
	if (controller)
		free(controller);
	return -1;
}
