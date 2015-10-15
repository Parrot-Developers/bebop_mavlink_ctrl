/*
 * ============================================================================
 *
 *      @file           test.c
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           20/08/2015 09:26:20
 *
 * ============================================================================
 */

#include <stdio.h>
#include <errno.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include "mavlink_drone.h"

static void update_status(struct mavlink_info_cache *c)
{
	c->hb.custom_mode = 0;
	c->hb.type = MAV_TYPE_QUADROTOR;
	c->hb.autopilot = MAV_AUTOPILOT_GENERIC;
	c->hb.base_mode = MAV_MODE_GUIDED_ARMED;
	c->hb.system_status = MAV_STATE_ACTIVE;

	c->sys_stat.onboard_control_sensors_present = 0;
	c->sys_stat.onboard_control_sensors_enabled = 0;
	c->sys_stat.onboard_control_sensors_health = 0;
	c->sys_stat.load = 500;
	c->sys_stat.voltage_battery = 12000;
	c->sys_stat.current_battery = -1;
	c->sys_stat.drop_rate_comm = -1;
	c->sys_stat.errors_comm = 0;
	c->sys_stat.errors_count1 = 0;
	c->sys_stat.errors_count2 = 0;
	c->sys_stat.errors_count3 = 0;
	c->sys_stat.errors_count4 = 0;
	c->sys_stat.battery_remaining = 0;

	c->lpn.time_boot_ms = mavlink_drone_get_time_ms();
	c->lpn.x = 48.0;
	c->lpn.y = 173.0;
	c->lpn.z = 70.0;
	c->lpn.vx = 0.0;
	c->lpn.vy = 0.0;
	c->lpn.vz = 0.0;

	c->att.time_boot_ms = c->lpn.time_boot_ms;
	c->att.roll = 0.0;
	c->att.pitch = 0.0;
	c->att.yaw = 1.57;
	c->att.rollspeed = 0.0;
	c->att.pitchspeed = 0.0;
	c->att.yawspeed = 0.0;
}


int main(int argc, char* argv[])
{
	struct mavlink_drone *mavlink_drone;
	struct mavlink_drone_cfg cfg = {
		.remote_system_id = 1,
		.remote_component_id = 1,
		.update_status_cb = &update_status,
	};

	mavlink_drone = mavlink_drone_new(&cfg);
	if (!mavlink_drone) {
		printf("bad init ctrl\n");
		return -1;
	}

	mavlink_drone_poll_loop(mavlink_drone);
	mavlink_drone_destroy(mavlink_drone);

	return 0;
}
