/*
 * ============================================================================
 *
 *      @file           mavlink_drone.h
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 12:25:30
 *
 * ============================================================================
 */
#ifndef __MAVLINK_DRONE_H__
#define __MAVLINK_DRONE_H__

#include "mavlink_cache.h"

struct mavlink_drone_cfg {
	unsigned int remote_system_id;
	unsigned int remote_component_id;
	void (*update_status_cb)(struct mavlink_info_cache *cache);
};
uint64_t mavlink_drone_get_time_ms(void);
struct mavlink_drone *mavlink_drone_new(struct mavlink_drone_cfg *cfg);
void mavlink_drone_destroy(struct mavlink_drone *c);
void mavlink_drone_stop(struct mavlink_drone *c);
void mavlink_drone_poll_loop(struct mavlink_drone *c);
#endif
