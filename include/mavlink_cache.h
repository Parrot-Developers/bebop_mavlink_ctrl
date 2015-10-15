/*
 * ============================================================================
 *
 *      @file           mavlink_cache.h
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 14:16:25
 *
 * ============================================================================
 */
#ifndef __MAVLINK_CACHE_H__
#define __MAVLINK_CACHE_H__
#include <mavlink.h>
struct mavlink_info_cache {
	mavlink_heartbeat_t hb;
	mavlink_sys_status_t sys_stat;
	mavlink_local_position_ned_t lpn;
	mavlink_attitude_t att;
};
#endif
