/*
 * ============================================================================
 *
 *      @file           mavlink_gcs.h
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 14:14:34
 *
 * ============================================================================
 */
#include "mavlink_cache.h"
#ifndef _MAVLINK_GCS_H_
#define _MAVLINK_GCS_H_
struct mavlink_gcs;
#define MAVLINK_GCS_ADDRESS_SIZE 64
struct mavlink_gcs_cfg {
	unsigned int remote_system_id;
	unsigned int remote_component_id;
	void (*update_status_cb)(struct mavlink_info_cache *cache);
	void (*controller_cb)(struct mavlink_gcs *gcs,
				void *controller_userdata);
	void *controller_userdata;
	int controller_fd;
	char remote_addr[MAVLINK_GCS_ADDRESS_SIZE];
};
uint64_t mavlink_gcs_get_time_ms(void);
struct mavlink_gcs *mavlink_gcs_new(struct mavlink_gcs_cfg *cfg);
void mavlink_gcs_destroy(struct mavlink_gcs *c);
void mavlink_gcs_stop(struct mavlink_gcs *c);
void mavlink_gcs_poll_loop(struct mavlink_gcs *c);
void mavlink_gcs_send_takeoff(struct mavlink_gcs *gcs);
void mavlink_gcs_send_landing(struct mavlink_gcs *gcs);
void mavlink_gcs_send_piloting_cmd(struct mavlink_gcs *gcs,
				int16_t x, int16_t y, int16_t z, int16_t r);
#endif
