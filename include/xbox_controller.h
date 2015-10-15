/*
 * ============================================================================
 *
 *      @file           xbox_controller.h
 *      @brief
 *      Copyright (C)   2013 Parrot S.A. www.parrot.com
 *      @author         Jeremie LESKA , jeremie.leska@parrot.com
 *      @date           14/10/2015 15:07:03
 *
 * ============================================================================
 */
struct xbox_controller_cmd {
	int lx;
	int ly;
	int rx;
	int ry;
	int button[2];
};

struct xbox_controller *xbox_controller_new(const char *path);

void xbox_controller_handle_command(struct xbox_controller *c,
				struct xbox_controller_cmd *cmd);

int xbox_controller_getfd(struct xbox_controller *c);
void xbox_controller_destroy(struct xbox_controller *c);
