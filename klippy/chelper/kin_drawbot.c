// Drawbot kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct drawbot_stepper {
    struct stepper_kinematics sk;
    struct coord anchor; // X,Y,Z of each motor
};

static double
drawbot_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct drawbot_stepper *hs = container_of(sk, struct drawbot_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double dx = hs->anchor.x - c.x;
    double dy = hs->anchor.y - c.y;
    double dz = hs->anchor.z - c.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

struct stepper_kinematics * __visible
drawbot_stepper_alloc(double anchor_x, double anchor_y, double anchor_z)
{
    struct drawbot_stepper *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->anchor.x = anchor_x;
    hs->anchor.y = anchor_y;
    hs->anchor.z = anchor_z;
    hs->sk.calc_position_cb = drawbot_stepper_calc_position;
    hs->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &hs->sk;
}