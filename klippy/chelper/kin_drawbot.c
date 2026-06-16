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
    struct coord anchor; // X,Y,Z of this motor
    struct coord other_anchor; // X,Y,Z of opposing motor
    double k_factor; // density / (10^6 * toolhead_mass)
    int is_catenary; // Flag to enable catenary sag correction
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
    double d = sqrt(dx*dx + dy*dy + dz*dz);
    
    if (!hs->is_catenary || hs->k_factor <= 0.0)
        return d;
        
    // Catenary sag correction approximation: S = d * (1 + F^2 / 24)
    // F = k * (dy_this + dy_other * |x - x_this| / |x - x_other|)
    double x_diff_this = fabs(c.x - hs->anchor.x);
    double x_diff_other = fabs(c.x - hs->other_anchor.x);
    
    // Guard against division by zero if directly under other anchor
    if (x_diff_other < 0.1)
        x_diff_other = 0.1;
        
    double dy_this = hs->anchor.y - c.y;
    double dy_other = hs->other_anchor.y - c.y;
    
    double F = hs->k_factor * (dy_this + dy_other * (x_diff_this / x_diff_other));
    
    // Guard against excessive F causing overflow
    if (F > 2.0)
        F = 2.0;
        
    return d * (1.0 + (F * F) / 24.0);
}

struct stepper_kinematics * __visible
drawbot_stepper_alloc(double anchor_x, double anchor_y, double anchor_z
                    , double other_x, double other_y, double other_z
                    , double k_factor, int is_catenary)
{
    struct drawbot_stepper *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->anchor.x = anchor_x;
    hs->anchor.y = anchor_y;
    hs->anchor.z = anchor_z;
    hs->other_anchor.x = other_x;
    hs->other_anchor.y = other_y;
    hs->other_anchor.z = other_z;
    hs->k_factor = k_factor;
    hs->is_catenary = is_catenary;
    hs->sk.calc_position_cb = drawbot_stepper_calc_position;
    hs->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &hs->sk;
}