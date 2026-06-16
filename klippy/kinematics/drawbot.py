# Code for handling the kinematics of drawbot robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, math

class DrawbotKinematics:
    def __init__(self, toolhead, config):
        # Setup steppers at each anchor
        self.steppers = []
        self.anchors = []

        # Load physical parameters for dynamic limit solving
        self.toolhead_mass = config.getfloat('toolhead_mass', 0.2, above=0.) # kg
        self.max_cable_tension = config.getfloat('max_cable_tension', 30.0, above=0.) # Newtons
        self.cable_linear_density = config.getfloat('cable_linear_density', 0.0, minval=0.0)
        self.enable_catenary_compensation = config.getboolean('enable_catenary_compensation', False)
        
        # Calculate k_factor (rho / m) in mm^-1
        self.k_factor = 0.0
        if self.enable_catenary_compensation and self.toolhead_mass > 0.0:
            self.k_factor = self.cable_linear_density / (1000000.0 * self.toolhead_mass)

        # Pull global velocity and acceleration limits
        self.max_velocity = config.getfloat('max_velocity', 300.0, above=0.)
        self.max_accel = config.getfloat('max_accel', 1000.0, above=0.)

        # 1. Parse configs and extract anchors
        stepper_configs = []
        for name in ['stepper_a', 'stepper_b']:
            s_config = config.getsection(name)
            s = stepper.PrinterStepper(s_config)
            self.steppers.append(s)
            a = tuple([s_config.getfloat('anchor_' + n) for n in 'xyz'])
            self.anchors.append(a)
            stepper_configs.append((s, a))

        # 2. Call setup_itersolve now that we have both anchors
        # Stepper A
        s_a, a_a = stepper_configs[0]
        a_b = stepper_configs[1][1]
        s_a.setup_itersolve('drawbot_stepper_alloc', *a_a, *a_b, self.k_factor, int(self.enable_catenary_compensation))
        s_a.set_trapq(toolhead.get_trapq())
        toolhead.register_step_generator(s_a.generate_steps)

        # Stepper B
        s_b, a_b = stepper_configs[1]
        a_a = stepper_configs[0][1]
        s_b.setup_itersolve('drawbot_stepper_alloc', *a_b, *a_a, self.k_factor, int(self.enable_catenary_compensation))
        s_b.set_trapq(toolhead.get_trapq())
        toolhead.register_step_generator(s_b.generate_steps)

        # Setup boundary checks
        # X range from left anchor to right anchor.
        # Y range goes from a safe lower limit (-300) up to the height of the anchors.
        self.axes_min = toolhead.Coord(min(self.anchors[0][0], self.anchors[1][0]), -300., 0., e=0.)
        self.axes_max = toolhead.Coord(max(self.anchors[0][0], self.anchors[1][0]), max(self.anchors[0][1], self.anchors[1][1]), 0., e=0.)
        self.set_position([0., 0., 0.], "")

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        # https://en.wikipedia.org/wiki/Two-center_bipolar_coordinates
        anchor_left, anchor_right = self.anchors[:2]
        
        # Calculate half-width W (distance from origin to each anchor)
        width = (anchor_right[0] - anchor_left[0]) / 2.0
        
        # Get actual commanded position values (float) from the steppers
        stepper_left, stepper_right = self.steppers[:2]
        L = stepper_positions[stepper_left.get_name()]
        R = stepper_positions[stepper_right.get_name()]

        x = (L*L - R*R) / (4.0 * width)
        chunk = L*L - R*R + 4.0 * width*width
        
        # Guard against domain error in sqrt if input values are out of bounds
        val = 16.0 * width*width * L*L - chunk*chunk
        if val < 0.0:
            val = 0.0
            
        y = math.sqrt(val) / (4.0 * width)
        
        # Subtract from anchor Y coordinate (anchor_left[1]) as the toolhead hangs below anchors
        y_pos = anchor_left[1] - y
        
        return (x, y_pos, 0.)

    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)

    def clear_homing_state(self, clear_axes):
        # XXX - homing not implemented
        pass

    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])

    def check_move(self, move):
        x, y, z = move.end_pos[:3]
        if move.move_d == 0.:
            return
            
        # Unit direction vector of the move in Cartesian space
        nx = move.axes_d[0] / move.move_d
        ny = move.axes_d[1] / move.move_d
        
        gravity = 9.81 # m/s^2
        
        # Track the most restrictive speed/accel scaling factors
        speed_factor = 1.0
        accel_factor = 1.0
        
        # Calculate distance to anchors
        L1 = math.sqrt((self.anchors[0][0] - x)**2 + (self.anchors[0][1] - y)**2)
        L2 = math.sqrt((self.anchors[1][0] - x)**2 + (self.anchors[1][1] - y)**2)
        
        # Dynamic Effective Mass calculation to accommodate cable weight:
        # L1 and L2 are in mm. Linear density is in grams/meter.
        # total_cable_mass = (L1 + L2) * 1e-3 (m) * density * 1e-3 (kg)
        total_cable_mass_kg = (L1 + L2) * self.cable_linear_density * 0.000001
        
        # Half of the cable mass is assumed to be supported by the carriage
        effective_mass = self.toolhead_mass + (total_cable_mass_kg / 2.0)
        
        for i, (L, anchor) in enumerate(zip([L1, L2], self.anchors[:2])):
            if L == 0.:
                raise move.move_error("Collision with anchor point")
                
            # Unit vector pointing from toolhead to anchor
            ux = (anchor[0] - x) / L
            uy = (anchor[1] - y) / L
            
            # Projection of move direction onto cable direction
            u_dot_n = ux * nx + uy * ny
            
            # 1. Stepper Acceleration Constraint (Anchor proximity)
            # Limit carriage velocity to prevent centrifugal acceleration from exceeding limits:
            # v^2 <= (max_accel * L) / (1 - (u.n)^2)
            denom = 1.0 - u_dot_n**2
            if denom > 0.001:
                v_limit = math.sqrt((self.max_accel * L) / denom)
                if v_limit < move.max_velocity:
                    speed_factor = min(speed_factor, v_limit / move.max_velocity)

            # 2. Cable Tension Constraint (Top-center proximity & gravity loading)
            # Tension dynamic formula: T = m_eff * (g + a_y) * L / (2 * dy)
            vertical_drop = anchor[1] - y
            if vertical_drop <= 0.:
                raise move.move_error("Move goes above anchor height")
                
            # Max vertical acceleration allowed (in m/s^2)
            max_y_accel = (2.0 * self.max_cable_tension * vertical_drop) / (effective_mass * L) - gravity
            
            # Convert to mm/s^2 for Klipper planner
            max_y_accel_mm = max_y_accel * 1000.0
            
            if max_y_accel_mm <= 0.:
                raise move.move_error("Move exceeds static holding torque of motors (due to tension and cable weight)")
                
            # Scale move acceleration and speed if there is a vertical component
            if abs(ny) > 0.01:
                path_accel_limit = abs(max_y_accel_mm / ny)
                if path_accel_limit < self.max_accel:
                    accel_factor = min(accel_factor, path_accel_limit / self.max_accel)
                    # Velocity scales with the square root of the acceleration (v = sqrt(2*a*s))
                    speed_factor = min(speed_factor, math.sqrt(path_accel_limit / self.max_accel))

        # Apply the dynamically calculated constraints to Klipper's planner
        if speed_factor < 1.0 or accel_factor < 1.0:
            new_velocity = move.max_velocity * speed_factor
            new_accel = move.max_accel * accel_factor
            move.limit_speed(new_velocity, new_accel)

    def get_status(self, eventtime):
        # XXX - homed_checks and rail limits not implemented
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return DrawbotKinematics(toolhead, config)
