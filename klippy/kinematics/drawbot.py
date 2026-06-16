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

        name = 'stepper_a'
        stepper_config = config.getsection(name)
        s = stepper.PrinterStepper(stepper_config)
        self.steppers.append(s)
        a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz'])
        self.anchors.append(a)
        s.setup_itersolve('drawbot_stepper_alloc', *a)
        s.set_trapq(toolhead.get_trapq())
        toolhead.register_step_generator(s.generate_steps)

        name = 'stepper_b'
        stepper_config = config.getsection(name)
        s = stepper.PrinterStepper(stepper_config)
        self.steppers.append(s)
        a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz'])
        self.anchors.append(a)
        s.setup_itersolve('drawbot_stepper_alloc', *a)
        s.set_trapq(toolhead.get_trapq())
        toolhead.register_step_generator(s.generate_steps)

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
        # XXX - boundary checks and speed limits not implemented
        pass

    def get_status(self, eventtime):
        # XXX - homed_checks and rail limits not implemented
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return DrawbotKinematics(toolhead, config)
