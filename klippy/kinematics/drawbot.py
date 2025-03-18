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
        a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz']) #TODO use motor separation to infer x, set y and z to 0
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
        acoords = list(zip(*self.anchors))
        self.axes_min = toolhead.Coord(*[min(a) for a in acoords], e=0.)
        self.axes_max = toolhead.Coord(*[max(a) for a in acoords], e=0.)
        self.set_position([0., 0., 0.], "")
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        # https://en.wikipedia.org/wiki/Two-center_bipolar_coordinates
        anchor_left, anchor_right = self.anchors[:2]
        width = anchor_right[0] - anchor_left[0]
        stepper_left, stepper_right = self.steppers[:2]

        x = (stepper_left*stepper_left - stepper_right*stepper_right) / (4 * width)
        chunk = stepper_left*stepper_left - stepper_right*stepper_right + 4 * width*width
        y = math.sqrt(16 * width*width * stepper_left*stepper_left - chunk*chunk) / (4 * width)
        return tuple(x, y, 0)
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
