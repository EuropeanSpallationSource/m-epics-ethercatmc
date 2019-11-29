#!/usr/bin/env python
#

import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per30_UserPosition  = round((7 * llm + 3 * hlm) / 10)

    range_postion    = hlm - llm
    saved_JVEL       = capv_lib.capvget(motor + '.JVEL')
    moving_velocity  = capv_lib.capvget(motor + '.VELO')
    acceleration     = capv_lib.capvget(motor + '.ACCL')
    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    if (saved_JVEL != 0.0):
        jogging_velocity = saved_JVEL;
    else:
        jogging_velocity = moving_velocity / 2.0;

    print('llm=%f hlm=%f per30_UserPosition=%f' % (llm, hlm, per30_UserPosition))
    print('saved_JVEL=%f jogging_velocity=%f' % (saved_JVEL, jogging_velocity))

    # Assert if motor is homed
    def test_TC_1321(self):
        motor = self.motor
        tc_no = "TC-1321"
        self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        self.assertNotEqual(self.llm, self.hlm, 'llm must be != hlm')
        self.assertNotEqual(0, self.jogging_velocity, 'JVEL or VELO must be != 0.0')


    # high limit switch
    def test_TC_1322(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED) and (self.jogging_velocity != 0.0):
            tc_no = "TC-1322-low-limit-switch"
            print('%s' % tc_no)
            old_high_limit = capv_lib.capvget(motor + '.HLM')
            old_low_limit = capv_lib.capvget(motor + '.LLM')
            capv_lib.capvput(motor + '.STOP', 1)
            #Go away from limit switch
            lib.movePosition(motor, tc_no, self.per30_UserPosition, self.moving_velocity, self.acceleration)

            if (self.saved_JVEL == 0.0) :
                capv_lib.capvput(motor + '.JVEL', self.jogging_velocity)
            jvel = capv_lib.capvget(motor + '.JVEL')

            #switch off the soft limits. Depending on the postion
            # low or high must be set to 0 first
            lib.setSoftLimitsOff(motor)
            done = lib.jogDirection(motor, tc_no, 0)

            # Get values, check them later
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            mstaE = int(capv_lib.capvget(motor + '.MSTA'))
            #Go away from limit switch
            lib.movePosition(motor, tc_no, old_high_limit, self.moving_velocity, self.acceleration)
            print('%s msta=%x lvio=%d' % (tc_no, mstaE, lvio))

            lib.setSoftLimitsOn(motor, old_low_limit, old_high_limit)
            capv_lib.capvput(motor + '.JVEL', self.saved_JVEL)

            self.assertEqual(True, done, 'done should be True after jogDirection')
            self.assertEqual(0, mstaE & lib.MSTA_BIT_PROBLEM,    'Schould be no Error MSTA.Problem at PLUS_LS')
            self.assertNotEqual(0, mstaE & lib.MSTA_BIT_MINUS_LS,'Minus hard limit switch should be active')
            self.assertEqual(0, mstaE & lib.MSTA_BIT_PLUS_LS, 'Plus hard limit switch should not ne active')


