#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
from motor_lib import motor_lib
###

class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per30_UserPosition  = round((7 * llm + 3 * hlm) / 10)

    range_postion    = hlm - llm
    jogging_velocity = epics.caget(motor + '.JVEL')
    moving_velocity  = epics.caget(motor + '.VELO')
    acceleration     = epics.caget(motor + '.ACCL')
    msta             = int(epics.caget(motor + '.MSTA'))

    print 'llm=%f hlm=%f per30_UserPosition=%f' % (llm, hlm, per30_UserPosition)

    # Assert if motor is homed
    def test_TC_1321(self):
        motor = self.motor
        tc_no = "TC-1321"
        self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.assertNotEqual(self.llm, self.hlm, 'llm must be != hlm')



    # high limit switch
    def test_TC_1322(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1322-low-limit-switch"
            print '%s' % tc_no
            old_high_limit = epics.caget(motor + '.HLM')
            old_low_limit = epics.caget(motor + '.LLM')
            epics.caput(motor + '.STOP', 1)
            #Go away from limit switch
            self.lib.movePosition(motor, tc_no, self.per30_UserPosition, self.moving_velocity, self.acceleration)
            #switch off the soft limits. Depending on the postion
            # low or high must be set to 0 first
            if epics.caget(motor + '.RBV') > 0:
                epics.caput(motor + '.LLM', 0.0)
                epics.caput(motor + '.HLM', 0.0)
            else:
                epics.caput(motor + '.HLM', 0.0)
                epics.caput(motor + '.LLM', 0.0)

            epics.caput(motor + '.JOGR', 1, wait=True)
            # Get values, check them later
            lvio = int(epics.caget(motor + '.LVIO'))
            mstaE = int(epics.caget(motor + '.MSTA'))
            #Go away from limit switch
            self.lib.movePosition(motor, tc_no, old_high_limit, self.moving_velocity, self.acceleration)
            print '%s msta=%x lvio=%d' % (tc_no, mstaE, lvio)

            epics.caput(motor + '.LLM', old_low_limit)
            epics.caput(motor + '.HLM', old_high_limit)

            #self.assertEqual(0, lvio, 'LVIO == 0')
            self.assertEqual(0, mstaE & self.lib.MSTA_BIT_PROBLEM,    'No Error MSTA.Problem at PLUS_LS')
            self.assertNotEqual(0, mstaE & self.lib.MSTA_BIT_MINUS_LS,   'Minus hard limit switch not active')
            self.assertEqual(0, mstaE & self.lib.MSTA_BIT_PLUS_LS, 'Plus hard limit switch active')


