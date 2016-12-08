#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
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
    print "motor=%s" % (motor)

    hlm = float(epics.caget(motor + '.HLM'))
    llm = float(epics.caget(motor + '.LLM'))
    range_postion    = hlm - llm
    homing_velocity  = epics.caget(motor + '.HVEL')
    acceleration     = epics.caget(motor + '.ACCL')

    # Home the motor
    def test_TC_100(self):
        motor = self.motor
        tc_no = "TC-100"
        print '%s Home the motor' % tc_no
        msta = int(epics.caget(motor + '.MSTA'))
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            epics.caput(motor + '.HOMR', 1)
        else:
            epics.caput(motor + '.HOMF', 1)
        time_to_wait = 30
        if self.range_postion > 0 and self.homing_velocity > 0:
            time_to_wait = 1 + self.range_postion / self.homing_velocity + 2 * self.acceleration

        # Homing velocity not implemented, wait longer
        time_to_wait = 180
        done = self.lib.waitForStartAndDone(motor, tc_no, time_to_wait)

        msta = int(epics.caget(motor + '.MSTA'))
        self.assertEqual(True, done, 'done = True')
        self.assertNotEqual(0, msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')



