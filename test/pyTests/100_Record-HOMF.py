#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

class Test(unittest.TestCase):

    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")
    print( "motor=%s" % (motor))
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])


    hlm = float(capv_lib.capvget(motor + '.HLM'))
    llm = float(capv_lib.capvget(motor + '.LLM'))

    range_postion    = hlm - llm
    homing_velocity  = capv_lib.capvget(motor + '.HVEL')
    acceleration     = capv_lib.capvget(motor + '.ACCL')

    # Home the motor
    def test_TC_100(self):
        motor = self.motor
        tc_no = "TC-100"
        print( '%s Home the motor' % tc_no)
        msta = int(capv_lib.capvget(motor + '.MSTA'))
        if (msta & lib.MSTA_BIT_PLUS_LS):
            capv_lib.capvput(motor + '.HOMR', 1)
        else:
            capv_lib.capvput(motor + '.HOMF', 1)
        time_to_wait = 30
        if self.range_postion > 0 and self.homing_velocity > 0:
            time_to_wait = 1 + self.range_postion / self.homing_velocity + 2 * self.acceleration

        # Homing velocity not implemented, wait longer
        time_to_wait = 180
        done = lib.waitForStartAndDone(motor, tc_no, time_to_wait)

        msta = int(capv_lib.capvget(motor + '.MSTA'))
        self.assertEqual(True, done, 'done = True')
        self.assertNotEqual(0, msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')



