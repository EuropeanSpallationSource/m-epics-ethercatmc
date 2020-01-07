#!/usr/bin/env python

# EPICS Single Motion application test script
#

import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    saved_DLY  = capv_lib.capvget(motor + '.DLY')
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')
    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    # Assert that motor is homed
    def test_TC_241(self):
        motor = self.motor
        tc_no = "TC-241"

        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')

    # Jog, wait for start, stop behind MR
    def test_TC_242(self):
        motor = self.motor
        tc_no = "TC-242-JOGF_stopped"
        print('%s' % tc_no)

        if (self.msta & lib.MSTA_BIT_HOMED):
            capv_lib.capvput(motor + '.DLY', 0)
            destination = self.per10_UserPosition
            res1 = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))

            capv_lib.capvput(motor + '.JOGF', 1)
            ret2 = lib.waitForStart(motor, tc_no, 2.0)

            time.sleep(1)
            capv_lib.capvput(motor + '-Stop', 1)
            ret3 = lib.waitForStop(motor, tc_no, 2.0)

            val = capv_lib.capvget(motor + '.VAL')

            try:
                res4 = lib.verifyPosition(motor, val)
            except:
                e = sys.exc_info()
                print(str(e))
                res4 = globals.FAIL


            capv_lib.capvput(motor + '.DLY', self.saved_DLY)

            self.assertEqual(res1, globals.SUCCESS, 'move returned SUCCESS')
            self.assertEqual(res4, globals.SUCCESS, 'VAL synched with RBV')
