#!/usr/bin/env python

# EPICS Single Motion application test script
#

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
from motor_globals import motor_globals
###

class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")
    epics.caput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    lib.initializeMotorRecordSimulatorAxis(motor, '240')
    saved_DLY  = epics.caget(motor + '.DLY')
    msta             = int(epics.caget(motor + '.MSTA'))

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')
    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    # Assert that motor is homed
    def test_TC_241(self):
        motor = self.motor
        tc_no = "TC-241"

        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')

    # Jog, wait for start, stop behind MR
    def test_TC_242(self):
        motor = self.motor
        tc_no = "TC-242-JOGF_stopped"
        print('%s' % tc_no)

        if (self.msta & self.lib.MSTA_BIT_HOMED):
            epics.caput(motor + '.DLY', 0)
            destination = self.per10_UserPosition
            res1 = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))

            epics.caput(motor + '.JOGF', 1)
            ret2 = self.lib.waitForStart(motor, tc_no, 2.0)

            time.sleep(1)
            epics.caput(motor + '-Stop', 1)
            ret3 = self.lib.waitForStop(motor, tc_no, 2.0)

            val = epics.caget(motor + '.VAL')

            try:
                res4 = self.lib.verifyPosition(motor, val)
            except:
                e = sys.exc_info()
                print(str(e))
                res4 = self.__g.FAIL


            epics.caput(motor + '.DLY', self.saved_DLY)

            self.assertEqual(res1, self.__g.SUCCESS, 'move returned SUCCESS')
            self.assertEqual(res4, self.__g.SUCCESS, 'VAL synched with RBV')
