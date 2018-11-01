#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
from motor_lib import motor_lib
from motor_globals import motor_globals
###

class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(epics.caget(motor + '.MSTA'))

    print 'llm=%f hlm=%f per10_UserPosition=%f' % (llm, hlm, per10_UserPosition)

    # Assert that motor is homed
    def test_TC_1341(self):
        motor = self.motor
        tc_no = "TC-1341"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.lib.initializeMotorRecordSimulatorAxis(motor, '1341')


    # per10 UserPosition
    def test_TC_1342(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1342-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per10_UserPosition
            res = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per10_UserPosition)
            self.assertEqual(res, self.__g.SUCCESS, 'move returned SUCCESS')

    # Low soft limit in controller when using MoveAbs
    def test_TC_1343(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1343-low-soft-limit Moveabs"
            print '%s' % tc_no
            readbackEGU = epics.caget(motor + '-RB-EGU-RB')
            if readbackEGU == 1:
                mres = 1.0
            else:
                mres = epics.caget(motor + '.MRES')

            rbv = epics.caget(motor + '.RBV')

            jar = epics.caget(motor + '.JAR')
            epics.caput(motor + '-ACSS', jar/mres)

            jvel = epics.caget(motor + '.JVEL')
            epics.caput(motor + '-VELO', jvel/mres)

            jvel = epics.caget(motor + '.JVEL')

            res = epics.caput(motor + '-MoveAbs', (self.llm - 1) / mres)
            if (res == None):
                print '%s caput -Moveabs res=None' % (tc_no)
                self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            else:
                print '%s caput -Moveabs res=%d' % (tc_no, res)
                self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            time_to_wait = 180
            done = self.lib.waitForStartAndDone(motor, tc_no, time_to_wait)

            msta = int(epics.caget(motor + '.MSTA'))
            miss = int(epics.caget(motor + '.MISS'))
            success = self.lib.verifyPosition(motor, rbv)

            if (msta & self.lib.MSTA_BIT_PROBLEM):
                epics.caput(motor + '-ErrRst', 1)

            self.assertEqual(success, self.__g.SUCCESS, 'verifyPosition returned SUCCESS')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached Moveabs')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached Moveabs')
            self.assertEqual(0, miss,                              'DLY MISS not set Moveabs')

