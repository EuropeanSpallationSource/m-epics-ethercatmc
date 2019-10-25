#!/usr/bin/env python
#

import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f per10_UserPosition=%f' % (llm, hlm, per10_UserPosition))

    # Assert that motor is homed
    def test_TC_1341(self):
        motor = self.motor
        tc_no = "TC-1341"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        lib.initializeMotorRecordSimulatorAxis(motor, '1341')


    # per10 UserPosition
    def test_TC_1342(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1342-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per10_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per10_UserPosition))
            self.assertEqual(res, globals.SUCCESS, 'move returned SUCCESS')

    # Low soft limit in controller when using MoveAbs
    def test_TC_1343(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1343-low-soft-limit Moveabs"
            print('%s' % tc_no)
            drvUseEGU = capv_lib.capvget(motor + '-DrvUseEGU-RB')
            if drvUseEGU == 1:
                mres = 1.0
            else:
                mres = capv_lib.capvget(motor + '.MRES')

            rbv = capv_lib.capvget(motor + '.RBV')

            jar = capv_lib.capvget(motor + '.JAR')
            capv_lib.capvput(motor + '-ACCS', jar/mres)

            jvel = capv_lib.capvget(motor + '.JVEL')
            capv_lib.capvput(motor + '-VELO', jvel/mres)

            jvel = capv_lib.capvget(motor + '.JVEL')

            res = capv_lib.capvput(motor + '-MoveAbs', (self.llm - 1) / mres)
            # TODO: The -MoveVel PV is not always there ?
            # Investigations needed
            #if (res == None):
            #    print('%s caput -Moveabs res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            #else:
            #    print('%s caput -Moveabs res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            time_to_wait = 180
            done = lib.waitForStartAndDone(motor, tc_no, time_to_wait)

            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))
            success = lib.verifyPosition(motor, rbv)

            if (msta & lib.MSTA_BIT_PROBLEM):
                capv_lib.capvput(motor + '-ErrRst', 1)

            self.assertEqual(success, globals.SUCCESS, 'verifyPosition returned SUCCESS')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached Moveabs')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached Moveabs')
            self.assertEqual(0, miss,                              'DLY MISS not set Moveabs')

