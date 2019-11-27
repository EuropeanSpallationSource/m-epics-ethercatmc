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

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition))

    # Assert that motor is homed
    def test_TC_1241(self):
        motor = self.motor
        tc_no = "TC-1241"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        lib.initializeMotorRecordSimulatorAxis(motor, '1241')


    # per90 UserPosition
    def test_TC_1242(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1242-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per90_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))
            self.assertEqual(res, globals.SUCCESS, 'move returned SUCCESS')

    # High soft limit in controller when using MoveAbs
    def test_TC_1243(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1243-high-soft-limit Moveabs"
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

            destination = self.hlm + 1
            timeout = lib.calcTimeOut(motor, destination, jvel)
            print('%s rbv=%f destination=%f timeout=%f' % (tc_no, rbv, destination, timeout))

            res = capv_lib.capvput(motor + '-MoveAbs', (destination) / mres)
            #if (res == None):
            #    print('%s caput -Moveabs res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            #else:
            #    print('%s caput -Moveabs res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            done = lib.waitForStartAndDone(motor, tc_no, timeout)

            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))
            success = lib.verifyPosition(motor, rbv)

            if (msta & lib.MSTA_BIT_PROBLEM):
                capv_lib.capvput(motor + '-ErrRst', 1)

            self.assertEqual(success, globals.SUCCESS, 'verifyPosition returned SUCCESS')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached Moveabs')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached Moveabs')
            self.assertEqual(0, miss,                              'DLY MISS not set Moveabs')

