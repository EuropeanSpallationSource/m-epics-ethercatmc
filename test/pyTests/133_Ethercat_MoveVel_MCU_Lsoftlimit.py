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
    jvel = capv_lib.capvget(motor + '.JVEL')

    margin = 1.0
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos    = llm + jvel + margin

    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f jog_start_pos=%f' % (llm, hlm, jog_start_pos))

    # Assert that motor is homed
    def test_TC_1331(self):
        motor = self.motor
        tc_no = "TC-1331"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')


    # per10 UserPosition
    def test_TC_1332(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1332-10-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                   tc_no, UserPosition, self.jog_start_pos, done))
            self.assertEqual(1, done,                'moveWait should return done')

    # Low soft limit in controller when using MoveVel
    def test_TC_1333(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1333-low-soft-limit MoveVel"
            print('%s' % tc_no)

            jar = capv_lib.capvget(motor + '.JAR')
            capv_lib.capvput(motor + '-ACCS', jar)

            rbv = capv_lib.capvget(motor + '.RBV')
            destination = capv_lib.capvget(motor + '.LLM') - 1
            jvel = capv_lib.capvget(motor + '.JVEL')
            timeout = lib.calcTimeOut(motor, destination, jvel)
            print('%s rbv=%f destination=%f timeout=%f' % (tc_no, rbv, destination, timeout))
            res = capv_lib.capvput(motor + '-MoveVel', 0 - jvel)
            # TODO: The -MoveVel PV is not always there ?
            # Investigations needed
            #if (res == None):
            #    print('%s caput -MoveVel res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -MoveVel retuned not None. PV not found ?')
            #else:
            #    print('%s caput -MoveVel res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -MoveVel returned 1')

            done = lib.waitForStartAndDone(motor, tc_no, timeout)

            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            if (msta & lib.MSTA_BIT_PROBLEM):
                capv_lib.capvput(motor + '-ErrRst', 1)

            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached MoveVel')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached MoveVel')
            self.assertEqual(0, miss,                         'DLY MISS not set MoveVel')


    # per10 UserPosition
    def test_TC_1334(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1234-q0-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                   tc_no, UserPosition, self.jog_start_pos, done))
            self.assertEqual(1, done,                'moveWait should return done')

    # Low soft limit in controller when using MoveAbs
    def test_TC_1335(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1235-low-soft-limit Moveabs"
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

            destination = self.llm - 1
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
            self.assertEqual(0, miss,                         'DLY MISS not set Moveabs')

