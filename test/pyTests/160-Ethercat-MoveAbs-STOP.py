#!/usr/bin/env python
#

import unittest
import os
import sys
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

    hlm  = capv_lib.capvget(motor + '.HLM')
    llm  = capv_lib.capvget(motor + '.LLM')
    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)

    msta = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f' % (llm, hlm))

    # Assert that motor is homed
    def test_TC_1601(self):
        motor = self.motor
        tc_no = "TC-1601"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')


    # per10 UserPosition
    def test_TC_1602(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1602-10-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.per10_UserPosition)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                   tc_no, UserPosition, self.per10_UserPosition, done))
            self.assertEqual(1, done,                'moveWait should return done')

    # stress test; start and stop the motor quickly..
    def test_TC_1603(self):
        motor = self.motor
        tc_no = "TC-1603"

        msta = int(capv_lib.capvget(motor + '.MSTA'))
        nErrorId = capv_lib.capvget(motor + '-ErrId')
        for i in range(1,10):
            if not (msta & lib.MSTA_BIT_PROBLEM):
                res = capv_lib.capvput(motor + '-MoveAbs', self.per10_UserPosition + i*10)
                time.sleep(0.01)
                res2 = capv_lib.capvput(motor + '.STOP', 1)
                time.sleep(0.01)
                msta = int(capv_lib.capvget(motor + '.MSTA'))
                nErrorId = capv_lib.capvget(motor + '-ErrId')
                print('%s i=%d nErrorId=%x msta=%s' % (
                    tc_no, i, nErrorId, lib.getMSTAtext(msta)))

        if (msta & lib.MSTA_BIT_PROBLEM):
            capv_lib.capvput(motor + '-ErrRst', 1)

        self.assertEqual(0, nErrorId,  'nErrorId must be 0')
        self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'Problem bit must not be set')
