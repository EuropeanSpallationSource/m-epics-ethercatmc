#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

polltime = 0.2


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")

    saved_HLM = capv_lib.capvget(motor + '.HLM')
    saved_LLM = capv_lib.capvget(motor + '.LLM')
    saved_CNEN = capv_lib.capvget(motor + '.CNEN')
    saved_PwrAuto = capv_lib.capvget(motor + '-PwrAuto')

    # 10% UserPosition
    def test_TC_2001(self):
        motor = self.motor
        tc_no = "TC-2001-10-percent-dialPosition"
        print('%s' % tc_no)
        capv_lib.capvput(motor + '.CNEN', 1)
        lib.waitForPowerOn(motor, tc_no, 8.0)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        capv_lib.capvput(motor + '.VAL', destination, wait=True)


    # Jog, wait for start, power off, check error, reset error
    def test_TC_2002(self):
        motor = self.motor
        tc_no = "TC-2002-JOG-_Enable"
        capv_lib.capvput(motor + '-PwrAuto', 0)
        capv_lib.capvput(motor + '.CNEN', 0)
        lib.waitForPowerOff(motor, tc_no, 8.0)
        capv_lib.capvput(motor + '.JOGF', 1)

        ret_1 = lib.jogDirection(motor, tc_no, 1)
        msta_1 = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
        bError_2   = capv_lib.capvget(motor + '-Err', use_monitor=False)
        nErrorId_2 = capv_lib.capvget(motor + '-ErrId', use_monitor=False)
        print('%s Error bError_2=%d nErrorId_2=%d' % (tc_no, bError_2, nErrorId_2))

        lib.resetAxis(motor, tc_no)

        msta_3 = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
        bError_3   = capv_lib.capvget(motor + '-Err', use_monitor=False)
        nErrorId_3 = capv_lib.capvget(motor + '-ErrId', use_monitor=False)
        print('%s Clean lib.MSTA_BIT_PROBLEM=%x msta_3=%x bError_3=%d nErrorId=%d' % (tc_no, lib.MSTA_BIT_PROBLEM, msta_3, bError_3, nErrorId_3))

        # Loop until moving has stopped and error has been reseted
        counter = 7
        msta = msta_3
        nErrorId = nErrorId_3
        bError = bError_3
        while (msta & lib.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0):
            time.sleep(polltime)
            print('%s sleep counter = %d' % (tc_no, counter))
            msta = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
            bError   = capv_lib.capvget(motor + '-Err', use_monitor=False)
            nErrorId = capv_lib.capvget(motor + '-ErrId', use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break
        # Restore the values we had before this test started
        capv_lib.capvput(motor + '.CNEN', self.saved_CNEN)
        if self.saved_CNEN:
            lib.waitForPowerOn(motor, tc_no + "restore", 8.0)
        else:
            lib.waitForPowerOff(motor, tc_no+ "restore", 3.0)

        capv_lib.capvput(motor + '-PwrAuto', self.saved_PwrAuto)

        # Run all the asserts after we have restored the original state
        self.assertEqual(True, ret_1, 'waitForStop return True')

        print('%s Error msta_1=%s' % (tc_no, lib.getMSTAtext(msta_1)))
        self.assertNotEqual(0, msta_1 & lib.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, msta_1 & lib.MSTA_BIT_SLIP_STALL, 'Error MSTA.Slip stall Error should not be set)')
        self.assertEqual(0, msta_1 & lib.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        self.assertNotEqual(0, bError_2,   'bError')
        self.assertNotEqual(0, nErrorId_2, 'nErrorId')

        self.assertEqual(0, msta & lib.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')


