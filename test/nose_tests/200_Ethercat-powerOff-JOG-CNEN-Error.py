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
import math
import time
from motor_lib import motor_lib
###

polltime = 0.2


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")
    #motmotor   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pvmotor = epics.PV(os.getenv("TESTEDMOTORAXIS"))
    pv_Err   = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")


    saved_HLM = epics.caget(motor + '.HLM')
    saved_LLM = epics.caget(motor + '.LLM')
    saved_CNEN = epics.caget(motor + '.CNEN')
    saved_PwrAuto = epics.caget(motor + '-PwrAuto')

    # 10% UserPosition
    def test_TC_2001(self):
        motor = self.motor
        tc_no = "TC-2001-10-percent-dialPosition"
        print '%s' % tc_no
        epics.caput(motor + '.CNEN', 1)
        self.lib.waitForPowerOn(motor, tc_no, 8.0)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(motor + '.VAL', destination, wait=True)


    # Jog, wait for start, power off, check error, reset error
    def test_TC_2002(self):
        motor = self.motor
        tc_no = "TC-2002-JOG-_Enable"
        epics.caput(motor + '-PwrAuto', 0)
        epics.caput(motor + '.CNEN', 0)
        self.lib.waitForPowerOff(motor, tc_no, 8.0)
        epics.caput(motor + '.JOGF', 1)
        # dummy wait
        ret_0 = self.lib.waitForStart(motor, tc_no, 2.0)

        ret_1 = self.lib.waitForStop(motor, tc_no, 10.0)

        msta_1 = int(epics.caget(motor + '.MSTA', use_monitor=False))
        mstaErr_1 = int(epics.caget(motor + '.MSTA', use_monitor=False))

        bError_2   = self.pv_Err.get(use_monitor=False)
        nErrorId_2 = self.pv_nErrorId.get(use_monitor=False)
        print '%s Error bError_2=%d nErrorId_2=%d' % (tc_no, bError_2, nErrorId_2)

        self.pv_nErrRst.put(1)

        msta_3 = int(self.pv_MSTA.get(use_monitor=False))
        bError_3   = self.pv_Err.get(use_monitor=False)
        nErrorId_3 = self.pv_nErrorId.get(use_monitor=False)
        print '%s Clean self.lib.MSTA_BIT_PROBLEM=%x msta_3=%x bError_3=%d nErrorId=%d' % (tc_no, self.lib.MSTA_BIT_PROBLEM, msta_3, bError_3, nErrorId_3)

        # Loop until moving has stopped and error has been reseted
        counter = 7
        msta = msta_3
        nErrorId = nErrorId_3
        bError = bError_3
        while (msta & self.lib.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0):
            time.sleep(polltime)
            print '%s sleep counter = %d' % (tc_no, counter)
            msta = int(self.pv_MSTA.get(use_monitor=False))
            bError   = self.pv_Err.get(use_monitor=False)
            nErrorId = self.pv_nErrorId.get(use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break
        # Restore the values we had before this test started
        epics.caput(motor + '.CNEN', self.saved_CNEN)
        if self.saved_CNEN:
            self.lib.waitForPowerOn(motor, tc_no + "restore", 8.0)
        else:
            self.lib.waitForPowerOff(motor, tc_no+ "restore", 3.0)

        epics.caput(motor + '-PwrAuto', self.saved_PwrAuto)

        # Run all the asserts after we have restored the original state
        self.assertEqual(True, ret_1, 'waitForStop return True')

        print '%s Error mstaErr_1=%s' % (tc_no, self.lib.getMSTAtext(mstaErr_1))
        self.assertNotEqual(0, msta_1 & self.lib.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, msta_1 & self.lib.MSTA_BIT_SLIP_STALL, 'Error MSTA.Slip stall Error should not be set)')
        self.assertEqual(0, msta_1 & self.lib.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        self.assertNotEqual(0, bError_2,   'bError')
        self.assertNotEqual(0, nErrorId_2, 'nErrorId')

        self.assertEqual(0, msta & self.lib.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')


