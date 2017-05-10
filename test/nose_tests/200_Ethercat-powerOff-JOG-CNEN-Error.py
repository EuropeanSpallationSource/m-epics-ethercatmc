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
    m1 = os.getenv("TESTEDMOTORAXIS")
    #motm1   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pvm1 = epics.PV(os.getenv("TESTEDMOTORAXIS"))
    pv_Err   = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")


    saved_HLM = epics.caget(m1 + '.HLM')
    saved_LLM = epics.caget(m1 + '.LLM')
    saved_Enable = epics.caget(m1 + '-En')

    # 10% UserPosition
    def test_TC_201(self):
        motor = self.m1
        tc_no = "TC-201-10-percent-dialPosition"
        print '%s' % tc_no
        epics.caput(motor + '-En', 1)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(motor + '.VAL', destination, wait=True)


    # Jog, wait for start, power off, check error, reset error
    def test_TC_202(self):
        motor = self.m1
        tc_no = "TC-202-JOG-_Enable"
        epics.caput(motor + '-En', 0)
        epics.caput(motor + '.JOGF', 1)
        ret = self.lib.waitForStart(motor, tc_no, 2.0)
        # dummy wait

        ret = self.lib.waitForStop(motor, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')

        msta = int(epics.caget(motor + '.MSTA', use_monitor=False))
        print '%s Error msta=%x' % (tc_no, msta)
        self.assertNotEqual(0, msta & self.lib.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, msta & self.lib.MSTA_BIT_SLIP_STALL, 'Error MSTA.Slip stall Error should not be set)')
        self.assertEqual(0, msta & self.lib.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Error bError=%d nErrorId=%d' % (tc_no, bError, nErrorId)

        self.assertNotEqual(0, bError,   'bError')
        self.assertNotEqual(0, nErrorId, 'nErrorId')

        self.pv_nErrRst.put(1)

        msta = int(self.pv_MSTA.get(use_monitor=False))
        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Clean self.lib.MSTA_BIT_PROBLEM=%x msta=%x bError=%d nErrorId=%d' % (tc_no, self.lib.MSTA_BIT_PROBLEM, msta, bError, nErrorId)

        counter = 7
        while (msta & self.lib.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0):
            time.sleep(polltime)
            print '%s sleep counter = %d' % (tc_no, counter)
            msta = int(self.pv_MSTA.get(use_monitor=False))
            bError   = self.pv_Err.get(use_monitor=False)
            nErrorId = self.pv_nErrorId.get(use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break

        self.assertEqual(0, msta & self.lib.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')


