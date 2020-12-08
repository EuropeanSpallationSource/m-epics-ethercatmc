#!/usr/bin/env python

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

filnam = "200xx.py"

###

polltime = 0.2


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    saved_HLM = axisCom.get(".HLM")
    saved_LLM = axisCom.get(".LLM")
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")

    # 10% UserPosition
    def test_TC_2001(self):
        tc_no = "2001"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
        self.axisCom.put(".CNEN", 1)
        self.axisMr.waitForPowerOn(tc_no, 8.0)
        destination = (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        self.axisMr.moveWait(tc_no, destination)

    # Jog, wait for start, power off, check error, reset error
    def test_TC_2002(self):
        tc_no = "2002"
        self.axisCom.put("-PwrAuto", 0)
        self.axisCom.put(".CNEN", 0)
        self.axisMr.waitForPowerOff(tc_no, 8.0)
        self.axisCom.put(".JOGF", 1)

        self.axisMr.jogDirection(tc_no, 1)
        msta_1 = int(self.axisCom.get(".MSTA", use_monitor=False))
        bError_2 = self.axisCom.get("-Err", use_monitor=False)
        nErrorId_2 = self.axisCom.get("-ErrId", use_monitor=False)
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Error bError_2={int(bError_2)} nErrorId_2={int(nErrorId_2)}")

        self.axisMr.resetAxis(tc_no)

        msta_3 = int(self.axisCom.get(".MSTA", use_monitor=False))
        bError_3 = self.axisCom.get("-Err", use_monitor=False)
        nErrorId_3 = self.axisCom.get("-ErrId", use_monitor=False)
        print(
            "%s Clean self.axisMr.MSTA_BIT_PROBLEM=%x msta_3=%x bError_3=%d nErrorId=%d"
            % (tc_no, self.axisMr.MSTA_BIT_PROBLEM, msta_3, bError_3, nErrorId_3)
        )

        # Loop until moving has stopped and error has been reseted
        counter = 7
        msta = msta_3
        nErrorId = nErrorId_3
        bError = bError_3
        while msta & self.axisMr.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0:
            time.sleep(polltime)
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} sleep counter = {int(counter)}")
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            bError = self.axisCom.get("-Err", use_monitor=False)
            nErrorId = self.axisCom.get("-ErrId", use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break
        # Restore the values we had before this test started
        self.axisCom.put(".CNEN", self.saved_CNEN)
        if self.saved_CNEN:
            self.axisMr.waitForPowerOn(tc_no + "restore", 8.0)
        else:
            self.axisMr.waitForPowerOff(tc_no + "restore", 3.0)

        self.axisCom.put("-PwrAuto", self.saved_PwrAuto)

        # Run all the asserts after we have restored the original state

        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Error msta_1={self.axisMr.getMSTAtext(msta_1)}")
        self.assertNotEqual(
            0,
            msta_1 & self.axisMr.MSTA_BIT_PROBLEM,
            "Error MSTA.Problem should be set)",
        )
        self.assertEqual(
            0,
            msta_1 & self.axisMr.MSTA_BIT_SLIP_STALL,
            "Error MSTA.Slip stall Error should not be set)",
        )
        self.assertEqual(0, msta_1 & self.axisMr.MSTA_BIT_MOVING, "Error MSTA.Moving)")

        self.assertNotEqual(0, bError_2, "bError")
        self.assertNotEqual(0, nErrorId_2, "nErrorId")

        self.assertEqual(0, msta & self.axisMr.MSTA_BIT_MOVING, "Clean MSTA.Moving)")
        self.assertEqual(0, bError, "bError")
        self.assertEqual(0, nErrorId, "nErrorId")
