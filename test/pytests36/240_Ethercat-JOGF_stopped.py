#!/usr/bin/env python

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    saved_DLY = axisCom.get(".DLY")
    msta = int(axisCom.get(".MSTA"))

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition = round((1 * llm + 9 * hlm) / 10)

    # Make sure that motor is homed
    def test_TC_2401(self):
        tc_no = "2401"

        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.homeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # Jog, wait for start, stop behind MR
    def test_TC_2402(self):
        tc_no = "2402"
        print(f"{tc_no}")

        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + tc_no[0:20])
            self.axisCom.put(".DLY", 0)
            destination = self.per10_UserPosition
            self.axisMr.moveWait(tc_no, destination)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f per10_UserPosition=%f"
                % (tc_no, UserPosition, self.per90_UserPosition)
            )

            self.axisCom.put(".JOGF", 1)
            self.axisMr.waitForStart(tc_no, 2.0)

            time.sleep(1)
            self.axisCom.put("-Stop", 1)
            self.axisMr.waitForStop(tc_no, 2.0)

            val = self.axisCom.get(".VAL")

            res4 = self.axisMr.verifyRBVinsideRDBD(tc_no, val)

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisCom.put("-DbgStrToLOG", "End " + tc_no[0:20])

            self.assertEqual(True, res4, "VAL synched with RBV")
