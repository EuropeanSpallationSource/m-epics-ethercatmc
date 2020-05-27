#!/usr/bin/env python
#

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

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__))

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)

    msta = int(axisCom.get(".MSTA"))

    print(f"llm={llm:f} hlm={hlm:f}")

    # Assert that motor is homed
    def test_TC_1601(self):
        tc_no = "TC-1601"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # per10 UserPosition
    def test_TC_1602(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1602-10-percent-UserPosition"
            print(f"{tc_no}")
            done = self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.per10_UserPosition, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    # stress test; start and stop the  quickly..
    def test_TC_1603(self):
        tc_no = "TC-1603"

        msta = int(self.axisCom.get(".MSTA"))
        nErrorId = self.axisCom.get("-ErrId")
        for i in range(1, 10):
            if not (msta & self.axisMr.MSTA_BIT_PROBLEM):
                res = self.axisCom.put("-MoveAbs", self.per10_UserPosition + i * 10)
                time.sleep(0.01)
                res2 = self.axisCom.put(".STOP", 1)
                time.sleep(0.01)
                msta = int(self.axisCom.get(".MSTA"))
                nErrorId = self.axisCom.get("-ErrId")
                print(
                    "%s i=%d nErrorId=%x msta=%s"
                    % (tc_no, i, nErrorId, self.axisMr.getMSTAtext(msta))
                )

        if msta & self.axisMr.MSTA_BIT_PROBLEM:
            self.axisMr.resetAxis(tc_no)

        self.assertEqual(0, nErrorId, "nErrorId must be 0")
        self.assertEqual(
            0, msta & self.axisMr.MSTA_BIT_PROBLEM, "Problem bit must not be set"
        )
