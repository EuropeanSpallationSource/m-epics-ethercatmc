#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
import sys
import time
from AxisMr import AxisMr
from AxisCom import AxisCom


filnam = os.path.basename(__file__)[0:3]


def lineno():
    return inspect.currentframe().f_back.f_lineno


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__), wait=True)

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)

    msta = int(axisCom.get(".MSTA"))

    print(f"llm={llm:f} hlm={hlm:f}")

    # Make sure that motor is homed
    def test_TC_1601(self):
        tc_no = "1601"
        self.axisMr.powerOnHomeAxis(tc_no)

    # per10 UserPosition
    def test_TC_1602(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1602"
            print(f"{tc_no}")
            self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f"
                % (tc_no, UserPosition, self.per10_UserPosition)
            )

    # stress test; start and stop the  quickly..
    def test_TC_1603(self):
        tc_no = "1603"

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

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
