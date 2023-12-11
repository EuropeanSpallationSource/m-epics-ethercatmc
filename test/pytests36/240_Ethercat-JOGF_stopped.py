#!/usr/bin/env python

import datetime
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom


filnam = os.path.basename(__file__)[0:3]
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

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
        self.axisMr.powerOnHomeAxis(tc_no)

    # Jog, wait for start, stop behind MR
    def test_TC_2402(self):
        tc_no = "2402"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")

        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.putDbgStrToLOG("Start " + tc_no[0:20], wait=True)
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
            self.axisCom.putDbgStrToLOG("End " + tc_no[0:20], wait=True)

            self.assertEqual(True, res4, "VAL synched with RBV")

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
