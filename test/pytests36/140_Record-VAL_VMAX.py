#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def motorPositionTC(self, tc_no, destination, velocity):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    msta = int(self.axisCom.get(".MSTA"))
    if msta & self.axisMr.MSTA_BIT_HOMED:
        # self.axisCom.put('-DbgStrToLOG', "Start TC " + tc_no[0:20], wait=True);
        if velocity != self.velo:
            self.axisCom.put(".VELO", velocity)

        testPassed = self.axisMr.moveWait(tc_no, destination, throw=False)
        if velocity != self.velo:
            self.axisCom.put(".VELO", self.velo)

        UserPosition = self.axisCom.get(".RBV", use_monitor=False)
        print(f"{tc_no} postion={UserPosition:f} destination={destination:f}")
        testPassed = testPassed and self.axisMr.postMoveCheck(tc_no)
        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__), wait=True)

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    velo = axisCom.get(".VELO")
    vmax = axisCom.get(".VMAX")
    if vmax == 0.0:
        vmax = velo * 100.0
    msta = int(axisCom.get(".MSTA"))
    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition = round((1 * llm + 9 * hlm) / 10)

    def test_TC_1400(self):
        tc_no = "1400"
        self.axisMr.powerOnHomeAxis(tc_no)

    def test_TC_1401(self):
        tc_no = "1401"
        motorPositionTC(self, tc_no, self.llm, self.velo)

    def test_TC_1402(self):
        tc_no = "1402"
        motorPositionTC(self, tc_no, self.per10_UserPosition, self.velo)

    def test_TC_1403(self):
        tc_no = "1403"
        motorPositionTC(self, tc_no, self.per90_UserPosition, self.velo)

    def test_TC_1404(self):
        tc_no = "1404"
        motorPositionTC(self, tc_no, self.hlm, self.velo)

    def test_TC_1405(self):
        tc_no = "1405"
        motorPositionTC(self, tc_no, self.per90_UserPosition, self.vmax)

    def test_TC_1406(self):
        tc_no = "1406"
        motorPositionTC(self, tc_no, self.per10_UserPosition, self.vmax)

    def test_TC_1407(self):
        tc_no = "1407"
        motorPositionTC(self, tc_no, self.llm, self.vmax)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
