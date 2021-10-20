#!/usr/bin/env python
#

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

###


def motorPositionTC(self, tc_no, destination, velocity):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    msta = int(self.axisCom.get(".MSTA"))
    if msta & self.axisMr.MSTA_BIT_HOMED:
        # self.axisCom.put('-DbgStrToLOG', "Start TC " + tc_no[0:20]);
        if velocity != self.velo:
            self.axisCom.put(".VELO", velocity)

        self.axisMr.moveWait(tc_no, destination)
        if velocity != self.velo:
            self.axisCom.put(".VELO", self.velo)

        UserPosition = self.axisCom.get(".RBV", use_monitor=False)
        print(f"{tc_no} postion={UserPosition:f} destination={destination:f}")
        testPassed = self.axisMr.postMoveCheck(tc_no)
        if testPassed:
            self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
        else:
            self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
        assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__))

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
