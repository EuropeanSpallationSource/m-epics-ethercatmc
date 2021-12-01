#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
tc_no_base = int(os.path.basename(__file__)[0:3]) * 10
###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}")

    # Make sure that motor is homed
    def test_TC_1221(self):
        tc_no = tc_no_base + 1
        self.axisMr.powerOnHomeAxis(tc_no)

    # high limit switch
    def test_TC_1222(self):
        tc_no = tc_no_base + 2
        direction = 1
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)), wait=True)
            passed = self.axisMr.moveIntoLS(tc_no=tc_no, direction=direction)
            if passed:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
            else:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
            assert passed

    # high limit switch, disabling softlimts after the JOG
    # had been started
    def test_TC_1223(self):
        tc_no = tc_no_base + 3
        direction = 1
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)), wait=True)
            passed = self.axisMr.moveIntoLS(
                tc_no=tc_no, direction=direction, paramWhileMove=True
            )
            if passed:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
            else:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
            assert passed
