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

direction = 0


def moveIntoLimitSwitch(
    self,
    tc_no,
    movingMethod="",
    paramWhileMove=False,
    doDisableSoftLimit=True,
    setInfiniteSoftLimit=False,
):
    msta = int(self.axisCom.get(".MSTA"))
    if msta & self.axisMr.MSTA_BIT_HOMED:
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        passed = self.axisMr.moveIntoLS(
            tc_no=tc_no,
            direction=direction,
            movingMethod=movingMethod,
            paramWhileMove=paramWhileMove,
            doDisableSoftLimit=doDisableSoftLimit,
            setInfiniteSoftLimit=setInfiniteSoftLimit,
        )
        if passed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}")

    # Make sure that motor is homed
    def test_TC_1321(self):
        tc_no = tc_no_base + 1
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        self.axisMr.powerOnHomeAxis(tc_no)
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)

    # low limit switch
    def test_TC_1322(self):
        tc_no = tc_no_base + 2
        moveIntoLimitSwitch(self, tc_no, movingMethod="JOG")

    # low limit switch, disabling softlimts after the JOG
    # had been started. This is not supported by our MCU SW
    #def test_TC_1323(self):
    #    tc_no = tc_no_base + 3
    #    moveIntoLimitSwitch(self, tc_no, movingMethod="JOG", paramWhileMove=True)

    # low limit switch via moveVel
    def test_TC_1324(self):
        tc_no = tc_no_base + 4
        moveIntoLimitSwitch(self, tc_no, movingMethod="MoveVel")

    # low limit switch via moveVel and "infinite" Soft limit
    def test_TC_1325(self):
        tc_no = tc_no_base + 5
        moveIntoLimitSwitch(
            self,
            tc_no,
            movingMethod="MoveVel",
            doDisableSoftLimit=False,
            setInfiniteSoftLimit=True,
        )
