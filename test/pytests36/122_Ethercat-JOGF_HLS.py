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

direction = 1


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
    def test_TC_1221(self):
        tc_no = tc_no_base + 1
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        self.axisMr.powerOnHomeAxis(tc_no)
        self.axisMr.setSoftLimitsOn(tc_no, initAbsMinMax=True)
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)

    # high limit switch
    def test_TC_1222(self):
        tc_no = tc_no_base + 2
        moveIntoLimitSwitch(self, tc_no, movingMethod="JOG")

    # high limit switch, disabling softlimts after the JOG
    # had been started. This is not supported by our MCU SW
    #def test_TC_1223(self):
    #    tc_no = tc_no_base + 3
    #    moveIntoLimitSwitch(self, tc_no, movingMethod="JOG", paramWhileMove=True)

    # high limit switch via moveVel
    # had been started
    def test_TC_1224(self):
        tc_no = tc_no_base + 4
        moveIntoLimitSwitch(self, tc_no, movingMethod="MoveVel")

    # low limit switch via moveVel and "infinite" Soft limit
    def test_TC_1225(self):
        tc_no = tc_no_base + 5
        moveIntoLimitSwitch(
            self,
            tc_no,
            movingMethod="MoveVel",
            doDisableSoftLimit=False,
            setInfiniteSoftLimit=True,
        )
