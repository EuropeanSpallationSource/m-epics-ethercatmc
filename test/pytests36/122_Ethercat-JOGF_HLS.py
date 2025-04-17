#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
tc_no_base = int(os.path.basename(__file__)[0:3]) * 10
###

direction = 1


def lineno():
    return inspect.currentframe().f_back.f_lineno


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
        assert self.axisMr.moveIntoLimitSwitchFromTestCase(
            tc_no, direction=direction, movingMethod="JOG"
        ) is True

    # high limit switch via DVAL
    def test_TC_1223(self):
        tc_no = tc_no_base + 3
        assert self.axisMr.moveIntoLimitSwitchFromTestCase(
            tc_no, movingMethod="DVAL", setDLYfield=1.0
        ) is True

    # high limit switch via moveVel
    # had been started
    def test_TC_1224(self):
        tc_no = tc_no_base + 4
        assert self.axisMr.moveIntoLimitSwitchFromTestCase(
            tc_no, direction=direction, movingMethod="MoveVel"
        ) is True

    # low limit switch via moveVel and "infinite" Soft limit
    def test_TC_1225(self):
        tc_no = tc_no_base + 5
        assert self.axisMr.moveIntoLimitSwitchFromTestCase(
            tc_no,
            movingMethod="MoveVel",
            doDisableSoftLimit=False,
            setInfiniteSoftLimit=True,
        ) is True

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
