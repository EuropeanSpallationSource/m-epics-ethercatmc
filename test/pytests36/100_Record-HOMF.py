#!/usr/bin/env python

#
# Test homing via EPICS motorRecord
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

polltime = 0.2


def lineno():
    return inspect.currentframe().f_back.f_lineno


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # Home the motor
    def test_TC_100(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        tc_no = "100"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
        )
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

        # Get values to be able calculate a timeout
        range_postion = axisCom.get(".HLM") - axisCom.get(".LLM")
        hvel = float(axisCom.get(".HVEL"))
        accl = axisCom.get(".ACCL")
        msta = int(axisCom.get(".MSTA"))

        self.axisMr.setCNENandWait(tc_no, 1)
        # Calculate the timeout, based on the driving range
        if range_postion > 0 and hvel > 0:
            time_to_wait = 1 + 2 * range_postion / hvel + 2 * accl
        else:
            time_to_wait = 180
        print(f"range_postion={range_postion} hvel={hvel} time_to_wait={time_to_wait}")

        # If we are sitting on the High limit switch, use HOMR
        if msta & axisMr.MSTA_BIT_PLUS_LS:
            axisCom.put(".HOMR", 1)
        else:
            axisCom.put(".HOMF", 1)
        axisMr.waitForStartAndDone(tc_no, time_to_wait)

        # Wait a little bit, motor/master reports "done" already when hitting a limit switch
        # Give the "homed" bit a chance to ripple through the poller
        while time_to_wait > 0:
            msta = int(axisCom.get(".MSTA"))
            print(f"msta={self.axisMr.getMSTAtext(msta)}")
            time.sleep(polltime)
            time_to_wait -= polltime
            if msta & axisMr.MSTA_BIT_HOMED:
                time_to_wait = 0
        testPassed = msta & axisMr.MSTA_BIT_HOMED
        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
