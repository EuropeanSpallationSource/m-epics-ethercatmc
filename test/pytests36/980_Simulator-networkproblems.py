#!/usr/bin/env python
#
#
# Test the communication robustness in the IOC
# Send "corrupted" resposense from the simulator to the IOC
# The IOC should close the connection, and open a new one
# After this, the initial poll must be performed
# To verify this, we manipulate the "abs limit" before each test case
# The "abs limit" is read via PILS, but only once
# When the re-reading is done, the new value should be in "-CfgPMAX-RB"
# When the re-reading is not done, we see the "old" value,
# and the test case fails
# What the different failures mean, is defined in the simulator


import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

filnam = "9800xx.py"
###

maxDelta = 0.5
timeout = 3.0


def lineno():
    return inspect.currentframe().f_back.f_lineno


def simulateNetworkProblem(self, tc_no, simulatedNetworkProblem):
    myHighHardLimitPos = tc_no
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    self.axisMr.setValueOnSimulator(tc_no, "fHighHardLimitPos", myHighHardLimitPos)
    self.axisMr.setValueOnSimulator(
        tc_no, "ads.simulatedNetworkProblem", simulatedNetworkProblem
    )
    passed = self.axisMr.waitForValueChanged(
        tc_no, "-CfgPMAX-RB", myHighHardLimitPos, maxDelta, timeout
    )
    if passed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
    assert passed


def invalidNetworkProblem(self, tc_no, simulatedNetworkProblem):
    myHighHardLimitPos = tc_no
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    self.axisMr.setValueOnSimulator(tc_no, "fHighHardLimitPos", myHighHardLimitPos)
    passed = False
    try:
        self.axisMr.setValueOnSimulator(
            tc_no, "ads.simulatedNetworkProblem", simulatedNetworkProblem
        )
    except Exception as ex:
        e_str = str(ex)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: invalidNetworkProblem e_str={e_str}"
        )
        # STAT == WRITE SEVR == INVALID
        if e_str == "stat=2 sevr=3":
            passed = True

    if passed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
    assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom, url_string=url_string)

    # Reset simulator
    def test_TC_98000(self):
        tc_no = 98000
        self.axisMr.setValueOnSimulator(tc_no, "fHighHardLimitPos", tc_no)

    #
    def test_TC_98001(self):
        tc_no = 98001
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98002(self):
        tc_no = 98002
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98003(self):
        tc_no = 98003
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98004(self):
        tc_no = 98004
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98005(self):
        tc_no = 98005
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98006(self):
        tc_no = 98006
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98007(self):
        tc_no = 98007
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98008(self):
        tc_no = 98008
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98009(self):
        tc_no = 98009
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98010(self):
        tc_no = 98010
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98011(self):
        tc_no = 98011
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98012(self):
        tc_no = 98012
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    def test_TC_98013(self):
        tc_no = 98013
        simulateNetworkProblem(self, tc_no, tc_no - 98000)

    # The last test case uses an invalid simulatedNetworkProblem
    # The simulator will not return "OK", and the PV will go into
    # an INVALID WRITE SEVR/STAT alarm
    def test_TC_98099(self):
        tc_no = 98099
        invalidNetworkProblem(self, tc_no, tc_no - 98000)
