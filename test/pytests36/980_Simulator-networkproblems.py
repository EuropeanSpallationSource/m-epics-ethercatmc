#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

filnam = "980xx.py"
###

maxDelta = 0.5
timeout = 3.0

def lineno():
    return inspect.currentframe().f_back.f_lineno

def simulateNetworkProblem(self, tc_no, simulatedNetworkProblem):
    myHighHardLimitPos = tc_no
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    self.axisMr.setValueOnSimulator(tc_no, "fHighHardLimitPos", myHighHardLimitPos)
    self.axisMr.setValueOnSimulator(tc_no, "ads.simulatedNetworkProblem", simulatedNetworkProblem)
    passed = self.axisMr.waitForValueChanged(tc_no, "-CfgPMAX-RB", myHighHardLimitPos, maxDelta, timeout)
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
    axisMr = AxisMr(axisCom)

    # Reset simulator
#    def test_TC_9800(self):
#        tc_no = 9800
#        simulateNetworkProblem(self, tc_no, 0);

    #
    def test_TC_9801(self):
        tc_no = 9801
        simulateNetworkProblem(self, tc_no, tc_no - 9800);

    def test_TC_9802(self):
        tc_no = 9802
        simulateNetworkProblem(self, tc_no, tc_no - 9800);

