#!/usr/bin/env python

# test script to run into both limit switches

import datetime
import inspect
import math
import os
import sys
import time
import unittest

from AxisTestUtil import AxisTestUtil
import AxisComm as AxisComm

filnam = "010"


def lineno():
    return inspect.currentframe().f_back.f_lineno


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisTestUtil = AxisTestUtil(url_string, log_debug=True)
    axisComm = axisTestUtil.getAxis()

    # Read variables (floating point)
    def test_TC_01001(self):
        tc_no = "01001"
        rbv = float(self.axisComm.getActPos(tc_no=tc_no))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} rbv={rbv}"
        )

    # Read variables (boolean)
    def test_TC_01002(self):
        tc_no = "01002"
        enabled = self.axisComm.getEnabledStatus(tc_no=tc_no)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} enabled={enabled}"
        )

    def teardown_class(self):
        tc_no = "010999"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisTestUtil.close()
