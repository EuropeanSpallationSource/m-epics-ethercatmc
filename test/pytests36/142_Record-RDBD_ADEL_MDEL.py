#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
polltime = 0.05
maxdelta = 1e-6


def lineno():
    return inspect.currentframe().f_back.f_lineno


def wait_for_adel_mdel_half_rdbd(self, tc_no, expected_half_rdbd, timeout=3.0):
    while timeout > 0.0:
        rdbd = float(self.axisCom.get(".RDBD"))
        adel = float(self.axisCom.get(".ADEL"))
        mdel = float(self.axisCom.get(".MDEL"))
        if (
            abs(rdbd - 2.0 * expected_half_rdbd) <= maxdelta
            and abs(adel - expected_half_rdbd) <= maxdelta
            and abs(mdel - expected_half_rdbd) <= maxdelta
        ):
            return True

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} expected_half_rdbd={expected_half_rdbd:.6f} rdbd={rdbd:.6f} adel={adel:.6f} mdel={mdel:.6f} timeout={timeout:.2f}"
        )
        time.sleep(polltime)
        timeout -= polltime
    return False


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_1420(self):
        tc_no = "1420"
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

        old_rdbd = float(self.axisCom.get(".RDBD"))
        old_adel = float(self.axisCom.get(".ADEL"))
        old_mdel = float(self.axisCom.get(".MDEL"))

        testPassed = True
        test_rdbd = 0.246
        expected_half_rdbd = test_rdbd / 2.0

        try:
            self.axisCom.put(".RDBD", test_rdbd)
            testPassed = wait_for_adel_mdel_half_rdbd(
                self, tc_no, expected_half_rdbd
            )
        finally:
            self.axisCom.put(".RDBD", old_rdbd)
            self.axisCom.put(".ADEL", old_adel)
            self.axisCom.put(".MDEL", old_mdel)

        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed

    def test_TC_1421(self):
        tc_no = "1421"
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

        rdbd = float(self.axisCom.get(".RDBD"))
        expected_half_rdbd = rdbd / 2.0
        testPassed = wait_for_adel_mdel_half_rdbd(self, tc_no, expected_half_rdbd)

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
