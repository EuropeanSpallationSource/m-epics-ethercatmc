#!/usr/bin/env python
#
# DMOV goes to 0 when VAL, DVAL, RVAL, RLV, TWF or TWR is written.
# The timestamp of DMOV=0 must not be older than the write.
# (The clocks of the IOC and of the test must be in sync)

import datetime
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]

polltime = 0.1
maxClockSkew = 0.005


def lineno():
    return inspect.currentframe().f_back.f_lineno


def dmovTimestampTC(self, tc_no, field_name, value):
    # Let the record be idle for a while, so that its timestamp is old
    time.sleep(1.0)
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    put_time = time.time()
    self.axisCom.put(field_name, value, wait=True, timeout=60.0)
    time_to_wait = 1.0  # wait max 1 second for the callbacks
    while time_to_wait > 0.0:
        num_change_cnt_dmov_true = (
            self.axisCom.get_change_cnts("dmov_true") - start_change_cnt_dmov_true
        )
        num_change_cnt_dmov_false = (
            self.axisCom.get_change_cnts("dmov_false") - start_change_cnt_dmov_false
        )
        if num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1:
            time_to_wait = 0.0
        else:
            time.sleep(polltime)
            time_to_wait -= polltime
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} {field_name}={value} num_change_cnt_dmov_false={num_change_cnt_dmov_false} num_change_cnt_dmov_true={num_change_cnt_dmov_true}"
    )
    testPassed = num_change_cnt_dmov_false == 1 and num_change_cnt_dmov_true == 1
    if testPassed:
        dmov_false_time = self.axisCom.get_change_timestamp("dmov_false")
        dmov_true_time = self.axisCom.get_change_timestamp("dmov_true")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} dmov_false-put={dmov_false_time - put_time:.3f} dmov_true-put={dmov_true_time - put_time:.3f}"
        )
        testPassed = (
            dmov_false_time >= put_time - maxClockSkew
            and dmov_true_time >= dmov_false_time
        )
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition = round((8 * llm + 2 * hlm) / 10)
    per50_UserPosition = round((5 * llm + 5 * hlm) / 10)

    def test_TC_1500(self):
        tc_no = "1500"
        self.axisMr.powerOnHomeAxis(tc_no)
        self.axisCom.put(".BDST", 0.0)
        assert self.axisMr.moveWait(tc_no, self.per10_UserPosition)

    def test_TC_1501(self):
        tc_no = "1501"
        dmovTimestampTC(self, tc_no, ".VAL", self.per20_UserPosition)

    def test_TC_1502(self):
        tc_no = "1502"
        dval = self.axisMr.calcDVALfromVAL(tc_no, self.per10_UserPosition)
        dmovTimestampTC(self, tc_no, ".DVAL", dval)

    def test_TC_1503(self):
        tc_no = "1503"
        rval = self.axisMr.calcRVALfromVAL(tc_no, self.per20_UserPosition)
        dmovTimestampTC(self, tc_no, ".RVAL", int(rval))

    def test_TC_1504(self):
        tc_no = "1504"
        rlv = self.per10_UserPosition - self.per20_UserPosition
        dmovTimestampTC(self, tc_no, ".RLV", rlv)

    def test_TC_1505(self):
        tc_no = "1505"
        self.axisCom.put(".TWV", 1.0)
        dmovTimestampTC(self, tc_no, ".TWF", 1)

    # Move to the current position: no movement, DMOV goes 1 0 1
    def test_TC_1506(self):
        tc_no = "1506"
        val = float(self.axisCom.get(".VAL"))
        dmovTimestampTC(self, tc_no, ".VAL", val)

    # Backlash
    def test_TC_1507(self):
        tc_no = "1507"
        self.axisCom.put(
            ".BDST", (self.per50_UserPosition - self.per10_UserPosition) / 4
        )
        dmovTimestampTC(self, tc_no, ".VAL", self.per50_UserPosition)
        self.axisCom.put(".BDST", 0.0)

    # JOGF at the high soft limit is refused, DMOV goes 1 0 1
    def test_TC_1508(self):
        tc_no = "1508"
        assert self.axisMr.moveWait(tc_no, self.hlm)
        dmovTimestampTC(self, tc_no, ".JOGF", 1)
        assert self.axisMr.moveWait(tc_no, self.per50_UserPosition)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
