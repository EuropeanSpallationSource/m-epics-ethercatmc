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

filnam = "960xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1


def readBackParamVerify(self, tc_no, field_name, expVal):
    maxTime = 5  # 5 seconds maximum to poll all parameters
    testPassed = False
    while maxTime > 0:
        actVal = int(self.axisCom.get(field_name))
        print(
            f"{tc_no}:{int(lineno())} {field_name} expVal={expVal:f} actVal={actVal:f}"
        )

        res = False
        if expVal == actVal:
            res = True
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} res={res}"
        )
        if res == True:
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    return False


def writeReadMotorFlag(self, tc_no, field_name="invalid_field_name", bit_mask=0):
    mflg_orig = int(self.axisCom.get(".MFLG"))

    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    if (mflg_orig & bit_mask) != 0:
        pv_val_first = 0
        pv_val_last = 1
        mflgExpFirst = mflg_orig & ~bit_mask
    else:
        pv_val_first = 1
        pv_val_last = 0
        mflgExpFirst = mflg_orig | bit_mask

    self.axisCom.put(field_name, pv_val_first)
    testPassedFirst = readBackParamVerify(self, tc_no, ".MFLG", mflgExpFirst)

    self.axisCom.put(field_name, pv_val_last)
    testPassedLast = readBackParamVerify(self, tc_no, ".MFLG", mflg_orig)

    testPassed = testPassedFirst and testPassedLast
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    return testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    vers = float(axisCom.get(".VERS"))
    if vers >= 7.04 and vers <= 7.09:
        axisCom.put(".SPAM", 255)
        hasMFLG = True

    def test_TC_9601(self):
        tc_no = 9601

        field_name = "-HomeOnLs"
        bit_mask = 1
        if self.hasMFLG == True:
            testPassed = writeReadMotorFlag(
                self, tc_no, field_name=field_name, bit_mask=bit_mask
            )
        assert testPassed

    def test_TC_9602(self):
        tc_no = 9602

        field_name = "-LsRampDown"
        bit_mask = 2
        if self.hasMFLG == True:
            testPassed = writeReadMotorFlag(
                self, tc_no, field_name=field_name, bit_mask=bit_mask
            )
        assert testPassed

    def test_TC_9603(self):
        tc_no = 9604

        field_name = "-NoStopOnLs"
        bit_mask = 4
        if self.hasMFLG == True:
            testPassed = writeReadMotorFlag(
                self, tc_no, field_name=field_name, bit_mask=bit_mask
            )
        assert testPassed

    def test_TC_9604(self):
        tc_no = 9604

        field_name = "-DrvUseEGU"
        bit_mask = 8
        if self.hasMFLG == True:
            testPassed = writeReadMotorFlag(
                self, tc_no, field_name=field_name, bit_mask=bit_mask
            )
        assert testPassed

    def test_TC_9605(self):
        tc_no = 9605

        field_name = "-AdjAfterHomed"
        bit_mask = 0x10
        if self.hasMFLG == True:
            testPassed = writeReadMotorFlag(
                self, tc_no, field_name=field_name, bit_mask=bit_mask
            )
        assert testPassed
