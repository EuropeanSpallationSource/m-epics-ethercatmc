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
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1


def readBackParamVerify(self, tc_no, field_name, expVal):
    maxTime = 5  # 5 seconds maximum to poll all parameters
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
        if res:
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    return False


def writeReadMotorFlag(self, tc_no, field_name="invalid_field_name", bit_mask=0):
    mflg_orig = int(self.axisCom.get(".MFLG"))
    field_sevr = self.axisCom.get(field_name + ".SEVR")
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    print(f"field_name={field_name} bit_mask={bit_mask} field_sevr={field_sevr}")
    if field_sevr != 0:
        self.axisCom.putDbgStrToLOG("Skipped " + str(tc_no), wait=True)
        return True

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
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    return testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_9601(self):
        tc_no = 9601

        field_name = "-HomeOnLs"
        bit_mask = 1
        testPassed = writeReadMotorFlag(
            self, tc_no, field_name=field_name, bit_mask=bit_mask
        )
        assert testPassed

    def test_TC_9602(self):
        tc_no = 9602

        field_name = "-LsRampDown"
        bit_mask = 2
        testPassed = writeReadMotorFlag(
            self, tc_no, field_name=field_name, bit_mask=bit_mask
        )
        assert testPassed

    def test_TC_9603(self):
        tc_no = 9604

        field_name = "-NoStopOnLs"
        bit_mask = 4
        testPassed = writeReadMotorFlag(
            self, tc_no, field_name=field_name, bit_mask=bit_mask
        )
        assert testPassed

    def test_TC_9604(self):
        tc_no = 9604

        field_name = "-DrvUseEGU"
        bit_mask = 8
        testPassed = writeReadMotorFlag(
            self, tc_no, field_name=field_name, bit_mask=bit_mask
        )
        assert testPassed

    def test_TC_9605(self):
        tc_no = 9605

        field_name = "-AdjAfterHomed"
        bit_mask = 0x10
        testPassed = writeReadMotorFlag(
            self, tc_no, field_name=field_name, bit_mask=bit_mask
        )
        assert testPassed

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
