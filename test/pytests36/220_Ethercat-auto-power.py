#!/usr/bin/env python

import datetime
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]

###

PwrOnDly = 6.0
globalPwrOffDly = 3.0
DLY = 4.0
BDST = 5.0


def lineno():
    return inspect.currentframe().f_back.f_lineno


def restorePwrSettings(self, tc_no, dly, bdst, pwrAuto, pwrOnDly, pwrOffDly):
    self.axisCom.put(".DLY", dly)
    self.axisCom.put(".BDST", bdst)
    self.axisCom.put("-PwrAuto", pwrAuto)
    self.axisCom.put("-PwrOnDly", pwrOnDly)
    self.axisCom.put("-PwrOffDly", pwrOffDly)


def do_220_autopower(self, tc_no, autopower, pwrOffDly):
    self.axisCom.putDbgStrToLOG("Start " + tc_no[0:20], wait=True)
    self.axisMr.setCNENandWait(tc_no, 0)
    self.axisCom.put("-PwrAuto", autopower)
    self.axisCom.put("-PwrOnDly", PwrOnDly)
    self.axisCom.put("-PwrOffDly", pwrOffDly)
    bdst = self.axisCom.get(".BDST")
    dly = self.axisCom.get(".DLY")
    destination = self.saved_LLM + bdst + 2 * (int(tc_no) - 2200) + 2 * autopower
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} destination={destination}"
    )
    self.axisMr.moveWait(tc_no, destination)

    if dly < pwrOffDly:
        # Make sure drive is still enabled
        power1 = self.axisCom.get(".CNEN", use_monitor=False)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Check drive is still enabled power1={int(power1)}"
        )
    else:
        power1 = 1
    time.sleep(globalPwrOffDly + pwrOffDly + 2.0)
    self.axisMr.waitForPowerOff(
        tc_no + "autopower", 2 * (globalPwrOffDly + float(pwrOffDly) + 2.0)
    )
    power2 = self.axisCom.get(".CNEN", use_monitor=False)
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Wait 8s and check drive is now disabled power2={int(power2)}"
    )
    restorePwrSettings(
        self,
        tc_no,
        self.saved_DLY,
        self.saved_BDST,
        self.saved_PwrAuto,
        self.saved_PwrOnDly,
        self.saved_PwrOffDly,
    )
    self.axisMr.setCNENandWait(tc_no, self.saved_CNEN)

    msta = int(self.axisCom.get(".MSTA"))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} power1={int(power1)} power2={int(power2)}"
    )
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}  msta={self.axisMr.getMSTAtext(msta)}"
    )
    if (power1 == 1) and (power2 == 0):
        testPassed = True
    else:
        testPassed = False
    if (msta & self.axisMr.MSTA_BIT_PROBLEM) != 0:
        testPassed = False
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)
    # self.axisCom.put('-DbgStrToLOG', "Start of " + os.path.basename(__file__)[0:20], wait=True)
    saved_DLY = axisCom.get(".DLY")
    saved_BDST = axisCom.get(".BDST")
    saved_LLM = axisCom.get(".LLM")
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")
    saved_PwrOnDly = axisCom.get("-PwrOnDly")
    saved_PwrOffDly = axisCom.get("-PwrOffDly")

    def test_TC_2000(self):
        tc_no = "2000"
        self.axisMr.powerOnHomeAxis(tc_no)

    def test_TC_2201(self):
        tc_no = "2201"

        # Enable power
        self.axisCom.putDbgStrToLOG("Start " + tc_no[0:20], wait=True)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Enable drive and move to LLM"
        )
        self.axisCom.put("-PwrAuto", 2)
        self.axisCom.put("-PwrOnDly", PwrOnDly)
        self.axisMr.setCNENandWait(tc_no, 1)
        destination = self.saved_LLM + 1
        self.axisMr.moveWait(tc_no, destination)
        restorePwrSettings(
            self,
            tc_no,
            self.saved_DLY,
            self.saved_BDST,
            self.saved_PwrAuto,
            self.saved_PwrOnDly,
            self.saved_PwrOffDly,
        )
        self.axisCom.putDbgStrToLOG("End   " + tc_no[0:20], wait=True)

    def test_TC_2202(self):
        tc_no = "2202"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} autopower "
        )
        do_220_autopower(self, tc_no, 1, globalPwrOffDly)

    def test_TC_2203(self):
        tc_no = "2203"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} autopower "
        )
        do_220_autopower(self, tc_no, 2, globalPwrOffDly)

    def test_TC_2204(self):
        tc_no = "2204"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} autopower "
        )
        self.axisCom.put(".DLY", DLY)
        self.axisCom.put(".BDST", BDST)
        do_220_autopower(self, tc_no, 2, 0.0)

    def test_TC_2205(self):
        tc_no = "2205"
        msta = int(self.axisCom.get(".MSTA", use_monitor=False))
        print(f"{tc_no} Error msta={self.axisMr.getMSTAtext(msta)}")
        if msta & self.axisMr.MSTA_BIT_PROBLEM:
            self.axisMr.resetAxis(tc_no)

        self.axisMr.setCNENandWait(tc_no, self.saved_CNEN)

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
