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

###


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1

# Values to be used for test
# Note: Make sure to use different values to hae a good
# test coverage
myVMAX = 5.0  # maximimum velocity
myVELO = 4.0  # positioning velocity
myJVEL = 3.0  # jogging velocity
myBVEL = 2.0  # backlash velocity
myVBAS = 1.0  # base velocity

# Comparing floating points may fail because of rounding problems
maxdelta = 0.01


def InitVelocities(self, tc_no):
    # Prepare velocites
    self.axisCom.put(".VMAX", myVMAX)
    self.axisCom.put(".VELO", myVELO)
    self.axisCom.put(".JVEL", myJVEL)
    self.axisCom.put(".BVEL", myBVEL)
    self.axisCom.put(".VBAS", myVBAS)


def readBackParamVerify(self, tc_no, field_name, expVal):
    maxTime = 5  # 5 seconds maximum to poll all parameters
    testPassed = False
    maxDelta = math.fabs(expVal) * 0.02  # 2 % error tolerance margin
    while maxTime > 0:
        actVal = self.axisCom.get(field_name)
        print(
            f"{tc_no}:{int(lineno())} {field_name} expVal={expVal:f} actVal={actVal:f}"
        )

        res = self.axisMr.calcAlmostEqual(tc_no, expVal, actVal, maxDelta)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}:{int(lineno())} res={res}"
        )
        if (res == True) or (res != 0):
            return True
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime
    return False


def checkAllVelocities(self, tc_no, expVELO):
    urev = self.axisCom.get(".UREV")
    expS = self.axisCom.get(".VELO") / urev
    ok_S = readBackParamVerify(self, tc_no, ".S", expS)

    expSBAS = self.axisCom.get(".VBAS") / urev
    ok_SBAS = readBackParamVerify(self, tc_no, ".SBAS", expSBAS)

    expSBAK = self.axisCom.get(".BVEL") / urev
    ok_SBAK = readBackParamVerify(self, tc_no, ".SBAK", expSBAK)

    expSMAX = self.axisCom.get(".VMAX") / urev
    ok_SMAX = readBackParamVerify(self, tc_no, ".SMAX", expSMAX)

    ok_VELO = readBackParamVerify(self, tc_no, ".VELO", expVELO)

    print(
        f"{tc_no}: ok_S={ok_S} ok_SBAS={ok_SBAS} SBAK={ok_SBAK} SMAX={ok_SMAX} VELO={ok_VELO}"
    )
    return ok_S and ok_SBAS and ok_SBAK and ok_SMAX and ok_VELO


def changeResolutionCheckVelocities(self, tc_no, field_name, drvUseEGU=0):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    expVELO = self.axisCom.get(".VELO")
    oldFieldValue = self.axisCom.get(field_name)

    testPassed1 = checkAllVelocities(self, tc_no, expVELO)
    if drvUseEGU == 0:
        expVELO2 = expVELO / 2.0
    else:
        expVELO2 = expVELO
    self.axisCom.put(field_name, oldFieldValue / 2.0)
    tc_no = tc_no + 1
    testPassed2 = checkAllVelocities(self, tc_no, expVELO2)

    tc_no = tc_no + 1
    self.axisCom.put(field_name, oldFieldValue)
    testPassed3 = checkAllVelocities(self, tc_no, expVELO)
    testPassed = testPassed1 and testPassed2 and testPassed3
    self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)

    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    hasROlimit = 0
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    vers = float(axisCom.get(".VERS"))
    if vers >= 6.94 and vers <= 7.09:
        drvUseEGU_RB = axisCom.get("-DrvUseEGU-RB")
        drvUseEGU = 0
        if drvUseEGU_RB == 1:
            axisCom.put("-DrvUseEGU", drvUseEGU)

    def test_TC_97000(self):
        tc_no = 97000
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        InitVelocities(self, tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    # Change UREV VELO must change
    def test_TC_97010(self):
        tc_no = 970101
        changeResolutionCheckVelocities(self, tc_no, ".UREV")

    # Change MRES VELO must change
    def test_TC_97020(self):
        tc_no = 970201
        changeResolutionCheckVelocities(self, tc_no, ".MRES")

    # Change drvUseEGU = 1
    def test_TC_97030(self):
        tc_no = 97030
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        if self.drvUseEGU_RB != None:
            mflg = int(self.axisCom.get(".MFLG"))
            drvUseEGU = 1
            self.axisCom.put("-DrvUseEGU", drvUseEGU)
            expMFLG = mflg | 8  # Bit 3
            testPassed = readBackParamVerify(self, tc_no, ".MFLG", expMFLG)
            if testPassed:
                self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
            else:
                self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
            assert testPassed
        else:
            self.axisCom.putDbgStrToLOG("Skipped " + str(tc_no), wait=True)

    # Change UREV VELO must not change
    def test_TC_97040(self):
        tc_no = 970401
        if self.drvUseEGU_RB != None:
            changeResolutionCheckVelocities(
                self, tc_no, ".UREV", drvUseEGU=self.drvUseEGU_RB
            )

    # Change MRES VELO must not change
    def test_TC_97040(self):
        tc_no = 970401
        if self.drvUseEGU_RB != None:
            changeResolutionCheckVelocities(
                self, tc_no, ".MRES", drvUseEGU=self.drvUseEGU_RB
            )
